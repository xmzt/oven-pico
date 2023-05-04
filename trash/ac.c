#include "ac.h"
#include "mainq.h"
#include "util.h"

#include "stdio.h"

//-----------------------------------------------------------------------------------------------------------------------
// globals

static uint32_t g_tick;
static uint32_t g_pwm16; // todo 16 bit
static uint16_t g_pwm16_0;
static uint32_t g_timeraw;
static uint32_t g_timeraw_0;
static uint32_t g_pio_result;

static uint32_t g_ts[3];

typedef struct {
	uint32_t ts;
	thunk_t thunk;
} timer_q_item_t;

//-----------------------------------------------------------------------------------------------------------------------
// gpio_ctrl_*

static const uint32_t gpio_ctrl_pwm_inv = UTIL_IO_BANK0_GPIO_CTRL_FUNCSEL_OUTOVER_OEOVER_INOVER_IRQOVER
	(IO_BANK0_GPIO0_CTRL_FUNCSEL_VALUE_PWM_A_0, IO_BANK0_GPIO0_CTRL_OUTOVER_VALUE_INVERT, 0, 0, 0);

static const uint32_t gpio_ctrl_pwm_lo = UTIL_IO_BANK0_GPIO_CTRL_FUNCSEL_OUTOVER_OEOVER_INOVER_IRQOVER
	(IO_BANK0_GPIO0_CTRL_FUNCSEL_VALUE_PWM_A_0, IO_BANK0_GPIO0_CTRL_OUTOVER_VALUE_LOW, 0, 0, 0);

static const uint32_t gpio_ctrl_pwm_hi = UTIL_IO_BANK0_GPIO_CTRL_FUNCSEL_OUTOVER_OEOVER_INOVER_IRQOVER
	(IO_BANK0_GPIO0_CTRL_FUNCSEL_VALUE_PWM_A_0, IO_BANK0_GPIO0_CTRL_OUTOVER_VALUE_HIGH, 0, 0, 0);

//-----------------------------------------------------------------------------------------------------------------------
// dma

static const uint32_t dma_ctrl_xfer = UTIL_DMA_CTRL_TRIG_EN_DSZ_INCR_INCW_RSZ_RSEL_TO_TREQ_IQ
	(1, DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD, 0, 0, 0, /*r0w1*/ 0, AC_DMA_A, 0x3F, 1);

static const uint32_t dma_ctrl_xfer_pio = UTIL_DMA_CTRL_TRIG_EN_DSZ_INCR_INCW_RSZ_RSEL_TO_TREQ_IQ
	(1, DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD, 0, 0, 0, /*r0w1*/ 0, AC_DMA_A, AC_PHASE_PIO_DREQ_RX, 1);

static const uint32_t dma_ctrl_chain = UTIL_DMA_CTRL_TRIG_EN_DSZ_INCR_INCW_RSZ_RSEL_TO_TREQ_IQ
	(1, DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD, 1, 1, 4, /*r0w1*/ 1, AC_DMA_B, 0x3F, 1);

static const util_dma_block_0_t dma_chain[] = {
	{ (uintptr_t)&AC_PWM_SLICE_PHASE->ctr, (uintptr_t)&g_pwm16, 1, dma_ctrl_xfer_pio },
	{ (uintptr_t)&timer_hw->timerawl, (uintptr_t)&g_timeraw, 1, dma_ctrl_xfer },
	{ (uintptr_t)&AC_PIO->rxf[AC_PHASE_PIO_SM_ID], (uintptr_t)&g_pio_result, 1, dma_ctrl_xfer },
	{ 0, 0, 0, 0 }
};

inline static void dma_chain_start(void) {
	dma_channel_hw_t *ch = &dma_hw->ch[AC_DMA_A];
	ch->read_addr = (uintptr_t)dma_chain;
	ch->write_addr = (uintptr_t)&dma_hw->ch[AC_DMA_B];
	ch->transfer_count = 4;
	ch->ctrl_trig = dma_ctrl_chain;
}

inline static void dma_chain_restart(void) {
	dma_channel_hw_t *ch = &dma_hw->ch[AC_DMA_A];
	ch->read_addr = (uintptr_t)dma_chain;
	ch->ctrl_trig = dma_ctrl_chain;
}

//-----------------------------------------------------------------------------------------------------------------------
// pio

inline static void ac_phase_pio_post_wait0(uint32_t result) {
	AC_PHASE_PIO->txf[AC_PHASE_PIO_SM_ID] = pio_encode_wait_pin(/*polarity*/ 0, /*pin*/ 0) | (result << 16);
}

inline static void ac_phase_pio_post_wait1(uint32_t result) {
	AC_PHASE_PIO->txf[AC_PHASE_PIO_SM_ID] = pio_encode_wait_pin(/*polarity*/ 1, /*pin*/ 0) | (result << 16);
}

static void ac_phase_in_result_pio_isr(void) {
	// hardware clears rxnempty

	g_phase_in_elapsed = AC_PHASE_PIO->rxf[AC_PHASE_SM_ID];
	if(-1 != g_phase_in_elapsed)
		g_phase_in_elapsed = AC_PHASE_IN_ELAPSED(g_phase_in_elapsed);
	mainq_pro1(&g_mainq, phase_mainq_fun);

	// set next timer
	util_irq_isr(AC_ALARM_IRQ, ac_phase_in_alarm_isr);
	timer_hw->alarm[AC_ALARM_ID] = timer_hw->timerawl + AC_PHASE_IN_ALARM_DUR;
}

static void ac_phase_start_nisr(void) {
	// prepare pio rx interrupt
	util_irq_isr(AC_PHASE_PIO_IRQ, ac_phase_in_result_pio_isr);
	nvic_hw->iser = 1 << AC_PHASE_PIO_IRQ;
	AC_PHASE_PIO_INTE = 1 << AC_PHASE_PIO_INTR_BIT;

	// disable interrupts: get initial timetamps and start pio
	uint32_t ints = save_and_disable_interrupts();
	g_ts[0] = AC_PWM_SLICE_PHASE->ctr;
	g_timeraw_0 = timer_hw->timerawl;
	iobank0_hw->io[AC_PIN_PHASE_OUT].ctrl = gpio_ctrl_pwm_hi;
	restore_interrupts(ints);

	g_ts[2] = g_ts[1] = g_ts[0];
}
	
//-----------------------------------------------------------------------------------------------------------------------
// mainq

static void phase_mainq_fun(void) {
	// pwm_d difference in pwm (use 16 bit math to handle counter wrapping)
	uint16_t pwm16_d = (uint16_t)g_pwm16 - g_pwm16_0;
	// convert timer interval to tick/pwm scale
	uint32_t tick_raw_d = 125 * (g_timeraw - g_timeraw_0);
	// refine tick_d: subtract pwm16_d, find nearest multiple of 0x10000, add pwm16_d back in
	uint32_t tick_d = (((tick_raw_d - pwm16_d) + 0x8000) & ~0xFFFF) + pwm16_d;
	int32_t tick_raw_e = tick_raw_d - tick_d;
	g_pwm16_0 = (uint16_t)g_pwm16;
	g_timeraw_0 = g_timeraw;
	
	printf("[ac %u] phase 0x%x pwm16=%u tick %u - %u = %d\n",
		   g_tick, g_pio_result, g_pwm16, tick_raw_d, tick_d, tick_raw_e);
	dma_chain_start();
}

//-----------------------------------------------------------------------------------------------------------------------
// isr

void ac_dma_b_isr(void) {
	dma_hw->intr = 1 << AC_DMA_B;

	g_tick++;
	// disable cancel timer
	UTIL_ALIAS_SET(timer_hw->armed) = 1 << AC_ALARM_ID;
	UTIL_ALIAS_SET(timer_hw->intr) = 1 << AC_ALARM_ID;

	mainq_pro1(&g_mainq, phase_mainq_fun);
}

static void nop_alarm_isr(void) {
 	UTIL_ALIAS_CLR(timer_hw->intr) = 1 << AC_ALARM_ID;
}

//-----------------------------------------------------------------------------------------------------------------------
// init

void ac_phase_pio_init_start(pio_hw_t *hw, uint sm_id) {
	pio_sm_hw_t *sm = &hw->sm[sm_id];
	
	util_pio_load(hw->instr_mem, AC_PHASE_PIO_CODE_A, ac_phase_pio_program.instructions, ac_phase_pio_program.length);
	
	// configure state machine registers
	//sm->clkdiv = UTIL_PIO_SM_CLKDIV_FRAC_INT(0, 1);
	sm->execctrl = UTIL_PIO_SM_EXECCTRL_STN_STS_WB_WT_OS_IOE_OES_JP_SPD_SEN
		(0, 0, AC_PHASE_PIO_CODE_A + ac_phase_pio_wrap_target, AC_PHASE_PIO_CODE_A + ac_phase_pio_wrap,
		 0, 0, 0, AC_PIN_PHASE_IN, 0, 0);
	sm->shiftctrl = UTIL_PIO_SM_SHIFTCTRL_APUSH_APULL_ISD_OSD_PUSHT_PULLT_FJT_FJR
		(1, 0, /*l0r1*/ 0, /*l0r1*/ 1, /*0->32*/ 0, /*0->32*/ 0, 0, 0);
	sm->pinctrl = UTIL_PIO_SM_PINCTRL_OB_SB_SSB_IB_OC_SC_SSC
		(0, 0, 0, 0, 0, 0, 0);
	
	// X = pre_sample_dur, OSR = sample_dur, start.
	hw->txf[sm_id] = AC_PHASE_IN_PRE_SAMPLE_DUR;
	hw->txf[sm_id] = AC_PHASE_IN_SAMPLE_DUR;
	sm->instr = pio_encode_pull(/*if_empty*/ false, /*block*/ false);
	sm->instr = pio_encode_mov(/*dest*/ pio_x, /*src*/ pio_osr);
	sm->instr = pio_encode_pull(/*if_empty*/ false, /*block*/ false);
	sm->instr = pio_encode_jmp(AC_PHASE_PIO_CODE_A);
	hw->ctrl = g_pio0_ctrl_sme_set(1 << sm_id);
}

void ac_init(void) {
 	g_tick = 0;
	//g_pwm16
	g_pwm16_0 = 0;
	//g_timeraw
	g_timeraw_0 = 0;
	//g_pio_result
	
	//-------------------------------------------------------------------------------------------------------------------
	// pwm (125 MHz) for all ac pins, syncd

	// defaults for all slices
	//AC_PWM_SLICE->div = UTIL_PWM_DIV_FRAC_INT(0, 1);
	//AC_PWM_SLICE->top = 0xFFFF;
	//AC_PWM_SLICE->csr = UTIL_PWM_CSR_EN_PHCORRECT_AINV_BINV_DIVMODE_PHRET_PHADV
	//    (0, 0, 0, 0, 0, 0, 0);

	UTIL_ALIAS_SET(pwm_hw->en) = 1 << AC_PWM_SLICE_ID_PHASE | 1 << AC_PWM_SLICE_ID_OUT01 | 1 << AC_PWM_SLICE_ID_OUT23;

	//-------------------------------------------------------------------------------------------------------------------
	// dma
	
	//dma_chain_start();

	//-------------------------------------------------------------------------------------------------------------------
	// pio
	
	ac_phase_pio_init_start(AC_PIO, AC_PHASE_PIO_SM_ID);

	//-------------------------------------------------------------------------------------------------------------------
	// timer

	UTIL_ALIAS_SET(timer_hw->inte) = 1 << AC_ALARM_ID;
	//util_irq_isr(AC_ALARM_IRQ, nop_alarm_isr);
	nvic_hw->iser = 1 << AC_ALARM_IRQ;

	//-------------------------------------------------------------------------------------------------------------------
	// pads prepare

	padsbank0_hw->io[AC_PIN_PHASE_IN] = UTIL_PADS_BANK0_GPIO_OD_IE_DRIVE_PUE_PDE_SCHMITT_SLEWFAST
		(1, 1, 0, 0, 1, 1, 0);
	padsbank0_hw->io[AC_PIN_PHASE_OUT] = UTIL_PADS_BANK0_GPIO_OD_IE_DRIVE_PUE_PDE_SCHMITT_SLEWFAST
		(0, 1, PADS_BANK0_GPIO0_DRIVE_VALUE_12MA, 0, 0, 1, 0);
	for(uint i = AC_PIN_OUT4, j = i + 4; i < j; i++) {
		padsbank0_hw->io[i] = UTIL_PADS_BANK0_GPIO_OD_IE_DRIVE_PUE_PDE_SCHMITT_SLEWFAST
			(0, 1, PADS_BANK0_GPIO0_DRIVE_VALUE_12MA, 0, 0, 1, 0);
	}
	
	//-------------------------------------------------------------------------------------------------------------------
	// pads funcsel

	iobank0_hw->io[AC_PIN_PHASE_IN].ctrl = UTIL_IO_BANK0_GPIO_CTRL_FUNCSEL_OUTOVER_OEOVER_INOVER_IRQOVER
		(IO_BANK0_GPIO0_CTRL_FUNCSEL_VALUE_PIO0_0, 0, 0, 0, 0);
	iobank0_hw->io[AC_PIN_PHASE_OUT].ctrl = gpio_ctrl_pwm_lo;
	for(uint i = AC_PIN_OUT4, j = i + 4; i < j; i++) {
		iobank0_hw->io[i].ctrl = gpio_ctrl_pwm_lo;
	}

	//-------------------------------------------------------------------------------------------------------------------
	// start

	ac_phase_pio_post_wait1(PHASE_STATE_WAIT1_0);
}
