#include "nmtc.h"
#include "mainq.h"

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/structs/dma.h"
#include "hardware/sync.h"
#include <stdio.h>

#include "nmtc.pio.h"

//-----------------------------------------------------------------------------------------------------------------------
// globals

static uint32_t g_tick;
static uint32_t g_dma_irq_n;
static uint32_t g_pio_irq_n;
static nmtc_q_item_t *g_q_head;
static nmtc_q_item_t *g_q_tail;

#define NMTC_CMDV_Z 40
static uint32_t g_cmdV[NMTC_CMDV_Z];
static nmtc_q_item_t g_q_item;

static void ready_mainq_fun() {
	printf("[nmtc %u] ready pio_irq_n=%u dma_irq_n=%u\n", g_tick, g_pio_irq_n, g_dma_irq_n);
}

//-----------------------------------------------------------------------------------------------------------------------
// constants

static const uint32_t start_cmdV[] = {
	NMTC_PIO_DELAY_0, NMTC_PIO_INST_W(NMTC_START_FUNC, 0),
	NMTC_PIO_DELAY_1, NMTC_PIO_INST_W(NMTC_START_FUNC, 0),
	NMTC_PIO_DELAY_2, NMTC_PIO_INST_W(NMTC_START_FUNC, 1),
};
#define START_CMDV_N (sizeof(start_cmdV) / sizeof(*start_cmdV))

static const uint32_t run0_cmdV[] = {
	NMTC_PIO_INST_W(NMTC_DISPLAY_INST | NMTC_DISPLAY_OFF, 0),
	NMTC_PIO_INST_W(NMTC_CLEAR_INST, 0),
	NMTC_PIO_INST_W(NMTC_ENTRY_MODE_INST | NMTC_ENTRY_MODE_INC, 0),
	NMTC_PIO_INST_W(NMTC_DISPLAY_INST | NMTC_DISPLAY_ON | NMTC_DISPLAY_CURSOR_ON, 0),
};
#define RUN0_CMDV_N (sizeof(run0_cmdV) / sizeof(*run0_cmdV))

//-----------------------------------------------------------------------------------------------------------------------
// q

static void nmtc_q_head_go() {
	g_q_head->status = 2;
	dma_hw->ch[NMTC_DMA_B].read_addr = g_q_head->read_addr;
	dma_hw->ch[NMTC_DMA_B].write_addr = (io_rw_32)&NMTC_PIO->txf[NMTC_PIO_SM_ID];
	dma_hw->ch[NMTC_DMA_B].transfer_count = g_q_head->transfer_count;
	dma_hw->ch[NMTC_DMA_B].ctrl_trig = UTIL_DMA_CTRL_TRIG_EN_DSZ_INCR_INCW_RSZ_RSEL_TO_TREQ_IQ
		(1, DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD, 1, 0, 0, 0, NMTC_DMA_B, NMTC_PIO_DREQ_TX, 0);
}

void nmtc_q_item_post_nirq(nmtc_q_item_t *self, uint32_t read_addr, uint32_t transfer_count) {
	self->q_next = (void*)0;
	self->read_addr = read_addr;
	self->transfer_count = transfer_count;
	self->status = 1;
	uint32_t ints = save_and_disable_interrupts();
	if(g_q_head)
		g_q_tail = g_q_tail->q_next = self;
	else {
		g_q_head = g_q_tail = self;
		nmtc_q_head_go();
	}
	restore_interrupts(ints);
}

uint32_t * nmtc_cmdV_from_u8(uint32_t *dst, const uint8_t *src) {
	uint8_t ch;
	for( ; (ch = *src); ++src) {
		if(! (0x80 & ch))
			*dst++ = NMTC_PIO_INST_W(NMTC_DATA_WRITE_INST | ch, 0);
		else if(0x90 > ch)
			*dst++ = NMTC_PIO_INST_W(NMTC_DDRAM_ADR_INST | NMTC_DDRAM_ADR_LINE1 + ch - 0x80, 0);
		else
			*dst++ = NMTC_PIO_INST_W(NMTC_DDRAM_ADR_INST | NMTC_DDRAM_ADR_LINE2 + ch - 0x90, 0);
	}
	return dst;
}

//-----------------------------------------------------------------------------------------------------------------------
// i/o

static void run_q_fin_dma_irq(void) {
	dma_hw->intr = 1<<NMTC_DMA_B;

	g_tick++;
	g_dma_irq_n++;

	// syncd inside irq
	g_q_head->status = 0;
	if((g_q_head = g_q_head->q_next))
		nmtc_q_head_go();
}

static void run0_fin_dma_irq(void) {
	dma_hw->intr = 1<<NMTC_DMA_B;
	
	g_tick++;
	g_dma_irq_n++;
	util_irq_hand(NMTC_DMA_IRQ, run_q_fin_dma_irq);
	nmtc_q_head_go(); // ASSERT init has preloaded g_q_head
 	mainq_pro1(&g_mainq, ready_mainq_fun);
}

static void nmtc_start_fin_pio_irq(void) {
	NMTC_PIO->irq = 1<<NMTC_PIO_PIOIRQ;

	g_tick++;
	g_pio_irq_n++;
	nmtc_run_from_start();
}

//-----------------------------------------------------------------------------------------------------------------------
// init start run

void nmtc_init() {
	g_tick = 0;
	g_dma_irq_n = 0;
	g_pio_irq_n = 0;

	// prepare first q item
	uint8_t str[NMTC_CMDV_Z];
	sprintf(str, "\x9C%04X", g_rando & 0xFFFF);
	g_q_item.q_next = (void*)0;
	g_q_item.read_addr = (uint32_t)g_cmdV;
	g_q_item.transfer_count = nmtc_cmdV_from_u8(g_cmdV, str) - g_cmdV;
	g_q_item.status = 1;
	g_q_tail = g_q_head = &g_q_item;
	
	pio_hw_t *hw = NMTC_PIO;
	pio_sm_hw_t *sm = NMTC_PIO_SM;
	uint sm_id = NMTC_PIO_SM_ID;
	
	// load code
	util_pio_load(hw->instr_mem, NMTC_PIO_CODE_A, nmtc_pio_start_program.instructions, nmtc_pio_start_program.length);

	// configure state machine registers
	sm->clkdiv = UTIL_PIO_SM_CLKDIV_F_I(NMTC_PIO_CLKDIV_FRAC, NMTC_PIO_CLKDIV_INT);
	sm->execctrl = UTIL_PIO_SM_EXECCTRL_STN_STS_WB_WT_OS_IOE_OES_JP_SPD_SEN
		(0, 0, NMTC_PIO_CODE_A + nmtc_pio_start_wrap_target, NMTC_PIO_CODE_A + nmtc_pio_start_wrap, 0, 0, 0, 0, 0, 0);
	sm->shiftctrl = UTIL_PIO_SM_SHIFTCTRL_APUSH_APULL_ISD_OSD_PUSHT_PULLT_FJT_FJR
		(0, 0, /*left*/0, /*right*/1, /*32*/0, /*32*/0, 0, 0);
	sm->pinctrl = UTIL_PIO_SM_PINCTRL_OB_SB_SSB_IB_OC_SC_SSC
		(NMTC_PIN_DB8_RS_RW_EN, NMTC_PIN_DB8_RS_RW_EN + 10, 0, NMTC_PIN_DB8_RS_RW_EN, 10, 1, 0);
	
	// set pindirs, set PC, start the state machine
	hw->txf[sm_id] = NMTC_PIO_PINDIRS_W;
	sm->instr = pio_encode_pull(/*if_empty*/ false, /*block*/ false);
	sm->instr = pio_encode_out(pio_pindirs, 10);
	sm->instr = pio_encode_set(pio_pindirs, 1);
	sm->instr = pio_encode_jmp(NMTC_PIO_CODE_A);
	hw->ctrl = g_pio0_ctrl_sme_set(1<<sm_id);

	// pio interrupt
	util_irq_hand_iser(NMTC_PIO_IRQ, nmtc_start_fin_pio_irq);
	NMTC_PIO_INTE = 1<<(8+NMTC_PIO_PIOIRQ);
}

void nmtc_start() {
	dma_hw->ch[NMTC_DMA_B].read_addr = (io_rw_32)start_cmdV;
	dma_hw->ch[NMTC_DMA_B].write_addr = (io_rw_32)&NMTC_PIO->txf[NMTC_PIO_SM_ID];
	dma_hw->ch[NMTC_DMA_B].transfer_count = START_CMDV_N;
	dma_hw->ch[NMTC_DMA_B].ctrl_trig = UTIL_DMA_CTRL_TRIG_EN_DSZ_INCR_INCW_RSZ_RSEL_TO_TREQ_IQ
		(1, DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD, 1, 0, 0, 0, NMTC_DMA_B, NMTC_PIO_DREQ_TX, 0);
}

void nmtc_run_from_start() {
	pio_hw_t *hw = NMTC_PIO;
	pio_sm_hw_t *sm = NMTC_PIO_SM;
	uint sm_id = NMTC_PIO_SM_ID;

	// stop state machine (only this one)
	hw->ctrl = g_pio0_ctrl_sme_clr(1<<sm_id);

	util_pio_load(hw->instr_mem, NMTC_PIO_CODE_A, nmtc_pio_run_program.instructions, nmtc_pio_run_program.length);
	
	sm->execctrl = UTIL_PIO_SM_EXECCTRL_STN_STS_WB_WT_OS_IOE_OES_JP_SPD_SEN
		(0, 0, NMTC_PIO_CODE_A + nmtc_pio_run_wrap_target, NMTC_PIO_CODE_A + nmtc_pio_run_wrap,
		 0, 0, 0, NMTC_PIN_DB8_RS_RW_EN + 7, 0, 0);

	// set X, set PC, start the state machine
	hw->txf[sm_id] = NMTC_PIO_RUN_X;
	sm->instr = pio_encode_pull(/*if_empty*/ false, /*block*/ false);
	sm->instr = pio_encode_mov(/*dest*/ pio_x, /*src*/ pio_osr);
	sm->instr = pio_encode_jmp(NMTC_PIO_CODE_A);
	hw->ctrl = g_pio0_ctrl_sme_set(1<<sm_id);

	// dma interrupt. clear existing interrupt from previous transfer.  
	dma_hw->intr = 1<<NMTC_DMA_B;
	util_irq_hand_iser(NMTC_DMA_IRQ, run0_fin_dma_irq);
	NMTC_DMA_INTE = 1<<NMTC_DMA_B;

	// dma
	dma_hw->ch[NMTC_DMA_B].read_addr = (io_rw_32)run0_cmdV;
	dma_hw->ch[NMTC_DMA_B].write_addr = (io_rw_32)&NMTC_PIO->txf[NMTC_PIO_SM_ID];
	dma_hw->ch[NMTC_DMA_B].transfer_count = RUN0_CMDV_N;
	dma_hw->ch[NMTC_DMA_B].ctrl_trig = UTIL_DMA_CTRL_TRIG_EN_DSZ_INCR_INCW_RSZ_RSEL_TO_TREQ_IQ
		(1, DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD, 1, 0, 0, 0, NMTC_DMA_B, NMTC_PIO_DREQ_TX, 0);
}
