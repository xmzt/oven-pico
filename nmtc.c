#include "nmtc.h"
#include "mainq.h"
#include "util.h"

#include "stdio.h"

//-----------------------------------------------------------------------------------------------------------------------
// globals

static uint32_t g_tick;
isr_t *g_nmtc_dma_b_isr;
static uint32_t g_start_res;
static nmtc_q_item_t *g_q_head;
static nmtc_q_item_t *g_q_tail;

#define NMTC_CMDV_Z 40
static uint32_t g_cmdV[NMTC_CMDV_Z];
static nmtc_q_item_t g_q_item;

static void ready_mainq_fun(void) {
	printf("[nmtc %u] ready\n", g_tick);
}

//-----------------------------------------------------------------------------------------------------------------------
// dma

static const uint32_t dma_ctrl_rx = UTIL_DMA_CTRL_TRIG_EN_DSZ_INCR_INCW_RSZ_RSEL_TO_TREQ_IQ
	(1, DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD, 0, 1, 0, /*r0w1*/ 0, NMTC_DMA_A, NMTC_PIO_DREQ_RX, 1);

static const uint32_t dma_ctrl_tx = UTIL_DMA_CTRL_TRIG_EN_DSZ_INCR_INCW_RSZ_RSEL_TO_TREQ_IQ
	(1, DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD, 1, 0, 0, /*r0w1*/ 0, NMTC_DMA_A, NMTC_PIO_DREQ_TX, 1);

static const uint32_t dma_ctrl_tx_nchain = UTIL_DMA_CTRL_TRIG_EN_DSZ_INCR_INCW_RSZ_RSEL_TO_TREQ_IQ
	(1, DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD, 1, 0, 0, /*r0w1*/ 0, NMTC_DMA_B, NMTC_PIO_DREQ_TX, 1);

static const uint32_t dma_ctrl_chain = UTIL_DMA_CTRL_TRIG_EN_DSZ_INCR_INCW_RSZ_RSEL_TO_TREQ_IQ
	(1, DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD, 1, 1, 4, /*r0w1*/ 1, NMTC_DMA_B, 0x3F, 1);

static const uint32_t start_cmdV[] = {
	NMTC_PIO_DELAY_0, NMTC_PIO_INST_W(NMTC_START_FUNC, 0),
	NMTC_PIO_DELAY_1, NMTC_PIO_INST_W(NMTC_START_FUNC, 0),
	NMTC_PIO_DELAY_2, NMTC_PIO_INST_W(NMTC_START_FUNC, 0x69),
};
#define START_CMDV_N (sizeof(start_cmdV) / sizeof(*start_cmdV))

static const util_dma_block_0_t dma_chain_start[] = {
	{ (uintptr_t)start_cmdV, (uintptr_t)&NMTC_PIO->txf[NMTC_PIO_SM_ID], START_CMDV_N, dma_ctrl_tx },
	{ (uintptr_t)&NMTC_PIO->rxf[NMTC_PIO_SM_ID], (uintptr_t)&g_start_res, 1, dma_ctrl_rx },
	{ 0, 0, 0, 0 }
};

static const uint32_t run0_cmdV[] = {
	NMTC_PIO_INST_W(NMTC_DISPLAY_INST | NMTC_DISPLAY_OFF, 0),
	NMTC_PIO_INST_W(NMTC_CLEAR_INST, 0),
	NMTC_PIO_INST_W(NMTC_ENTRY_MODE_INST | NMTC_ENTRY_MODE_INC, 0),
	NMTC_PIO_INST_W(NMTC_DISPLAY_INST | NMTC_DISPLAY_ON | NMTC_DISPLAY_CURSOR_ON, 0),
	NMTC_PIO_INST_W(NMTC_DDRAM_ADR_INST | NMTC_DDRAM_ADR_LINE1 + 3, 0),
	NMTC_PIO_INST_W(NMTC_DATA_WRITE_INST | 'H', 0),
};
#define RUN0_CMDV_N (sizeof(run0_cmdV) / sizeof(*run0_cmdV))

static const util_dma_block_0_t dma_chain_run0[] = {
	{ (uintptr_t)run0_cmdV, (uintptr_t)&NMTC_PIO->txf[NMTC_PIO_SM_ID], RUN0_CMDV_N, dma_ctrl_tx },
	{ 0, 0, 0, 0 }
};

static util_dma_block_0_t dma_chain_run1[] = {
	{ /*undef*/ 0, (uintptr_t)&NMTC_PIO->txf[NMTC_PIO_SM_ID], /*undef*/ 0, dma_ctrl_tx },
	{ 0, 0, 0, 0 }
};
//-----------------------------------------------------------------------------------------------------------------------
// queue

static void nmtc_q_head_go(void) {
	g_q_head->status = 2;

	dma_chain_run1[0].read_addr = g_q_head->read_addr;
	dma_chain_run1[0].transfer_count = g_q_head->transfer_count;

	dma_hw->ch[NMTC_DMA_A].read_addr = (uintptr_t)dma_chain_run1;
	dma_hw->ch[NMTC_DMA_A].write_addr = (uintptr_t)&dma_hw->ch[NMTC_DMA_B];
	dma_hw->ch[NMTC_DMA_A].transfer_count = 4;
	dma_hw->ch[NMTC_DMA_A].ctrl_trig = dma_ctrl_chain;
}

void nmtc_q_item_post_nisr(nmtc_q_item_t *self, uint32_t read_addr, uint32_t transfer_count) {
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
// isr

static void run_q_fin_dma_b_isr(void) {
	dma_hw->intr = 1 << NMTC_DMA_B;

	g_tick++;
	nmtc_q_item_t *head = g_q_head;
	if((g_q_head = g_q_head->q_next))
		nmtc_q_head_go();
	// mark item as not in use 
	head->status = 0;
}

static void run0_fin_dma_b_isr(void) {
	dma_hw->intr = 1 << NMTC_DMA_B;
	
	g_tick++;
	g_nmtc_dma_b_isr = run_q_fin_dma_b_isr;
	nmtc_q_head_go(); // ASSERT init has preloaded g_q_head
	mainq_pro1(&g_mainq, ready_mainq_fun);
}

static void start_fin_dma_b_isr(void) {
	dma_hw->intr = 1 << NMTC_DMA_B;
	g_tick++;
 	mainq_pro1(&g_mainq, nmtc_start_fin);
}

//-----------------------------------------------------------------------------------------------------------------------
// init start run

void nmtc_pio_start_init_start(pio_hw_t *hw, uint sm_id) {
	pio_sm_hw_t *sm = &hw->sm[sm_id];

	util_pio_load(hw->instr_mem, NMTC_PIO_CODE_A, nmtc_pio_start_program.instructions, nmtc_pio_start_program.length);

	// configure state machine registers
	sm->clkdiv = UTIL_PIO_SM_CLKDIV_FRAC_INT(NMTC_PIO_CLKDIV_FRAC, NMTC_PIO_CLKDIV_INT);
	sm->execctrl = UTIL_PIO_SM_EXECCTRL_STN_STS_WB_WT_OS_IOE_OES_JP_SPD_SEN
		(0, 0, NMTC_PIO_CODE_A + nmtc_pio_start_wrap_target, NMTC_PIO_CODE_A + nmtc_pio_start_wrap, 0, 0, 0, 0, 0, 0);
	sm->shiftctrl = UTIL_PIO_SM_SHIFTCTRL_APUSH_APULL_ISD_OSD_PUSHT_PULLT_FJT_FJR
		(1, 1, /*l0r1*/0, /*l0r1*/1, /*0->32*/0, /*0->32*/0, 0, 0);
	sm->pinctrl = UTIL_PIO_SM_PINCTRL_OB_SB_SSB_IB_OC_SC_SSC
		(NMTC_PIN_DB8_RS_RW_EN, NMTC_PIN_DB8_RS_RW_EN + 10, 0, NMTC_PIN_DB8_RS_RW_EN, 10, 1, 0);
	
	// set pindirs, set PC, start the state machine
	hw->txf[sm_id] = NMTC_PIO_PINDIRS_W;
	//sm->instr = pio_encode_pull(/*if_empty*/ false, /*block*/ false);
	sm->instr = pio_encode_out(pio_pindirs, 32);
	sm->instr = pio_encode_set(pio_pindirs, 1);
	sm->instr = pio_encode_jmp(NMTC_PIO_CODE_A + nmtc_pio_start_wrap_target);
	hw->ctrl = g_pio0_ctrl_sme_set(1 << sm_id);
}

void nmtc_pio_run_init_restart(pio_hw_t *hw, uint sm_id) {
	pio_sm_hw_t *sm = &hw->sm[sm_id];

	// stop state machine (only this one)
	hw->ctrl = g_pio0_ctrl_sme_clr(1 << sm_id);

	util_pio_load(hw->instr_mem, NMTC_PIO_CODE_A, nmtc_pio_run_program.instructions, nmtc_pio_run_program.length);
	
	sm->execctrl = UTIL_PIO_SM_EXECCTRL_STN_STS_WB_WT_OS_IOE_OES_JP_SPD_SEN
		(0, 0, NMTC_PIO_CODE_A + nmtc_pio_run_wrap_target, NMTC_PIO_CODE_A + nmtc_pio_run_wrap,
		 0, 0, 0, NMTC_PIN_DB8_RS_RW_EN + 7, 0, 0);
	sm->shiftctrl = UTIL_PIO_SM_SHIFTCTRL_APUSH_APULL_ISD_OSD_PUSHT_PULLT_FJT_FJR
		(0, 0, /*l0r1*/0, /*l0r1*/1, /*0->32*/0, /*0->32*/0, 0, 0);

	// set X, set PC, start the state machine
	hw->txf[sm_id] = NMTC_PIO_RUN_X;
	sm->instr = pio_encode_pull(/*if_empty*/ false, /*block*/ false);
	sm->instr = pio_encode_mov(/*dest*/ pio_x, /*src*/ pio_osr);
	sm->instr = pio_encode_jmp(NMTC_PIO_CODE_A + nmtc_pio_run_wrap_target);
	hw->ctrl = g_pio0_ctrl_sme_set(1 << sm_id);
}

void nmtc_start_fin(void) {
	printf("[nmtc %u] start_fin %p\n", g_tick, g_start_res);
	
	nmtc_pio_run_init_restart(NMTC_PIO, NMTC_PIO_SM_ID);

	g_nmtc_dma_b_isr = run0_fin_dma_b_isr;
	dma_hw->ch[NMTC_DMA_A].read_addr = (uintptr_t)dma_chain_run0;
	dma_hw->ch[NMTC_DMA_A].write_addr = (uintptr_t)&dma_hw->ch[NMTC_DMA_B];
	dma_hw->ch[NMTC_DMA_A].transfer_count = 4;
	dma_hw->ch[NMTC_DMA_A].ctrl_trig = dma_ctrl_chain;
}

void nmtc_init(void) {
 	g_tick = 0;

	// prepare first q item
	uint8_t str[NMTC_CMDV_Z];
	sprintf(str, "\x9C%04X", g_rando & 0xFFFF);
	g_q_item.q_next = (void*)0;
	g_q_item.read_addr = (uint32_t)g_cmdV;
	g_q_item.transfer_count = nmtc_cmdV_from_u8(g_cmdV, str) - g_cmdV;
	g_q_item.status = 1;
	g_q_tail = g_q_head = &g_q_item;
	
	nmtc_pio_start_init_start(NMTC_PIO, NMTC_PIO_SM_ID);

	// pads prepare
	for(uint i = NMTC_PIN_DB8_RS_RW_EN, j = i + 11; i < j; i++) {
		padsbank0_hw->io[i] = UTIL_PADS_BANK0_GPIO_OD_IE_DRIVE_PUE_PDE_SCHMITT_SLEWFAST
			(0, 1, PADS_BANK0_GPIO0_DRIVE_VALUE_4MA, 0, 0, 1, 0);
	}

	// pads funcsel
	for(uint i = NMTC_PIN_DB8_RS_RW_EN, j = i + 11; i < j; i++) {
		iobank0_hw->io[i].ctrl = UTIL_IO_BANK0_GPIO_CTRL_FUNCSEL_OUTOVER_OEOVER_INOVER_IRQOVER
			(IO_BANK0_GPIO0_CTRL_FUNCSEL_VALUE_PIO0_0, 0, 0, 0, 0);
	}

	// dma init and trig on pio
	g_nmtc_dma_b_isr = start_fin_dma_b_isr;
	dma_hw->ch[NMTC_DMA_A].read_addr = (uintptr_t)dma_chain_start;
	dma_hw->ch[NMTC_DMA_A].write_addr = (uintptr_t)&dma_hw->ch[NMTC_DMA_B];
	dma_hw->ch[NMTC_DMA_A].transfer_count = 4;
	dma_hw->ch[NMTC_DMA_A].ctrl_trig = dma_ctrl_chain;
}
