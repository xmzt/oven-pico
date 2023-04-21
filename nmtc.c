#include "nmtc.h"
#include "log.h"

#include "hardware/structs/dma.h"

static const uint32_t nmtc_start_cmdV[] = {
	NMTC_PIO_DELAY_0, NMTC_PIO_INST_W(NMTC_START_FUNC, 0),
	NMTC_PIO_DELAY_1, NMTC_PIO_INST_W(NMTC_START_FUNC, 0),
	NMTC_PIO_DELAY_2, NMTC_PIO_INST_W(NMTC_START_FUNC, 1),
};

#define NMTC_START_CMDV_N (sizeof(nmtc_start_cmdV) / sizeof(*nmtc_start_cmdV))

static const uint32_t nmtc_run0_cmdV[] = {
	NMTC_PIO_INST_W(NMTC_DISPLAY_INST | NMTC_DISPLAY_OFF, 0),
	NMTC_PIO_INST_W(NMTC_CLEAR_INST, 0),
	NMTC_PIO_INST_W(NMTC_ENTRY_MODE_INST | NMTC_ENTRY_MODE_INC, 0),
	NMTC_PIO_INST_W(NMTC_DISPLAY_INST | NMTC_DISPLAY_ON | NMTC_DISPLAY_CURSOR_ON, 0),
	NMTC_PIO_INST_W(NMTC_DDRAM_ADR_INST | NMTC_DDRAM_ADR_LINE1, 0),
	NMTC_PIO_INST_W(NMTC_DATA_WRITE_INST | '?', 0),
};

#define NMTC_RUN0_CMDV_N (sizeof(nmtc_run0_cmdV) / sizeof(*nmtc_run0_cmdV))

//-----------------------------------------------------------------------------------------------------------------------
// i/o

static void nmtc_pio_run0_fin_irq(void) {
	// clear interrupt and log
	NMTC_PIO->irq = 1<<NMTC_PIO_PIOIRQ;
	LOG_PUSH(log_nmtc_pio_run0_fin);
}

static void nmtc_pio_start_fin_irq(void) {
	//clear interrupt
	NMTC_PIO->irq = 1<<NMTC_PIO_PIOIRQ;
	//LOG_PUSH(log_nmtc_pio_start_fin);
	nmtc_run_from_start();
}

//-----------------------------------------------------------------------------------------------------------------------
// display

void nmtc_init_start() {
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
}

void nmtc_start() {
	// pio interrupt
	NMTC_PIO_INTE = 1<<(8+NMTC_PIO_PIOIRQ);
	util_irq_hand_iser(NMTC_PIO_IRQ, nmtc_pio_start_fin_irq);

	dma_hw->ch[NMTC_DMA_A].read_addr = (io_rw_32)nmtc_start_cmdV;
	dma_hw->ch[NMTC_DMA_A].write_addr = (io_rw_32)&NMTC_PIO->txf[NMTC_PIO_SM_ID];
	dma_hw->ch[NMTC_DMA_A].transfer_count = NMTC_START_CMDV_N;
	dma_hw->ch[NMTC_DMA_A].ctrl_trig = UTIL_DMA_CTRL_TRIG_EN_DSZ_INCR_INCW_RSZ_RSEL_TO_TREQ_IQ
		(1, DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD, 1, 0, 0, 0, NMTC_DMA_A, NMTC_PIO_DREQ_TX, 0);
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

	// pio interrupt update
	util_irq_hand_iser(NMTC_PIO_IRQ, nmtc_pio_run0_fin_irq);

	dma_hw->ch[NMTC_DMA_A].read_addr = (io_rw_32)nmtc_run0_cmdV;
	dma_hw->ch[NMTC_DMA_A].write_addr = (io_rw_32)&NMTC_PIO->txf[NMTC_PIO_SM_ID];
	dma_hw->ch[NMTC_DMA_A].transfer_count = NMTC_RUN0_CMDV_N;
	dma_hw->ch[NMTC_DMA_A].ctrl_trig = UTIL_DMA_CTRL_TRIG_EN_DSZ_INCR_INCW_RSZ_RSEL_TO_TREQ_IQ
		(1, DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD, 1, 0, 0, 0, NMTC_DMA_A, NMTC_PIO_DREQ_TX, 0);
}
