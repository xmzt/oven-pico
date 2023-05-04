//-----------------------------------------------------------------------------------------------------------------------
// ac_pwm_val

static uint32_t ac_pwm_val_inisr(uint32_t hf0) {
	// get hf1 with consistent intr flag
	uint32_t intr = pwm_hw->intr;
	uint32_t hf1 = AC_SLICE_PWMHF->ctr;
	uint32_t lf;
	if(hf1 < hf0) {
		// counter has wrapped. intr may be before or after wrap.
		// get intr again. if set, pwmlf has not been updated, use pre-wrap unmodified pwmlf.
		// if not set, interrupt has been handled. subtract one from pwmlf to get pre-wrap value.
		intr = pwm_hw->intr;
		lf = g_ac_pwmlf - !((1 << AC_SLICE_ID_PWMHF) & intr);
	} else {
		// same hf0 and hf1 on same side of intr
		lf = g_ac_pwmlf + !!((1 << AC_SLICE_ID_PWMHF) & intr);
	}
	return hf0 | (lf << 16);
}
	
void ac_pwm_init(void) {
	//ac_pio_pwmlf_init_start(AC_PIO, AC_PIO_PWMLF_SM_ID);

	g_ac_pwmlf = 0;
	
	AC_SLICE_PWMHF->div = UTIL_PWM_DIV_FRAC_INT(0, 1);
	AC_SLICE_PWMHF->top = 0xFFFF;
	AC_SLICE_PWMHF->cc = UTIL_PWM_CC_A_B(0xFFFE, 0); // low for 2 cycles, for a reason (todo)
	AC_SLICE_PWMHF->csr = UTIL_PWM_CSR_EN_PHCORRECT_AINV_BINV_DIVMODE_PHRET_PHADV
		(0, 0, 0, 0, 0, 0, 0);

	pwm_hw->en = 1 << AC_SLICE_ID_PWMHF;

	//-------------------------------------------------------------------------------------------------------------------
	// pads prepare

	//padsbank0_hw->io[AC_PIN_PWMHF_OUT] = UTIL_PADS_BANK0_GPIO_OD_IE_DRIVE_PUE_PDE_SCHMITT_SLEWFAST
	//	(0, 1, PADS_BANK0_GPIO0_DRIVE_VALUE_4MA, 0, 0, 1, 0);

	//-------------------------------------------------------------------------------------------------------------------
	// pads funcsel

	//iobank0_hw->io[AC_PIN_PWMHF_OUT].ctrl = UTIL_IO_BANK0_GPIO_CTRL_FUNCSEL
	//	(IO_BANK0_GPIO0_CTRL_FUNCSEL_VALUE_PWM_A_0);
}

void todoac_pio_pwmlf_init_start(pio_hw_t *hw, uint sm_id) {
	pio_sm_hw_t *sm = &hw->sm[sm_id];

	util_pio_load(hw->instr_mem, AC_PIO_PWMLF_CODE_A, ac_pio_pwmlf_program.instructions, ac_pio_pwmlf_program.length);
	
	sm->clkdiv = UTIL_PIO_SM_CLKDIV_FRAC_INT(0, 1);
	sm->execctrl = UTIL_PIO_SM_EXECCTRL_STN_STS_WB_WT_OS_IOE_OES_JP_SPD_SEN
		(0, 0, AC_PIO_PWMLF_CODE_A+ac_pio_pwmlf_wrap_target, AC_PIO_PWMLF_CODE_A+ac_pio_pwmlf_wrap, 0, 0, 0, 0, 0, 0);
	sm->shiftctrl = UTIL_PIO_SM_SHIFTCTRL_APUSH_APULL_ISD_OSD_PUSHT_PULLT_FJT_FJR
		(1, 0, /*l0r1*/ 0, /*l0r1*/ 1, /*0->32*/ 0, /*0->32*/ 0, 0, 0);
	sm->pinctrl = UTIL_PIO_SM_PINCTRL_OB_SB_SSB_IB_OC_SC_SSC
		(0, 0, 0, AC_PIN_PWMHF_OUT, 0, 0, 0);
	
	// prepare state machine, start.
	sm->instr = pio_encode_jmp(AC_PIO_PWMLF_CODE_A);
	hw->ctrl = g_pio0_ctrl_sme_set(1 << sm_id);
}

void ac_pwm_init(void) {
	pio_hw_t *hw = AC_PIO;
	pio_sm_hw_t *sm = AC_PWM_PIO_SM;
	uint sm_id = AC_PWM_PIO_SM_ID;

	//-------------------------------------------------------------------------------------------------------------------
	// pio

	// load code
	util_pio_load(hw->instr_mem, AC_PWM_PIO_CODE_A, ac_pwm_pio_program.instructions, ac_pwm_pio_program.length);
	
	// configure state machine registers
	sm->clkdiv = UTIL_PIO_SM_CLKDIV_FRAC_INT(0, 1);
	sm->execctrl = UTIL_PIO_SM_EXECCTRL_STN_STS_WB_WT_OS_IOE_OES_JP_SPD_SEN
		(0, 0, AC_PWM_PIO_CODE_A+ac_pwm_pio_wrap_target, AC_PWM_PIO_CODE_A+ac_pwm_pio_wrap, 0, 0, 0, 0, 0, 0);
	sm->shiftctrl = UTIL_PIO_SM_SHIFTCTRL_APUSH_APULL_ISD_OSD_PUSHT_PULLT_FJT_FJR
		(0, 0, /*l0r1*/1, /*l0r1*/1, /*0->32*/0, /*0->32*/0, 0, 0);
	sm->pinctrl = UTIL_PIO_SM_PINCTRL_OB_SB_SSB_IB_OC_SC_SSC
		(AC_PWM_PIN_LF_IN, 0, 0, AC_PWM_PIN_HF_OUT, 1, 0, 0);
	
	// prepare the state machine, start now.
	hw->txf[sm_id] = 1;
	sm->instr = pio_encode_pull(/*if_empty*/ false, /*block*/ false);
	sm->instr = pio_encode_out(pio_pindirs, 1);
	sm->instr = pio_encode_jmp(AC_PWM_PIO_CODE_A);
	hw->ctrl = g_pio0_ctrl_sme_set(1 << sm_id);

	//-------------------------------------------------------------------------------------------------------------------
	// pwm

	pwm_hw->slice[AC_PWM_SLICE_LF].div = UTIL_PWM_DIV_FRAC_INT(0, 1);
	pwm_hw->slice[AC_PWM_SLICE_LF].csr = UTIL_PWM_CSR_EN_PHCORRECT_AINV_BINV_DIVMODE_PHRET_PHADV
		(0, 0, 0, 0, PWM_CH0_CSR_DIVMODE_VALUE_FALL, 0, 0);

	pwm_hw->slice[AC_PWM_SLICE_HF].div = UTIL_PWM_DIV_FRAC_INT(0, 1);
	pwm_hw->slice[AC_PWM_SLICE_HF].top = 0xFFFF;
	pwm_hw->slice[AC_PWM_SLICE_HF].cc = UTIL_PWM_CC_A_B(0xF000, 0);
	pwm_hw->slice[AC_PWM_SLICE_HF].csr = UTIL_PWM_CSR_EN_PHCORRECT_AINV_BINV_DIVMODE_PHRET_PHADV
		(0, 0, 0, 0, 0, 0, 0);

	pwm_hw->en = 1 << AC_PWM_SLICE_HF | 1 << AC_PWM_SLICE_LF;

	//-------------------------------------------------------------------------------------------------------------------
	// pads prepare

	padsbank0_hw->io[AC_PWM_PIN_LF_IN] = UTIL_PADS_BANK0_GPIO_OD_IE_DRIVE_PUE_PDE_SCHMITT_SLEWFAST
		(0, 1, PADS_BANK0_GPIO0_DRIVE_VALUE_4MA, 0, 0, 1, 0);
	padsbank0_hw->io[AC_PWM_PIN_HF_OUT] = UTIL_PADS_BANK0_GPIO_OD_IE_DRIVE_PUE_PDE_SCHMITT_SLEWFAST
		(0, 1, PADS_BANK0_GPIO0_DRIVE_VALUE_4MA, 0, 0, 1, 0);

	//-------------------------------------------------------------------------------------------------------------------
	// pads funcsel

	iobank0_hw->io[AC_PWM_PIN_LF_IN].ctrl = UTIL_IO_BANK0_GPIO_CTRL_FUNCSEL
		(IO_BANK0_GPIO0_CTRL_FUNCSEL_VALUE_PIO0_0);
	iobank0_hw->io[AC_PWM_PIN_HF_OUT].ctrl = UTIL_IO_BANK0_GPIO_CTRL_FUNCSEL
		(IO_BANK0_GPIO0_CTRL_FUNCSEL_VALUE_PWM_A_0);
}


//-----------------------------------------------------------------------------------------------------------------------
// constants

static const uint32_t pio_dreq_ctrl_trig = UTIL_DMA_CTRL_TRIG_EN_DSZ_INCR_INCW_RSZ_RSEL_TO_TREQ_IQ
	(1, DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD, 0, 0, 0, 0, AC_DMA_A, AC_PIO_DREQ_RX, 1);

static const uint32_t now_ctrl_trig = UTIL_DMA_CTRL_TRIG_EN_DSZ_INCR_INCW_RSZ_RSEL_TO_TREQ_IQ
	(1, DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD, 0, 0, 0, 0, AC_DMA_A, 0x3F, 1);

static const uint32_t sg_ctrl_trig = UTIL_DMA_CTRL_TRIG_EN_DSZ_INCR_INCW_RSZ_RSEL_TO_TREQ_IQ
	(1, DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD, 1, 1, 4, 1, AC_DMA_B, 0x3F, 1);

static const util_dma_block_0_t dma_chain[] = {
	{ (io_rw_32)&timer_hw->timerawl, (io_rw_32)&g_phase_in_ts1, 1, pio_dreq_ctrl_trig },
	{ (io_rw_32)&, (io_rw_32)&g_phase_in_state, 1, now_ctrl_trig },
	{ 0, 0, 0, 0 }
};

{
	//dma_hw->ch[AC_DMA_A].al3_read_addr_trig = (io_rw_32)dma_chain;

	dma_hw->ch[AC_DMA_A].read_addr = (io_rw_32)&timer_hw->timerawl;
	dma_hw->ch[AC_DMA_A].write_addr = (io_rw_32)&dma_hw->ch[AC_DMA_B];
	dma_hw->ch[AC_DMA_A].transfer_count = 4;
	dma_hw->ch[AC_DMA_A].ctrl_trig = sg_ctrl_trig;
	(1, DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD, 0, 0, 0, 0, AC_DMA_A, AC_PIO_DREQ_RX, 1);
	{ (io_rw_32), (io_rw_32)&g_phase_in_ts1, 1, pio_dreq_ctrl_trig }, }
		}


//-----------------------------------------------------------------------------------------------------------------------
// init_config

void tmp101_req_wp0r2(i2c_hw_t *hw, uint ptr) {
	printf("[tmp101] req_wp0r2 %02x\n", ptr);
	hw->data_cmd = UTIL_I2C_DW(0, 0, ptr);
	hw->data_cmd = UTIL_I2C_DR(1, 0);
	hw->data_cmd = UTIL_I2C_DR(0, 1);
	hw->rx_tl = 2-1;
	hw->intr_mask = 1<<I2C_IC_INTR_MASK_M_TX_ABRT_LSB
		| 1<<I2C_IC_INTR_MASK_M_RX_FULL_LSB;
}

void tmp101_req_wp1r1(i2c_hw_t *hw, uint ptr, uint val) {
	printf("[tmp101] req_wp1r1 %02x,%02x\n", ptr, val);
	hw->data_cmd = UTIL_I2C_DW(0, 0, ptr);
	hw->data_cmd = UTIL_I2C_DW(0, 0, val);
	hw->data_cmd = UTIL_I2C_DR(1, 1);
	hw->rx_tl = 1-1;
	hw->intr_mask = 1<<I2C_IC_INTR_MASK_M_TX_ABRT_LSB
		| 1<<I2C_IC_INTR_MASK_M_RX_FULL_LSB;
}

void tmp101_req_wp2r2(i2c_hw_t *hw, uint ptr, uint val) {
	printf("[tmp101] req_wp2r2 %02x,%02x,%02x\n", ptr, val >> 8, val & 0xFF);
	hw->data_cmd = UTIL_I2C_DW(0, 0, ptr);
	hw->data_cmd = UTIL_I2C_DW(0, 0, val >> 8);
	hw->data_cmd = UTIL_I2C_DW(0, 0, val & 0xFF);
	hw->data_cmd = UTIL_I2C_DR(1, 0);
	hw->data_cmd = UTIL_I2C_DR(0, 1);
	hw->rx_tl = 2-1;
	hw->intr_mask = 1<<I2C_IC_INTR_MASK_M_TX_ABRT_LSB
		| 1<<I2C_IC_INTR_MASK_M_RX_FULL_LSB;
}

void tmp101_rsp_r1(i2c_hw_t *hw, tmp101_state_t *state, uint *dst) {
	uint32_t intr = hw->intr_stat;
	if(I2C_IC_INTR_STAT_R_TX_ABRT_BITS & intr) {
		tmp101_rsp_tx_abrt(hw, state);
	}
	else if(I2C_IC_INTR_STAT_R_RX_FULL_BITS & intr) {
		*dst = hw->data_cmd & 0xFF;
		printf("[tmp101] rsp1 0x%x\n", *dst);
	}
}

void tmp101_rsp_r2(i2c_hw_t *hw, tmp101_state_t *state, uint *dst) {
	uint32_t intr = hw->intr_stat;
	if(I2C_IC_INTR_STAT_R_TX_ABRT_BITS & intr) {
		tmp101_rsp_tx_abrt(hw, state);
	}
	else if(I2C_IC_INTR_STAT_R_RX_FULL_BITS & intr) {
		uint x = hw->data_cmd & 0xFF;
		x <<= 8;
		x |= hw->data_cmd & 0xFF;
		*dst = x;
		printf("[tmp101] rsp2 0x%x\n", x);
	}
}

void tmp101_temp_rsp_irq(void) {
	tmp101_rsp_r2(TMP101_I2C, &tmp101_state, &tmp101_state.temp);
	timer_hw->alarm[TMP101_ALARM_ID] += TMP101_TEMP_INTERVAL;
}

void tmp101_temp_req_irq(void) {
	hw_clear_bits(&timer_hw->intr, 1<<TMP101_ALARM_ID); // redundant when called from init config
	tmp101_req_wp0r2(TMP101_I2C, TMP101_PTR_TEMP);
	util_irq_hand(TMP101_I2C_IRQ, tmp101_temp_rsp_irq);
	util_irq_iser(1<<TMP101_I2C_IRQ);
}

void tmp101_tlim_hi_rsp_irq(void) {
	tmp101_rsp_r2(TMP101_I2C, &tmp101_state, &tmp101_state.tlim_hi);
	tmp101_temp_req_irq();
}

void tmp101_tlim_lo_rsp_irq(void) {
	uint32_t buf[3];
	tmp101_rsp_r2(TMP101_I2C, &tmp101_state, &tmp101_state.tlim_lo);
	tmp101_req_wp2r2(TMP101_I2C, TMP101_PTR_TLIM_HI, TMP101_TLIM_HI);
	util_irq_hand(TMP101_I2C_IRQ, tmp101_tlim_hi_rsp_irq);
	util_irq_iser(1<<TMP101_I2C_IRQ);
}

void tmp101_config_rsp_irq_ndma(void) {
	uint32_t buf[2];
	tmp101_rsp_r1(TMP101_I2C, &tmp101_state, &tmp101_state.config);
	tmp101_req_wp2r2(TMP101_I2C, TMP101_PTR_TLIM_LO, TMP101_TLIM_LO);
	util_irq_hand(TMP101_I2C_IRQ, tmp101_tlim_lo_rsp_irq);
	util_irq_iser(1<<TMP101_I2C_IRQ);
}

void tmp101_config_rsp_irq(void) {
	tmp101_rsp_tx_abrt(TMP101_I2C, &tmp101_state);
}

	
//tmp101_req_wp1r1(TMP101_I2C, TMP101_PTR_CONFIG, TMP101_CONFIG);
//tmp101_rsp_r1(TMP101_I2C, &tmp101_state, &tmp101_state.config);

void tmp101_rsp_tx_abrt(i2c_hw_t *hw) {
	uint a,b,c;

	g_tmp101_tx_abrt_source = hw->tx_abrt_source;
	
	a = hw->intr_stat;
	b = hw->intr_mask;
	c = hw->raw_intr_stat;
	hw->clr_tx_abrt;
	LOG_PUSH(log_tmp101_rsp_tx_abrt, state->tx_abrt_source, a, b, c);
}

inline static void log_push_dma_01() {
	LOG_PUSH(log_dma, 0, 
			 dma_hw->ch[0].read_addr,
			 dma_hw->ch[0].write_addr,
			 dma_hw->ch[0].transfer_count,
			 dma_hw->ch[0].ctrl_trig);
	LOG_PUSH(log_dma, 1, 
			 dma_hw->ch[1].read_addr,
			 dma_hw->ch[1].write_addr,
			 dma_hw->ch[1].transfer_count,
			 dma_hw->ch[1].ctrl_trig);
}

//-----------------------------------------------------------------------------------------------------------------------
// nmtc_* i/o

#if NMTC_DEBUG
inline static uint32_t nmtc_get(PIO pio, uint sm) {
	uint32_t y = pio_sm_get_blocking(pio, sm);
	printf("nmtc_get 0x%x\n", y);
	return y;
}

inline static void nmtc_put(PIO pio, uint sm, uint32_t x) {
	printf("nmtc_put 0x%03x\n", x);
	pio_sm_put_blocking(pio, sm, x);
}

#else
#define nmtc_get pio_sm_get_blocking
#define nmtc_put pio_sm_put_blocking
#endif // #if NMTC_DEBUG

void nmtc_test(PIO pio, uint sm) {
	char ch = 0;
	char chE = 0x10;
	char chE2 = 0x20;

	nmtc_put(pio, sm, NMTC_DDRAM_ADR_INST | NMTC_DDRAM_ADR_LINE1);
	for(;;) {
		nmtc_put(pio, sm, NMTC_DATA_WRITE_INST | ch);
		if(++ch == chE) {
			if(ch == chE2) goto phase1_from_phase0;
			nmtc_put(pio, sm, NMTC_DDRAM_ADR_INST | NMTC_DDRAM_ADR_LINE2);
			chE += 0x10;
		}
	}

	for(;;) {
		sleep_ms(300);
		nmtc_put(pio, sm, NMTC_DATA_WRITE_INST | ch);
		if(++ch == chE) {
			if(ch == chE2) {
			phase1_from_phase0:
				nmtc_put(pio, sm, NMTC_DDRAM_ADR_INST | NMTC_DDRAM_ADR_LINE1);
				chE += 0x10;
				chE2 = chE + 0x10;
			} else {
				nmtc_put(pio, sm, NMTC_DDRAM_ADR_INST | NMTC_DDRAM_ADR_LINE2);
				chE += 0x10;
			}
		}
	}
}
#endif

void log_nmtc_pio_irq(uint32_t *v) {
	printf("[nmtc] pio_irq irq=%p intr=%p ints=%p\n", v[1], v[2], v[3]);
	uint32_t x, a=0, b=0, c=0, d=0, e=0;

	for(uint i = PIN_NMTC_DB8_RS_RW_EN, j = i + 11; i < j; i++) {
		x = iobank0_hw->io[i].status;
		a |= ((x >> 8) & 1) << i;
		b |= ((x >> 9) & 1) << i;
		c |= ((x >> 13) & 1) << i;
		d |= ((x >> 17) & 1) << i;
		e |= ((x >> 19) & 1) << i;
	}
	printf("    o_peri=%p o_pad=%p oe_pad=%p i_pad=%p i_peri=%p\n", a, b, c, d, e);
}

void nmtc_dump_cmdV() {
	printf("[nmtc_start_cmdV]\n");
	for(const uint32_t *i = nmtc_start_cmdV, *j = i + NMTC_START_CMDV_N; i < j; i += 2)
		printf("    %10u %03x %03x %x\n", i[0], i[1] & 0x3FF, (i[1] >> 10) & 0x3FF, i[1] >> 20);
}
