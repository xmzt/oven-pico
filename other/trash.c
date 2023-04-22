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
