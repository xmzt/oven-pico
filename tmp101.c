#include "tmp101.h"
#include "util.h"

#include "hardware/clocks.h"

// i2c: write bytes have RESTART=0 STOP=0, following read byte is RESTART=1 STOP=1
// i2c: also fine to STOP after writes to allow other bus activity
// t_lo and t_hi are signed int16_t, but treated as uint16_t for byte-by-byte transfer

//-----------------------------------------------------------------------------------------------------------------------
// globals

uint32_t g_tmp101_temp;

static uint32_t rx_buf[7];

//-----------------------------------------------------------------------------------------------------------------------
// constants

static const uint32_t config_tx_buf[] = {
	UTIL_I2C_DATA_WRITE_R_S_D(0, 0, TMP101_PTR_CONFIG),
	UTIL_I2C_DATA_WRITE_R_S_D(0, 0, TMP101_CONFIG),
	UTIL_I2C_DATA_READ_R_S(0, 1),
};

static const uint32_t tlim_lo_tx_buf[] = {
	UTIL_I2C_DATA_WRITE_R_S_D(0, 0, TMP101_PTR_TLIM_LO),
	UTIL_I2C_DATA_WRITE_R_S_D(0, 0, TMP101_TLIM_LO >> 8),
	UTIL_I2C_DATA_WRITE_R_S_D(0, 0, TMP101_TLIM_LO & 0xFF),
	UTIL_I2C_DATA_READ_R_S(0, 0),
	UTIL_I2C_DATA_READ_R_S(0, 1),
};

static const uint32_t tlim_hi_tx_buf[] = {
	UTIL_I2C_DATA_WRITE_R_S_D(0, 0, TMP101_PTR_TLIM_HI),
	UTIL_I2C_DATA_WRITE_R_S_D(0, 0, TMP101_TLIM_HI >> 8),
	UTIL_I2C_DATA_WRITE_R_S_D(0, 0, TMP101_TLIM_HI & 0xFF),
	UTIL_I2C_DATA_READ_R_S(0, 0),
	UTIL_I2C_DATA_READ_R_S(0, 1),
};

static const uint32_t temp_tx_buf[] = {
	UTIL_I2C_DATA_WRITE_R_S_D(0, 0, TMP101_PTR_TEMP),
	UTIL_I2C_DATA_READ_R_S(0, 0),
	UTIL_I2C_DATA_READ_R_S(0, 1),
};

static const uint32_t tx_ctrl_trig = UTIL_DMA_CTRL_TRIG_EN_DSZ_INCR_INCW_RSZ_RSEL_TO_TREQ_IQ
	(1, DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD, 1, 0, 0, 0, TMP101_DMA_A, TMP101_I2C_DREQ_TX, 1);

static const uint32_t rx_ctrl_trig = UTIL_DMA_CTRL_TRIG_EN_DSZ_INCR_INCW_RSZ_RSEL_TO_TREQ_IQ
	(1, DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD, 0, 1, 0, 0, TMP101_DMA_A, TMP101_I2C_DREQ_RX, 1);

static const uint32_t sg_ctrl_trig = UTIL_DMA_CTRL_TRIG_EN_DSZ_INCR_INCW_RSZ_RSEL_TO_TREQ_IQ
	(1, DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD, 1, 1, 4, 1, TMP101_DMA_B, 0x3F, 1);

static const util_dma_block_0_t dma_chain[] = {
	{ (uint32_t)config_tx_buf, (uint32_t)&TMP101_I2C->data_cmd, 3, tx_ctrl_trig },
	{ (uint32_t)&TMP101_I2C->data_cmd, (uint32_t)(rx_buf + 6), 1, rx_ctrl_trig },
	{ (uint32_t)tlim_lo_tx_buf, (uint32_t)&TMP101_I2C->data_cmd, 5, tx_ctrl_trig },
	{ (uint32_t)&TMP101_I2C->data_cmd, (uint32_t)(rx_buf + 2), 2, rx_ctrl_trig },
	{ (uint32_t)tlim_hi_tx_buf, (uint32_t)&TMP101_I2C->data_cmd, 5, tx_ctrl_trig },
	{ (uint32_t)&TMP101_I2C->data_cmd, (uint32_t)(rx_buf + 4), 2, rx_ctrl_trig },
	{ (uint32_t)temp_tx_buf, (uint32_t)&TMP101_I2C->data_cmd, 3, tx_ctrl_trig },
	{ (uint32_t)&TMP101_I2C->data_cmd, (uint32_t)(rx_buf + 0), 2, rx_ctrl_trig },
	{ 0, 0, 0, 0 }
};

//-----------------------------------------------------------------------------------------------------------------------
// i/o

void tmp101_tx_abrt_irq(void) {
	uint32_t raw = TMP101_I2C->raw_intr_stat;
	uint32_t source = TMP101_I2C->tx_abrt_source;
	TMP101_I2C->clr_tx_abrt;
	LOG_PUSH(log_tmp101_tx_abrt, source, raw);
}

void tmp101_temp_req_alarm_irq(void) { 
	hw_clear_bits(&timer_hw->intr, 1<<TMP101_ALARM_ID);

	LOG_PUSH(log_tmp101_vcc, 2);
	dma_hw->ch[TMP101_DMA_A].read_addr = (io_rw_32)(dma_chain + 6); // skip config, start at temp_tx_buf
	dma_hw->ch[TMP101_DMA_A].write_addr = (io_rw_32)&dma_hw->ch[TMP101_DMA_B];
	dma_hw->ch[TMP101_DMA_A].transfer_count = 4;
	dma_hw->ch[TMP101_DMA_A].ctrl_trig = sg_ctrl_trig;
}

void tmp101_vcc_on_alarm_irq(void) { 
	hw_clear_bits(&timer_hw->intr, 1<<TMP101_ALARM_ID);

	LOG_PUSH(log_tmp101_vcc, 1);
	// dma channel 0 (blockr)
	dma_hw->ch[TMP101_DMA_A].read_addr = (io_rw_32)dma_chain;
	dma_hw->ch[TMP101_DMA_A].write_addr = (io_rw_32)&dma_hw->ch[TMP101_DMA_B];
	dma_hw->ch[TMP101_DMA_A].transfer_count = 4;
	dma_hw->ch[TMP101_DMA_A].ctrl_trig = sg_ctrl_trig;
}

void tmp101_vcc_off_alarm_irq(void) { 
	hw_clear_bits(&timer_hw->intr, 1<<TMP101_ALARM_ID);

	LOG_PUSH(log_tmp101_vcc, 0);
	util_irq_hand(TMP101_ALARM_IRQ_ID, tmp101_vcc_on_alarm_irq);
	timer_hw->alarm[TMP101_ALARM_ID] += TMP101_VCC_ON_ALARM_DUR;
	sio_hw->gpio_set = 1<<TMP101_PIN_VCC;
}

void tmp101_dma_irq(void) {
	// clear interrupt and log
	uint32_t x = dma_hw->intr;
	uint32_t y = TMP101_DMA_INTS; 
	TMP101_DMA_INTS = 1<<TMP101_DMA_B;
	
	g_tmp101_temp = ((rx_buf[0] & 0xFF) << 8) | (rx_buf[1] & 0xFF);
	LOG_PUSH(log_tmp101_dma_irq, x, y, (uint32_t)rx_buf, g_tmp101_temp);

	util_irq_hand(TMP101_ALARM_IRQ_ID, tmp101_temp_req_alarm_irq);
	timer_hw->alarm[TMP101_ALARM_ID] += TMP101_TEMP_REQ_ALARM_DUR;
}

//-----------------------------------------------------------------------------------------------------------------------
// init_i2c

static void tmp101_set_baudrate_lt_1e6(i2c_hw_t *hw, uint baudrate) {
	uint freq_in = clock_get_hz(clk_sys);
		
    // TODO there are some subtleties to I2C timing which we are completely ignoring here
    uint period = (freq_in + baudrate / 2) / baudrate;
    uint lcnt = period * 3 / 5; // oof this one hurts
    uint hcnt = period - lcnt;

	// sda_tx_hold_count = freq_in [cycles/s] * 300ns * (1s / 1e9ns)
	// Reduce 300/1e9 to 3/1e7 to avoid numbers that don't fit in uint.
	// Add 1 to avoid division truncation.
	uint sda_tx_hold_count = ((freq_in * 3) / 10000000) + 1;

    hw->fs_scl_hcnt = hcnt;
    hw->fs_scl_lcnt = lcnt;
    hw->fs_spklen = lcnt < 16 ? 1 : lcnt / 16;
	hw->sda_hold = sda_tx_hold_count << I2C_IC_SDA_HOLD_IC_SDA_TX_HOLD_LSB
		| 0<<I2C_IC_SDA_HOLD_IC_SDA_RX_HOLD_LSB;
}

void tmp101_init_hw() {
	i2c_hw_t *hw = TMP101_I2C;

	hw->enable = 0;
	hw->con = UTIL_I2C_CON_ME_SPD_TENAS_TENAM_RE_SLVD_SDIA_TXEC_RXFFHC
		(1, I2C_IC_CON_SPEED_VALUE_FAST, 0, 0, 1, 1, 0, 0, 0);
	hw->tar = TMP101_TAR;
	tmp101_set_baudrate_lt_1e6(hw, TMP101_BAUD);
	
    hw->tx_tl = 1-1;
	hw->rx_tl = 1-1;
    hw->dma_cr = 1<<I2C_IC_DMA_CR_TDMAE_LSB | 1<<I2C_IC_DMA_CR_RDMAE_LSB;

	TMP101_I2C->intr_mask = 1<<I2C_IC_INTR_MASK_M_TX_ABRT_LSB;
	util_irq_hand_iser(TMP101_I2C_IRQ, tmp101_tx_abrt_irq);

	hw->enable = 1<<I2C_IC_ENABLE_ENABLE_LSB;

	TMP101_DMA_INTE = 1<<TMP101_DMA_B;
	util_irq_hand_iser(TMP101_DMA_IRQ, tmp101_dma_irq);
}

void tmp101_start() {
	hw_set_bits(&timer_hw->inte, 1<<TMP101_ALARM_ID);
	util_irq_hand_iser(TMP101_ALARM_IRQ_ID, tmp101_vcc_off_alarm_irq);
	timer_hw->alarm[TMP101_ALARM_ID] = timer_hw->timerawl + TMP101_VCC_OFF_ALARM_DUR;
}
