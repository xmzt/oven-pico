#include "tmp101.h"
#include "mainq.h"
#include "nmtc.h"
#include "util.h"

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/clocks.h"
#include "hardware/i2c.h"
#include "hardware/structs/dma.h"
#include <stdio.h>

// i2c: write bytes have RESTART=0 STOP=0, following read byte is RESTART=1 STOP=1
// i2c: also fine to STOP after writes to allow other bus activity
// t_lo and t_hi are signed int16_t, but treated as uint16_t for byte-by-byte transfer

//-----------------------------------------------------------------------------------------------------------------------
// globals

static uint32_t g_tick;
uint32_t g_tmp101_temp;
static uint32_t g_tx_abrt_source;
static uint32_t g_raw_intr_stat;

#define NMTC_CMDV_Z 40
static uint32_t g_nmtc_cmdV[NMTC_CMDV_Z];
static nmtc_q_item_t g_nmtc_q_item;

static void tx_abrt_mainq_fun() {
	printf("[tmp101 %u] TX_ABRT_SOURCE=%p RAW_INTR_STAT=%p\n", g_tick, g_tx_abrt_source, g_raw_intr_stat);
}

static void temp_mainq_fun() {
	float tempC = tmp101_tc_of_raw(g_tmp101_temp);
	float tempF = tmp101_tf_of_tc(tempC);
	printf("[tmp101 %u] temp %x C=%.4f F=%.4f\n", g_tick, g_tmp101_temp, tempC, tempF);

	uint8_t str[NMTC_CMDV_Z];
	sprintf(str, "%cC=%.4f%cF=%.4f", 0x80, tempC, 0x90, tempF);
	if(! g_nmtc_q_item.status) {
		uint32_t *cmdVE = nmtc_cmdV_from_u8(g_nmtc_cmdV, str);
		nmtc_q_item_post_nirq(&g_nmtc_q_item, (uint32_t)g_nmtc_cmdV, cmdVE - g_nmtc_cmdV);
	}
}

static uint32_t g_rx_buf[7];

//-----------------------------------------------------------------------------------------------------------------------
// constants

static const uint32_t tx_ctrl_trig = UTIL_DMA_CTRL_TRIG_EN_DSZ_INCR_INCW_RSZ_RSEL_TO_TREQ_IQ
	(1, DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD, 1, 0, 0, 0, TMP101_DMA_A, TMP101_I2C_DREQ_TX, 1);

static const uint32_t rx_ctrl_trig = UTIL_DMA_CTRL_TRIG_EN_DSZ_INCR_INCW_RSZ_RSEL_TO_TREQ_IQ
	(1, DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD, 0, 1, 0, 0, TMP101_DMA_A, TMP101_I2C_DREQ_RX, 1);

static const uint32_t sg_ctrl_trig = UTIL_DMA_CTRL_TRIG_EN_DSZ_INCR_INCW_RSZ_RSEL_TO_TREQ_IQ
	(1, DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD, 1, 1, 4, 1, TMP101_DMA_B, 0x3F, 1);

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

static const util_dma_block_0_t dma_chain[] = {
	// alternating between tx and tx prevents using aliases to reduce control block size
	{ (uint32_t)config_tx_buf, (uint32_t)&TMP101_I2C->data_cmd, 3, tx_ctrl_trig },
	{ (uint32_t)&TMP101_I2C->data_cmd, (uint32_t)(g_rx_buf + 6), 1, rx_ctrl_trig },
	{ (uint32_t)tlim_lo_tx_buf, (uint32_t)&TMP101_I2C->data_cmd, 5, tx_ctrl_trig },
	{ (uint32_t)&TMP101_I2C->data_cmd, (uint32_t)(g_rx_buf + 2), 2, rx_ctrl_trig },
	{ (uint32_t)tlim_hi_tx_buf, (uint32_t)&TMP101_I2C->data_cmd, 5, tx_ctrl_trig },
	{ (uint32_t)&TMP101_I2C->data_cmd, (uint32_t)(g_rx_buf + 4), 2, rx_ctrl_trig },
	{ (uint32_t)temp_tx_buf, (uint32_t)&TMP101_I2C->data_cmd, 3, tx_ctrl_trig },
	{ (uint32_t)&TMP101_I2C->data_cmd, (uint32_t)(g_rx_buf + 0), 2, rx_ctrl_trig },
	{ 0, 0, 0, 0 }
};

//-----------------------------------------------------------------------------------------------------------------------
// i/o

void tmp101_tx_abrt_irq(void) {
	g_tx_abrt_source = TMP101_I2C->tx_abrt_source;
	g_raw_intr_stat = TMP101_I2C->raw_intr_stat;
	TMP101_I2C->clr_tx_abrt;

	g_tick++;
	mainq_pro1(&g_mainq, tx_abrt_mainq_fun);
}

void tmp101_temp_req_alarm_irq(void) { 
	hw_clear_bits(&timer_hw->intr, 1<<TMP101_ALARM_ID);

	g_tick++;
	dma_hw->ch[TMP101_DMA_A].read_addr = (io_rw_32)(dma_chain + 6); // skip config, start at temp_tx_buf
	dma_hw->ch[TMP101_DMA_A].write_addr = (io_rw_32)&dma_hw->ch[TMP101_DMA_B];
	dma_hw->ch[TMP101_DMA_A].transfer_count = 4;
	dma_hw->ch[TMP101_DMA_A].ctrl_trig = sg_ctrl_trig;
}

void tmp101_dma_irq(void) {
	dma_hw->intr = 1<<TMP101_DMA_B;
	
	g_tick++;
	g_tmp101_temp = ((g_rx_buf[0] & 0xFF) << 8) | (g_rx_buf[1] & 0xFF);
	mainq_pro1(&g_mainq, temp_mainq_fun);

	util_irq_hand(TMP101_ALARM_IRQ_ID, tmp101_temp_req_alarm_irq);
	timer_hw->alarm[TMP101_ALARM_ID] += TMP101_TEMP_REQ_ALARM_DUR;
}

void tmp101_vcc_on_alarm_irq(void) { 
	hw_clear_bits(&timer_hw->intr, 1<<TMP101_ALARM_ID);

	g_tick++;
	dma_hw->ch[TMP101_DMA_A].read_addr = (io_rw_32)dma_chain;
	dma_hw->ch[TMP101_DMA_A].write_addr = (io_rw_32)&dma_hw->ch[TMP101_DMA_B];
	dma_hw->ch[TMP101_DMA_A].transfer_count = 4;
	dma_hw->ch[TMP101_DMA_A].ctrl_trig = sg_ctrl_trig;
}

void tmp101_vcc_off_alarm_irq(void) { 
	hw_clear_bits(&timer_hw->intr, 1<<TMP101_ALARM_ID);

	g_test[0] += 1;
	g_tick++;
	util_irq_hand(TMP101_ALARM_IRQ_ID, tmp101_vcc_on_alarm_irq);
	timer_hw->alarm[TMP101_ALARM_ID] += TMP101_VCC_ON_ALARM_DUR;
	sio_hw->gpio_set = 1<<TMP101_PIN_VCC;
}

//-----------------------------------------------------------------------------------------------------------------------
// init

void tmp101_init() {
	g_tick = 0;
	g_tmp101_temp = -1;
	g_tx_abrt_source = -1;
	g_raw_intr_stat = -1;
	g_nmtc_q_item.status = 0;

	i2c_hw_t *hw = TMP101_I2C;

	hw->enable = 0;
	hw->con = UTIL_I2C_CON_ME_SPD_TENAS_TENAM_RE_SLVD_SDIA_TXEC_RXFFHC
		(1, I2C_IC_CON_SPEED_VALUE_FAST, 0, 0, 1, 1, 0, 0, 0);
	hw->tar = TMP101_TAR;
	util_i2c_set_baudrate_lt_1e6(hw, TMP101_BAUD);
	
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
	g_test[0] = 0;
	hw_set_bits(&timer_hw->inte, 1<<TMP101_ALARM_ID);
	util_irq_hand_iser(TMP101_ALARM_IRQ_ID, tmp101_vcc_off_alarm_irq);
	timer_hw->alarm[TMP101_ALARM_ID] = timer_hw->timerawl + TMP101_VCC_OFF_ALARM_DUR;
}
