#ifndef BASE_H
#define BASE_H

#include "util.h"
#include "nmtc.pio.h"

//=======================================================================================================================
// constant configuration
//=======================================================================================================================

//-----------------------------------------------------------------------------------------------------------------------
// debug

#define NMTC_DEBUG 1
#define TMP101_DEBUG 1

//-----------------------------------------------------------------------------------------------------------------------
// pin allocation

// VBUS = USB 5V
// VSYS = VBUS - Schottky*1

#define PIN_NMTC_DB8_RS_RW_EN 0 // starting pin for 11 contiguous pins ordered DB0-DB7,RS,RW,EN
// disp.1.vss - pico.GND, logic gnd
// disp.2.vdd - pico.3V3, logic power (VDD-VSS: -0.3 - 7.0)
// disp.3.vee - pico.GND, lcd power neg, should be neg 1.2V, recommend vdd-vee of 4.8@-20C 4.5@25C 4.2@70C
// disp.4.rs - pico.gpio[8]
// disp.5.rw - pico.gpio[9]
// disp.6.en - pico.gpio[10]
// disp.[7:14] - pico.gpio[0-7]

#define PIN_TMP101_VCC 11 // tmp101.4.v+ yel
#define PIN_TMP101_SDA 12 // tmp101.6.sda blu
#define PIN_TMP101_SCL 13 // tmp101.1.scl yel
// tmp101.2.gnd - GND blu

//-----------------------------------------------------------------------------------------------------------------------
// dma allocation

#define DMA_TMP101_A 0
#define DMA_TMP101_B 1
#define DMA_NMTC_A   2

#define DMA_IRQ_TMP101 DMA_IRQ_0
#define DMA_INTE_TMP101 (dma_hw->inte0)
#define DMA_INTS_TMP101 (dma_hw->ints0)

#define DMA_IRQ_NMTC DMA_IRQ_1
#define DMA_INTE_NMTC (dma_hw->inte1)
#define DMA_INTS_NMTC (dma_hw->ints1)

//-----------------------------------------------------------------------------------------------------------------------
// pio allocation

#define MMAXX(a,b) ((a) >= (b) ? (a) : (b))

#define PIO_NMTC pio0_hw

#define PIO0_SM_ID_NMTC 0

#define PIO0_CODE_NMTC_A 0
#define PIO0_CODE_NMTC_E (PIO_CODE_NMTC_A + MMAXX(nmtc_pio_start_program.length, nmtc_pio_run_program.length))

#define PIO0_DREQ_NMTC_RX DREQ_PIO0_RX0
#define PIO0_DREQ_NMTC_TX DREQ_PIO0_TX0

#define PIO0_IRQ_NMTC PIO0_IRQ_0
#define PIO0_INTE_NMTC (pio0_hw->inte0)
#define PIO0_INTS_NMTC (pio0_hw->ints0)

#define PIO0_PIOIRQ_NMTC 0

extern uint32_t g_pio0_ctrl; // to keep enable state for other state machines

inline static uint32_t g_pio0_ctrl_sme_set(uint32_t sme) {
	return g_pio0_ctrl |= sme << PIO_CTRL_SM_ENABLE_LSB;
}

inline static uint32_t g_pio0_ctrl_sme_clr(uint32_t sme) {
	return g_pio0_ctrl &= ~(sme << PIO_CTRL_SM_ENABLE_LSB);
}

//-----------------------------------------------------------------------------------------------------------------------
// i2c allocation

#define I2C_TMP101 i2c0_hw

#define I2C_IRQ_TMP101 I2C0_IRQ

#define I2C_DREQ_TMP101_TX DREQ_I2C0_TX
#define I2C_DREQ_TMP101_RX DREQ_I2C0_RX

//-----------------------------------------------------------------------------------------------------------------------
// alarms

#define ALARM_ID_TMP101 0
#define ALARM_ID_NMTC 1

#define ALARM_IRQ_ID_TMP101 TIMER_IRQ_0
#define ALARM_IRQ_ID_NMTC TIMER_IRQ_1

//-----------------------------------------------------------------------------------------------------------------------
// power supply ? todo is this implemented
//
// Rvdd,vee = 8.5kohm min. 5V / 8500 = 0.588mA
// target 1mA @ -5V, 100 khz freq

#endif
