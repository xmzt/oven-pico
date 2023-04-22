#ifndef BASE_H
#define BASE_H

#include "util.h"
#include "nmtc.pio.h"

//=======================================================================================================================
// resource allocation and globals
//=======================================================================================================================

extern uint32_t g_pio0_ctrl; // to keep enable state for other state machines
extern uint32_t g_rando;
extern uint32_t g_test[];

//-----------------------------------------------------------------------------------------------------------------------
// debug

#define NMTC_DEBUG 1
#define TMP101_DEBUG 1

//-----------------------------------------------------------------------------------------------------------------------
// clocks
//
// copied from SDK clocks_init()

#define MHZ 1000000

// pll_init(pll_sys, 1, 1500 * MHZ, 6, 2);
// pll_init(pll_usb, 1, 1200 * MHZ, 5, 5);

#define CLK_REF_HZ (12 * MHZ)
#define CLK_SYS_HZ (125 * MHZ)
#define CLK_USB_HZ (48 * MHZ)
#define CLK_ADC_HZ (48 * MHZ)
#define CLK_RTC_HZ (46875)
#define CLK_PERI_HZ (125 * MHZ)

#define TIMER_HZ (1 * MHZ)

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

#define DMA_TMP101_A 8
#define DMA_TMP101_B 9
#define DMA_NMTC_A   10
#define DMA_NMTC_B   11

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

#define ALARM_IRQ_ID_TMP101 TIMER_IRQ_0

//-----------------------------------------------------------------------------------------------------------------------
// power supply ? todo is this implemented
//
// Rvdd,vee = 8.5kohm min. 5V / 8500 = 0.588mA
// target 1mA @ -5V, 100 khz freq

//-----------------------------------------------------------------------------------------------------------------------
// power saving: todo
//
// NVIC_ISER in use:
// 0=TIMER_IRQ_0
// 5=USBCTRL_IRQ_0 SWD debug
// 7=PIO0_IRQ_0
// 11=DMA_IRQ_0
// 12=DMA_IRQ_1
// 23=I2C0_IRQ

//NVIC_ISER=808008A9 SLEEP_EN0=FFFFFFFF SLEEP_EN1=00007FFF

#define OVEN_SLEEP_EN0								\
	(CLOCKS_SLEEP_EN0_CLK_SYS_SRAM3					\
	 | CLOCKS_SLEEP_EN0_CLK_SYS_SRAM2				\
	 | CLOCKS_SLEEP_EN0_CLK_SYS_SRAM1				\
	 | CLOCKS_SLEEP_EN0_CLK_SYS_SRAM0				\
	 | CLOCKS_SLEEP_EN0_CLK_SYS_SPI1				\
	 | CLOCKS_SLEEP_EN0_CLK_PERI_SPI1				\
	 | CLOCKS_SLEEP_EN0_CLK_SYS_SPI0				\
	 | CLOCKS_SLEEP_EN0_CLK_PERI_SPI0				\
	 | CLOCKS_SLEEP_EN0_CLK_SYS_SIO					\
	 | CLOCKS_SLEEP_EN0_CLK_SYS_RTC					\
	 | CLOCKS_SLEEP_EN0_CLK_RTC_RTC					\
	 | CLOCKS_SLEEP_EN0_CLK_SYS_ROSC				\
	 | CLOCKS_SLEEP_EN0_CLK_SYS_ROM					\
	 | CLOCKS_SLEEP_EN0_CLK_SYS_RESETS				\
	 | CLOCKS_SLEEP_EN0_CLK_SYS_PWM					\
	 | CLOCKS_SLEEP_EN0_CLK_SYS_PSM					\
	 | CLOCKS_SLEEP_EN0_CLK_SYS_PLL_USB				\
	 | CLOCKS_SLEEP_EN0_CLK_SYS_PLL_SYS				\
	 | CLOCKS_SLEEP_EN0_CLK_SYS_PIO1				\
	 | CLOCKS_SLEEP_EN0_CLK_SYS_PIO0				\
	 | CLOCKS_SLEEP_EN0_CLK_SYS_PADS				\
	 | CLOCKS_SLEEP_EN0_CLK_SYS_VREG_AND_CHIP_RESET	\
	 | CLOCKS_SLEEP_EN0_CLK_SYS_JTAG				\
	 | CLOCKS_SLEEP_EN0_CLK_SYS_IO					\
	 | CLOCKS_SLEEP_EN0_CLK_SYS_I2C1				\
	 | CLOCKS_SLEEP_EN0_CLK_SYS_I2C0				\
	 | CLOCKS_SLEEP_EN0_CLK_SYS_DMA					\
	 | CLOCKS_SLEEP_EN0_CLK_SYS_BUSFABRIC			\
	 | CLOCKS_SLEEP_EN0_CLK_SYS_BUSCTRL				\
	 | CLOCKS_SLEEP_EN0_CLK_SYS_ADC					\
	 | CLOCKS_SLEEP_EN0_CLK_ADC_ADC					\
	 | CLOCKS_SLEEP_EN0_CLK_SYS_CLOCKS				\
	 )

#define OVEN_SLEEP_EN1									\
	(0 | /*0*/ CLOCKS_SLEEP_EN1_CLK_SYS_XOSC			\
	 | CLOCKS_SLEEP_EN1_CLK_SYS_XIP						\
	 | CLOCKS_SLEEP_EN1_CLK_SYS_WATCHDOG		\
	 | CLOCKS_SLEEP_EN1_CLK_USB_USBCTRL			\
	 | /*4*/ CLOCKS_SLEEP_EN1_CLK_SYS_USBCTRL	\
	 | CLOCKS_SLEEP_EN1_CLK_SYS_UART1			\
	 | CLOCKS_SLEEP_EN1_CLK_PERI_UART1			\
	 | CLOCKS_SLEEP_EN1_CLK_SYS_UART0			\
	 | /*8*/ CLOCKS_SLEEP_EN1_CLK_PERI_UART0		\
	 | CLOCKS_SLEEP_EN1_CLK_SYS_TIMER			\
	 | CLOCKS_SLEEP_EN1_CLK_SYS_TBMAN			\
	 | CLOCKS_SLEEP_EN1_CLK_SYS_SYSINFO			\
	 | /*12*/CLOCKS_SLEEP_EN1_CLK_SYS_SYSCFG	\
	 | CLOCKS_SLEEP_EN1_CLK_SYS_SRAM5			\
	 | CLOCKS_SLEEP_EN1_CLK_SYS_SRAM4			\
	 )

#define OVEN_WFIN_DUR 5000000


#endif
