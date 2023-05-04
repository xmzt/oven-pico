#ifndef BASE_H
#define BASE_H

#include "pico/stdlib.h"

//#include "hardware/address_mapped.h"
#include "hardware/pio.h"
#include "hardware/sync.h"

#include "hardware/regs/intctrl.h"
//#include "hardware/regs/m0plus.h"

#include "hardware/structs/dma.h"
#include "hardware/structs/i2c.h"
#include "hardware/structs/nvic.h"
#include "hardware/structs/pll.h"
#include "hardware/structs/pwm.h"
#include "hardware/structs/rosc.h"
#include "hardware/structs/scb.h"
#include "hardware/structs/systick.h"
#include "hardware/structs/timer.h"
#include "hardware/structs/watchdog.h"

#include "nmtc.pio.h"
#include "ac.pio.h"

//=======================================================================================================================
// types
//=======================================================================================================================

typedef void thunk_t(void);
typedef void isr_t(void);

//=======================================================================================================================
// resource allocation and globals
//=======================================================================================================================

extern uint32_t g_pio0_ctrl; // to keep enable state for other state machines
extern uint32_t g_rando;

//-----------------------------------------------------------------------------------------------------------------------
// debug

#define NMTC_DEBUG 1
#define TMP101_DEBUG 1
#define AC_DEBUG 1

//-----------------------------------------------------------------------------------------------------------------------
// clocks
//
// copied from SDK clocks_init()

#define KHZ 1000
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
// pin and PWM allocation

// VBUS = USB 5V
// VSYS = VBUS - Schottky*1

#define NMTC_PIN_DB8_RS_RW_EN 0 // starting pin for 11 contiguous pins ordered DB0-DB7,RS,RW,EN
// disp.1.vss - pico.GND, logic gnd
// disp.2.vdd - pico.3V3, logic power (VDD-VSS: -0.3 - 7.0)
// disp.3.vee - pico.GND, lcd power neg, should be neg 1.2V, recommend vdd-vee of 4.8@-20C 4.5@25C 4.2@70C
// disp.4.rs - pico.gpio[8]
// disp.5.rw - pico.gpio[9]
// disp.6.en - pico.gpio[10]
// disp.[7:14] - pico.gpio[0-7]

#define TMP101_PIN_VCC 11 // tmp101.4.v+ yel
#define TMP101_PIN_SDA 12 // tmp101.6.sda blu
#define TMP101_PIN_SCL 13 // tmp101.1.scl yel
// tmp101.2.gnd - GND blu

#define AC_PIN_PHASE_IN 16
#define AC_PIN_PHASE_OUT 17
#define AC_PWM_SLICE_ID_PHASE 0
#define AC_PWM_SLICE_PHASE (&pwm_hw->slice[AC_PWM_SLICE_ID_PHASE])

#define AC_PIN_OUT4 18 // starting pin for 4 contiguous
#define AC_PWM_SLICE_ID_OUT01 0
#define AC_PWM_SLICE_OUT01 (&pwm_hw->slice[AC_PWM_SLICE_ID_OUT01])
#define AC_PWM_SLICE_ID_OUT23 0
#define AC_PWM_SLICE_OUT23 (&pwm_hw->slice[AC_PWM_SLICE_ID_OUT23])

//-----------------------------------------------------------------------------------------------------------------------
// dma allocation

#define TMP101_DMA_A 0
#define TMP101_DMA_B 1
#define NMTC_DMA_A   2
#define NMTC_DMA_B   3
#define AC_DMA_A     4
#define AC_DMA_B     5

//-----------------------------------------------------------------------------------------------------------------------
// pio allocation

#define MMAXX(a,b) ((a) >= (b) ? (a) : (b))

#define NMTC_PIO pio0_hw
#define NMTC_PIO_SM_ID 0
#define NMTC_PIO_SM (&NMTC_PIO->sm[NMTC_PIO_SM_ID])
#define NMTC_PIO_DREQ_RX DREQ_PIO0_RX0
#define NMTC_PIO_DREQ_TX DREQ_PIO0_TX0

#define AC_PHASE_PIO pio0_hw
#define AC_PHASE_PIO_SM_ID 1
#define AC_PHASE_PIO_SM (&AC_PIO->sm[AC_PHASE_PIO_SM_ID])
#define AC_PHASE_PIO_DREQ_RX DREQ_PIO0_RX1

#define AC_PHASE_PIO_IRQ PIO0_IRQ_1
#define AC_PHASE_PIO_INTE (pio0_hw->inte1)
#define AC_PHASE_PIO_INTS (pio0_hw->ints1)
#define AC_PHASE_PIO_INTR_BIT PIO_INTR_SM1_RXNEMPTY_LSB

#define NMTC_PIO_CODE_A 0
#define NMTC_PIO_CODE_E (NMTC_PIO_CODE_A + MMAXX(nmtc_pio_start_program.length, nmtc_pio_run_program.length))
#define AC_PHASE_PIO_CODE_A NMTC_PIO_CODE_E
#define AC_PHASE_PIO_CODE_E (AC_PHASE_PIO_CODE_A + ac_phase_pio_program.length)

inline static uint32_t g_pio0_ctrl_sme_set(uint32_t sme) {
	return g_pio0_ctrl |= sme << PIO_CTRL_SM_ENABLE_LSB;
}

inline static uint32_t g_pio0_ctrl_sme_clr(uint32_t sme) {
	return g_pio0_ctrl &= ~(sme << PIO_CTRL_SM_ENABLE_LSB);
}

//-----------------------------------------------------------------------------------------------------------------------
// i2c allocation

#define TMP101_I2C i2c0_hw
#define TMP101_I2C_IRQ I2C0_IRQ
#define TMP101_I2C_DREQ_TX DREQ_I2C0_TX
#define TMP101_I2C_DREQ_RX DREQ_I2C0_RX

//-----------------------------------------------------------------------------------------------------------------------
// alarms

#define TMP101_ALARM_ID 0
#define TMP101_ALARM_IRQ TIMER_IRQ_0

#define AC_ALARM_ID 1
#define AC_ALARM_IRQ TIMER_IRQ_1

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
