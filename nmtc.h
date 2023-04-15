#ifndef NMTC_H
#define NMTC_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "nmtc.pio.h"

// Raspberry Pi Pico, PNMTC-S16205DRGHS display with SPLC780 controller

//-----------------------------------------------------------------------------------------------------------------------
// SPLC780 register descriptions

#define NMTC_RS_INST                (0<<8)
#define NMTC_RS_DATA                (1<<8)

#define NMTC_RW_WRITE               (0<<9)
#define NMTC_RW_READ                (1<<9)

#define NMTC_CLEAR_INST             (1<<0)

#define NMTC_RETURN_HOME_INST       (1<<1)

#define NMTC_ENTRY_MODE_INST        (1<<2)
#define NMTC_ENTRY_MODE_DEC         (0<<1)
#define NMTC_ENTRY_MODE_INC         (1<<1)
#define NMTC_ENTRY_MODE_NSHIFT      (0<<0)
#define NMTC_ENTRY_MODE_SHIFT       (1<<0)

#define NMTC_DISPLAY_INST           (1<<3)
#define NMTC_DISPLAY_OFF            (0<<2)
#define NMTC_DISPLAY_ON             (1<<2)
#define NMTC_DISPLAY_CURSOR_OFF     (0<<1)
#define NMTC_DISPLAY_CURSOR_ON      (1<<1)
#define NMTC_DISPLAY_BLINK_OFF      (0<<0)
#define NMTC_DISPLAY_BLINK_ON       (1<<0)

#define NMTC_SHIFT_INST             (1<<4)
#define NMTC_SHIFT_CURSOR           (0<<3)
#define NMTC_SHIFT_DISPLAY          (1<<3)
#define NMTC_SHIFT_LEFT             (0<<2)
#define NMTC_SHIFT_RIGHT            (1<<2)

#define NMTC_FUNC_INST              (1<<5)
#define NMTC_FUNC_BITS_4            (0<<4)
#define NMTC_FUNC_BITS_8            (1<<4)
#define NMTC_FUNC_LINES_1           (0<<3)
#define NMTC_FUNC_LINES_2           (1<<3)
#define NMTC_FUNC_FONT_5X8          (0<<2)
#define NMTC_FUNC_FONT_5X10         (1<<2)

#define NMTC_CGRAM_ADR_INST         (1<<6)

#define NMTC_DDRAM_ADR_INST         (1<<7)
#define NMTC_DDRAM_ADR_LINE1        (0<<6)
#define NMTC_DDRAM_ADR_LINE2        (1<<6)

#define NMTC_BUSY_ADR_READ_INST     NMTC_RW_READ

#define NMTC_DATA_WRITE_INST        NMTC_RS_DATA

#define NMTC_DATA_READ_INST         (NMTC_RS_DATA | NMTC_RW_READ)

#define NMTC_DELAY_INIT0        50e-3
#define NMTC_DELAY_INIT1         5e-3
#define NMTC_DELAY_INIT2       100e-6
#define NMTC_DELAY_CLEAR_HOME    2e-3
#define NMTC_DELAY_NORMAL       40e-6

//-----------------------------------------------------------------------------------------------------------------------
// nmtc_init

void nmtc_init(PIO pio, uint sm, float sysHz, uint pin_db8_rs_rw_en, uint32_t func0);

void nmtc_test(PIO pio, uint sm);
	
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

#endif
