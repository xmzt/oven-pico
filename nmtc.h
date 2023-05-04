#ifndef NMTC_H
#define NMTC_H

// Raspberry Pi Pico, PNMTC-S16205DRGHS display with SPLC780 controller

#include "base.h"

//-----------------------------------------------------------------------------------------------------------------------
// globals

extern isr_t *g_nmtc_dma_b_isr;

//-----------------------------------------------------------------------------------------------------------------------
// hardware, also in base.h

#define NMTC_PIO_CLKDIV_INT  4
#define NMTC_PIO_CLKDIV_FRAC 0

#define NMTC_PIO_CLKHZ            (CLK_SYS_HZ / NMTC_PIO_CLKDIV_INT) // 31250 * KHZ
#define NMTC_PIO_DELAY_0          (NMTC_PIO_CLKHZ * 5/100)    // 5e-2 sec
#define NMTC_PIO_DELAY_1          (NMTC_PIO_CLKHZ * 5/1000)   // 5e-3 sec
#define NMTC_PIO_DELAY_2          (NMTC_PIO_CLKHZ * 1/10000)  // 1e-4 sec
//#define NMTC_PIO_DELAY_CLEAR_HOME (NMTC_PIO_CLKHZ * 2/1000)   // 2e-3 sec (min 1.52ms @ fosc=270kHz)
//#define NMTC_PIO_DELAY_NORMAL     (NMTC_PIO_CLKHZ * 4/100000) // 4e-5 sec (min 38us @ fosc=270kHz)

//-----------------------------------------------------------------------------------------------------------------------
// SPLC780 register descriptions

#define NMTC_RS_INST                (0 << 8)
#define NMTC_RS_DATA                (1 << 8)

#define NMTC_RW_WRITE               (0 << 9)
#define NMTC_RW_READ                (1 << 9)

#define NMTC_CLEAR_INST             (1 << 0)

#define NMTC_RETURN_HOME_INST       (1 << 1)

#define NMTC_ENTRY_MODE_INST        (1 << 2)
#define NMTC_ENTRY_MODE_DEC         (0 << 1)
#define NMTC_ENTRY_MODE_INC         (1 << 1)
#define NMTC_ENTRY_MODE_NSHIFT      (0 << 0)
#define NMTC_ENTRY_MODE_SHIFT       (1 << 0)

#define NMTC_DISPLAY_INST           (1 << 3)
#define NMTC_DISPLAY_OFF            (0 << 2)
#define NMTC_DISPLAY_ON             (1 << 2)
#define NMTC_DISPLAY_CURSOR_OFF     (0 << 1)
#define NMTC_DISPLAY_CURSOR_ON      (1 << 1)
#define NMTC_DISPLAY_BLINK_OFF      (0 << 0)
#define NMTC_DISPLAY_BLINK_ON       (1 << 0)

#define NMTC_SHIFT_INST             (1 << 4)
#define NMTC_SHIFT_CURSOR           (0 << 3)
#define NMTC_SHIFT_DISPLAY          (1 << 3)
#define NMTC_SHIFT_LEFT             (0 << 2)
#define NMTC_SHIFT_RIGHT            (1 << 2)

#define NMTC_FUNC_INST              (1 << 5)
#define NMTC_FUNC_BITS_4            (0 << 4)
#define NMTC_FUNC_BITS_8            (1 << 4)
#define NMTC_FUNC_LINES_1           (0 << 3)
#define NMTC_FUNC_LINES_2           (1 << 3)
#define NMTC_FUNC_FONT_5X8          (0 << 2)
#define NMTC_FUNC_FONT_5X10         (1 << 2)

#define NMTC_CGRAM_ADR_INST         (1 << 6)

#define NMTC_DDRAM_ADR_INST         (1 << 7)
#define NMTC_DDRAM_ADR_LINE1        (0 << 6)
#define NMTC_DDRAM_ADR_LINE2        (1 << 6)

#define NMTC_BUSY_READ_INST         NMTC_RW_READ

#define NMTC_DATA_WRITE_INST        NMTC_RS_DATA

#define NMTC_DATA_READ_INST         (NMTC_RS_DATA | NMTC_RW_READ)

//-----------------------------------------------------------------------------------------------------------------------
// composite inst
 
#define NMTC_START_FUNC (NMTC_FUNC_INST | NMTC_FUNC_BITS_8 | NMTC_FUNC_LINES_2 | NMTC_FUNC_FONT_5X8)

#define NMTC_PIO_PINDIRS_R        0x300
#define NMTC_PIO_PINDIRS_W        0x3FF

#define NMTC_PIO_INST_R(pins, res) (NMTC_PIO_PINDIRS_R | (pins) << 10 | (res) << 20)
#define NMTC_PIO_INST_W(pins, res) (NMTC_PIO_PINDIRS_W | (pins) << 10 | (res) << 20)

#define NMTC_PIO_RUN_X (NMTC_PIO_PINDIRS_R | NMTC_BUSY_READ_INST << 10)

//-----------------------------------------------------------------------------------------------------------------------
// nmtc_q

typedef struct nmtc_q_item_t nmtc_q_item_t;

struct nmtc_q_item_t {
	nmtc_q_item_t *q_next;
	uint status;
	uint32_t read_addr;
	uint32_t transfer_count;
};

void nmtc_q_item_post_nisr(nmtc_q_item_t *self, uint32_t read_addr, uint32_t transfer_count);

uint32_t * nmtc_cmdV_from_u8(uint32_t *dst, const uint8_t *src);

//-----------------------------------------------------------------------------------------------------------------------
// nmtc_init

void nmtc_init();
void nmtc_start_fin();

void nmtc_dump_cmdV();

#endif
