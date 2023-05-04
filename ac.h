#ifndef AC_H
#define AC_H

#include "base.h"

//-----------------------------------------------------------------------------------------------------------------------
// globals

extern isr_t *g_ac_dma_b_isr;

//-----------------------------------------------------------------------------------------------------------------------
// hardware, also in base.h

#define AC_PHASE_PRE_SAMPLE_DUR ((CLK_SYS_HZ / 1000) - 4) // 1ms, at clk_sys

#define AC_PHASE_SAMPLE_DUR ((CLK_SYS_HZ/2 / 1000) - 1) // 1ms, at clk_sys/2

#define AC_PHASE_ELAPSED(rxval) (4 + AC_PHASE_PRE_SAMPLE_DUR + 2*(AC_PHASE_SAMPLE_DUR - (rxval)))

#define AC_PHASE_ALARM_DUR 300000 // 1/3sec at 1MHz

//-----------------------------------------------------------------------------------------------------------------------
// api

void ac_dma_b_isr(void);

void ac_init(void);

#endif
