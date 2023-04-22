#ifndef LOG_H
#define LOG_H

#include "pico/stdlib.h"
#include "stdio.h"

#define LOG_BLOCK_Z (1<<3)
#define LOG_BLOCK_N (1<<5)
#define LOG_BLOCK_BUF_Z (LOG_BLOCK_Z * LOG_BLOCK_N)

typedef void log_fun_t(uint32_t *block);

extern uint32_t g_logr_v[LOG_BLOCK_BUF_Z];
extern uint32_t *g_logr_v_e;
extern volatile uint32_t *g_logr_a;
extern volatile uint32_t *g_logr_b;

inline static void logr_init() {
	g_logr_v_e = g_logr_v + LOG_BLOCK_BUF_Z;
	g_logr_b = g_logr_a = g_logr_v;
	g_logr_v[0] = 0;
}

inline static void log_push(uint32_t *v) {
	g_logr_b[0] = v[0];
	g_logr_b[1] = v[1];
	g_logr_b[2] = v[2];
	g_logr_b[3] = v[3];
	g_logr_b[4] = v[4];
	g_logr_b[5] = v[5];
	g_logr_b[6] = v[6];
	g_logr_b[7] = v[7];
	if((g_logr_b += LOG_BLOCK_Z) == g_logr_v_e) g_logr_b = g_logr_v;
}

#define LOG_PUSH(fun, ...) log_push((uint32_t[8]){(uint32_t)(fun), __VA_ARGS__})

void log_pop_loop();

//-----------------------------------------------------------------------------------------------------------------------
// specific log callbacks

void log_nmtc_pio_run0_fin(uint32_t *v);
void log_nmtc_pio_start_fin(uint32_t *v);

void log_tmp101_dma_irq(uint32_t *v);
void log_tmp101_tx_abrt(uint32_t *v);
void log_tmp101_vcc(uint32_t *v);

#endif
