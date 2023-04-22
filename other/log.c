#include "log.h"
#include "base.h"

uint32_t g_logr_v[LOG_BLOCK_BUF_Z];
uint32_t *g_logr_v_e;
volatile uint32_t *g_logr_a;
volatile uint32_t *g_logr_b;

void log_pop_loop() {
	for(;;) {
		if(! g_logr_a[0]) continue;
		((log_fun_t*)g_logr_a[0])((uint32_t*)g_logr_a);
		g_logr_a[0] = 0;
		if((g_logr_a += LOG_BLOCK_Z) == g_logr_v_e) g_logr_a = g_logr_v;
	}
}

//-----------------------------------------------------------------------------------------------------------------------
// specific log callbacks

void log_nmtc_dma_irq(uint32_t *v) {
	printf("[nmtc] dma_irq intr=%p ints=%p\n", v[1], v[2]);
}

void log_nmtc_pio_run0_fin(uint32_t *v) {
	printf("[nmtc] run0_fin\n");
}

void log_nmtc_pio_start_fin(uint32_t *v) {
	printf("[nmtc] start_fin\n");
}

void log_tmp101_dma_irq(uint32_t *v) {
	printf("[tmp101] dma_irq intr=%p ints=%p rx_buf=%x,%x,%x,%x,%x,%x,%x temp=0x%x\n",
		   v[1], v[2],
		   ((uint32_t*)v[3])[0], ((uint32_t*)v[3])[1],
		   ((uint32_t*)v[3])[2], ((uint32_t*)v[3])[3],
		   ((uint32_t*)v[3])[4], ((uint32_t*)v[3])[5],
		   ((uint32_t*)v[3])[6],
		   v[4]);
}

void log_tmp101_tx_abrt(uint32_t *v) {
	printf("[tmp101] TX_ABRT_SOURCE=%p RAW_INTR_STAT=%p\n", v[1], v[2]);
}

void log_tmp101_vcc(uint32_t *v) {
	printf("[tmp101] log_tmp101_vcc %u\n", v[1]);
}
