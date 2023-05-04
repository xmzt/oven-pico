// raspberry pi pico, NMTC-S16205DRGHS display with SPLC780 controller
#include "base.h"
#include "ac.h"
#include "mainq.h"
#include "nmtc.h"
#include "tmp101.h"
#include "util.h"

#include "hardware/clocks.h"
//#include "hardware/resets.h"
#include "hardware/structs/nvic.h"
//#include "hardware/structs/rosc.h"
#include "hardware/structs/sio.h"
#include "hardware/structs/padsbank0.h"
#include "hardware/structs/iobank0.h"
#include "hardware/sync.h"
#include "stdio.h"

//-----------------------------------------------------------------------------------------------------------------------
// globals

uint32_t g_rando;
uint32_t g_test[8];
uint32_t g_pio0_ctrl;
mainq_fun_t * g_mainq_ring[MAINQ_RING_Z] __attribute__ (( aligned(0x80) ));
mainq_t g_mainq;

//-----------------------------------------------------------------------------------------------------------------------
// isr shared

static void dma0_isr(void) {
	uint32_t intr = dma_hw->intr;
	if((1 << AC_DMA_B) & intr) (*g_ac_dma_b_isr)();
	if((1 << NMTC_DMA_B) & intr) (*g_nmtc_dma_b_isr)();
	if((1 << TMP101_DMA_B) & intr) tmp101_dma_b_isr();
}

inline static void dma0_isr_init(void) {
	util_irq_isr(DMA_IRQ_0, dma0_isr);
	nvic_hw->iser = 1 << DMA_IRQ_0;
	dma_hw->inte0 = 1 << AC_DMA_B | 1 << NMTC_DMA_B | 1 << TMP101_DMA_B;
}

//-----------------------------------------------------------------------------------------------------------------------
// led_init

void led_init(void) {
    sio_hw->gpio_oe_set = 1 << PICO_DEFAULT_LED_PIN;
    sio_hw->gpio_set = 1 << PICO_DEFAULT_LED_PIN;
	padsbank0_hw->io[PICO_DEFAULT_LED_PIN] = UTIL_PADS_BANK0_GPIO_OD_IE_DRIVE_PUE_PDE_SCHMITT_SLEWFAST
		(0, 1, PADS_BANK0_GPIO0_DRIVE_VALUE_4MA, 0, 0, 1, 0);
	iobank0_hw->io[PICO_DEFAULT_LED_PIN].ctrl = UTIL_IO_BANK0_GPIO_CTRL_FUNCSEL_OUTOVER_OEOVER_INOVER_IRQOVER
		(IO_BANK0_GPIO0_CTRL_FUNCSEL_VALUE_SIO_0, 0, 0, 0, 0);
}
	
//-----------------------------------------------------------------------------------------------------------------------
// main

int main() {
	stdio_init_all();
	sleep_ms(3000); // wait for stdio 

	g_rando = util_rando(32);
	clock_hw_t *ref = &clocks_hw->clk[clk_ref];
	clock_hw_t *sys = &clocks_hw->clk[clk_sys];
	printf("\n\n[%p]\n"
		   "    systick  SYST_CSR=%p SYST=RVR=%u SYST_CVR=%u SYST_CALIB=%p\n"
		   "    watchdog CTRL=%p LOAD=%p TICK=%p\n"
		   "    clk_ref CTRL=%p DIV=%p SELECTED=%p\n"
		   "    clk_sys CTRL=%p DIV=%p SELECTED=%p\n"
		   "    pll_sys CS=%p PWR=%p FBDIV_INT=%p PRIM=%p\n",
		   g_rando, 
		   systick_hw->csr, systick_hw->rvr, systick_hw->cvr, systick_hw->calib,
		   watchdog_hw->ctrl, watchdog_hw->load, watchdog_hw->tick,
		   ref->ctrl, ref->div, ref->selected,
		   sys->ctrl, sys->div, sys->selected,
		   pll_sys_hw->cs, pll_sys_hw->pwr, pll_sys_hw->fbdiv_int, pll_sys_hw->prim);
	
	mainq_init(&g_mainq, g_mainq_ring);
	g_pio0_ctrl = 0;

	//-------------------------------------------------------------------------------------------------------------------
	// init components

	dma0_isr_init();

	led_init();
	tmp101_init();
	nmtc_init();
	ac_init();
	
	//-------------------------------------------------------------------------------------------------------------------
	// main loop

	printf("[regs] NVIC_ISER=%p SLEEP_EN0=%p SLEEP_EN1=%p\n",
		   nvic_hw->iser, clocks_hw->sleep_en0, clocks_hw->sleep_en1);
	
	uint32_t ts0 = timer_hw->timerawl;
	uint32_t ts1;
	uint32_t wfiN = 0;
	mainq_fun_t *fun;
	for(;;) {
		__wfi();
		wfiN++;
		if((fun = mainq_con1(&g_mainq)))
			fun();
		// stats
		ts1 = timer_hw->timerawl;
		if(OVEN_WFIN_DUR <= (ts1 - ts0)) {
			printf("[main] wfi/sec=%.4f q.z_max=%u q.dropN=%u\n",
				   (float)TIMER_HZ * wfiN / (ts1 - ts0),
				   g_mainq.z_max,
				   g_mainq.dropN);
			wfiN = 0;
			ts0 = ts1;
		}
	}
	return 0;
}
