// raspberry pi pico, NMTC-S16205DRGHS display with SPLC780 controller
#include "base.h"
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
#include <stdio.h>

//-----------------------------------------------------------------------------------------------------------------------
// globals

uint32_t g_rando;
uint32_t g_test[8];
uint32_t g_pio0_ctrl;
mainq_fun_t * g_mainq_ring[MAINQ_RING_Z] __attribute__ (( aligned(0x80) ));
mainq_t g_mainq;
					  
//-----------------------------------------------------------------------------------------------------------------------
// main

int main() {
	stdio_init_all();
	sleep_ms(3000); // wait for stdio 

	g_rando = util_rando(32);
	float sysHz = clock_get_hz(clk_sys);
	printf("\n\n[%p] sysHz=%f\n", g_rando, sysHz);

	mainq_init(&g_mainq, g_mainq_ring);

	//-------------------------------------------------------------------------------------------------------------------
	// init peripherals before pads are enabled. these should not cause interrupt to be executed

	g_pio0_ctrl = 0;
	tmp101_init();
	nmtc_init();

	//-------------------------------------------------------------------------------------------------------------------
	// pins
	
    sio_hw->gpio_oe_set = 1<<PICO_DEFAULT_LED_PIN
		| 1<<PIN_TMP101_VCC;
    sio_hw->gpio_set = 1<<PICO_DEFAULT_LED_PIN;
	sio_hw->gpio_clr = 1<<PIN_TMP101_VCC;

	padsbank0_hw->io[PICO_DEFAULT_LED_PIN] = UTIL_PADS_BANK0_GPIO_OD_IE_DRV_PUE_PDE_SCH_SF
		(0, 1, PADS_BANK0_GPIO0_DRIVE_VALUE_4MA, 0, 0, 1, 0);
	iobank0_hw->io[PICO_DEFAULT_LED_PIN].ctrl = UTIL_IO_BANK0_GPIO_CTRL_FS
		(IO_BANK0_GPIO0_CTRL_FUNCSEL_VALUE_SIO_0);

	// tmp101

	padsbank0_hw->io[PIN_TMP101_VCC] = UTIL_PADS_BANK0_GPIO_OD_IE_DRV_PUE_PDE_SCH_SF
		(0, 1, PADS_BANK0_GPIO0_DRIVE_VALUE_12MA, 0, 0, 1, 0);
	iobank0_hw->io[PIN_TMP101_VCC].ctrl = UTIL_IO_BANK0_GPIO_CTRL_FS
		(IO_BANK0_GPIO0_CTRL_FUNCSEL_VALUE_SIO_0);

	padsbank0_hw->io[PIN_TMP101_SDA] = UTIL_PADS_BANK0_GPIO_OD_IE_DRV_PUE_PDE_SCH_SF
		(0, 1, PADS_BANK0_GPIO0_DRIVE_VALUE_4MA, 1, 0, 1, 0);
	iobank0_hw->io[PIN_TMP101_SDA].ctrl = UTIL_IO_BANK0_GPIO_CTRL_FS
		(IO_BANK0_GPIO0_CTRL_FUNCSEL_VALUE_I2C0_SDA);

	padsbank0_hw->io[PIN_TMP101_SCL] = UTIL_PADS_BANK0_GPIO_OD_IE_DRV_PUE_PDE_SCH_SF
		(0, 1, PADS_BANK0_GPIO0_DRIVE_VALUE_4MA, 1, 0, 1, 0);
	iobank0_hw->io[PIN_TMP101_SCL].ctrl = UTIL_IO_BANK0_GPIO_CTRL_FS
		(IO_BANK0_GPIO0_CTRL_FUNCSEL_VALUE_I2C0_SDA);
	
	// nmtc

	for(uint i = PIN_NMTC_DB8_RS_RW_EN, j = i + 11; i < j; i++) {
		padsbank0_hw->io[i] = UTIL_PADS_BANK0_GPIO_OD_IE_DRV_PUE_PDE_SCH_SF
			(0, 1, PADS_BANK0_GPIO0_DRIVE_VALUE_12MA, 0, 0, 1, 0);
		iobank0_hw->io[i].ctrl = UTIL_IO_BANK0_GPIO_CTRL_FS
			(IO_BANK0_GPIO0_CTRL_FUNCSEL_VALUE_PIO0_0);
	}

	//-------------------------------------------------------------------------------------------------------------------
	// start the action

	tmp101_start();
	nmtc_start();

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
		ts1 = timer_hw->timerawl;
		if(OVEN_WFIN_DUR <= (ts1 - ts0)) {
			printf("[wfiN] rate=%.4f\n", (float)TIMER_HZ * wfiN / (ts1 - ts0));
			wfiN = 0;
			ts0 = ts1;
		}
	}
	return 0;
}
