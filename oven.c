// raspberry pi pico, NMTC-S16205DRGHS display with SPLC780 controller
#include "base.h"
#include "log.h"
#include "nmtc.h"
#include "tmp101.h"
#include "util.h"

#include "hardware/clocks.h"
#include "hardware/resets.h"
#include "hardware/structs/dma.h"
#include "hardware/structs/rosc.h"
#include "hardware/structs/sio.h"
#include "hardware/structs/padsbank0.h"
#include "hardware/structs/iobank0.h"
#include "hardware/sync.h"

//-----------------------------------------------------------------------------------------------------------------------
// globals

uint32_t g_pio0_ctrl;

//-----------------------------------------------------------------------------------------------------------------------
// main

int main() {
	stdio_init_all();
	sleep_ms(3000); // wait for stdio 
	logr_init();
	float sysHz = clock_get_hz(clk_sys);
	printf("\n\n[%08x] sysHz=%f\n", util_rando(32), sysHz);

	//-------------------------------------------------------------------------------------------------------------------
	// init peripherals to try and get everything ready before turning the pads on

	g_pio0_ctrl = 0;
	tmp101_init_hw();
	nmtc_init_start();

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
			(0, 1, PADS_BANK0_GPIO0_DRIVE_VALUE_4MA, 0, 0, 1, 0);
		iobank0_hw->io[i].ctrl = UTIL_IO_BANK0_GPIO_CTRL_FS
			(IO_BANK0_GPIO0_CTRL_FUNCSEL_VALUE_PIO0_0);
	}

	//-------------------------------------------------------------------------------------------------------------------
	// start the action

	tmp101_start();
	nmtc_start();

	//-------------------------------------------------------------------------------------------------------------------
	// sleep mode, power down, main loop

	//todo sleep mode, power down
	//clocks_hw->sleep_en1 = 0;
	//clocks_hw->wake_en1 = 0;
	//printf("sleep_en0=%08x\n", clocks_hw->sleep_en0);
	//printf(" wake_en0=%08x\n", clocks_hw->wake_en0);
	//printf("sleep_en1=%08x\n", clocks_hw->sleep_en1);
	//printf(" wake_en1=%08x\n", clocks_hw->wake_en1);

	// main loop
	//for(;;) __wfi();
	log_pop_loop();
	return 0;
}
