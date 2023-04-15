// raspberry pi pico, NMTC-S16205DRGHS display with SPLC780 controller

#define NMTC_DEBUG 0
#include "nmtc.h"

#define TMP101_DEBUG 1
#include "tmp101.h"

#include "util.h"

#include "hardware/sync.h"

// VBUS = USB 5V
// VSYS = VBUS - Schottky*1

//-----------------------------------------------------------------------------------------------------------------------
// nmtc display 

// GND disp.1.vss logic gnd
// 3V3 disp.2.vdd logic power (VDD-VSS: -0.3 - 7.0)
// ?GND disp.3.vee.vo lcd power neg (VLCD: VDD-15.0 - VDD+0.3, recommend 4.8@-20C 4.5@25C 4.2@70C)
// DB[0:7] = disp[7:14]
// RS = disp.4.rs
// RW = disp.5.rw
// EN = disp.6.en

#define DISP_PIN_DB8_RS_RW_EN 0

#define DISP_FUNC (NMTC_FUNC_INST | NMTC_FUNC_BITS_8 | NMTC_FUNC_LINES_2 | NMTC_FUNC_FONT_5X8)

#define DISP_PIO pio0
#define DISP_SM 0

//-----------------------------------------------------------------------------------------------------------------------
// tmp101 i2c temperature sensor

#define TMP101_PIN_VCC 11 // yel tmp101.4.v+
#define TMP101_PIN_SDA 12 // blu tmp101.6.sda
#define TMP101_PIN_SCL 13 // yel tmp101.1.scl
// GND blu tmp101.1.gnd

#define TMP101_I2C i2c0
#define TMP101_ADDR 0b1001001

#define TMP101_CONFIG TMP101_CONFIG_RESOLUTION_12
#define TMP101_TLO 0x8000
#define TMP101_THI 0x7FFF

#define TMP101_ALARM_ID 0
#define TMP101_ALARM_IRQ_ID TIMER_IRQ_0
#define TMP101_ALARM_INTERVAL 1000000

//-----------------------------------------------------------------------------------------------------------------------
// power supply ? todo is this implemented
//
// Rvdd,vee = 8.5kohm min. 5V / 8500 = 0.588mA
// target 1mA @ -5V, 100 khz freq

//-----------------------------------------------------------------------------------------------------------------------
// main

void tmp101_alarm_irq(void) {
	// clear interrupt and reschedule
	hw_clear_bits(&timer_hw->intr, 1u << TMP101_ALARM_ID);
	util_alarm_inc(TMP101_ALARM_ID, TMP101_ALARM_INTERVAL);

	printf("tmp101_alarm_irq\n");
	uint8_t buf[2];
	tmp101_read_2(TMP101_I2C, TMP101_ADDR, buf);
	float tc = tmp101_tc_of_buf(buf);
	float tf = tmp101_tf_of_tc(tc);
	printf("tmp101 buf=%02x,%02x C=%.4f F=%.4f\n", buf[0], buf[1], tc, tf);

	char line1[16];
	sprintf(line1, "C=%.4f", tc);
	char line2[16];
	sprintf(line2, "F=%.4f", tf);
			
	char *x;
	nmtc_put(DISP_PIO, DISP_SM, NMTC_DDRAM_ADR_INST | NMTC_DDRAM_ADR_LINE1);
	for(x = line1; *x; x++)
		nmtc_put(DISP_PIO, DISP_SM, NMTC_DATA_WRITE_INST | *x);
	nmtc_put(DISP_PIO, DISP_SM, NMTC_DDRAM_ADR_INST | NMTC_DDRAM_ADR_LINE2);
	for(x = line2; *x; x++)
		nmtc_put(DISP_PIO, DISP_SM, NMTC_DATA_WRITE_INST | *x);
}

int main() {
	stdio_init_all();
	sleep_ms(3000); // wait for stdio 

	float sysHz = clock_get_hz(clk_sys);
	printf("main sysHz=%f\n", sysHz);
	nmtc_init(DISP_PIO, DISP_SM, sysHz, DISP_PIN_DB8_RS_RW_EN, DISP_FUNC);
	tmp101_init_i2c(TMP101_I2C, TMP101_PIN_VCC, TMP101_PIN_SDA, TMP101_PIN_SCL);
	tmp101_init_chip(TMP101_I2C, TMP101_ADDR, TMP101_CONFIG, TMP101_TLO, TMP101_THI);

	gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
	gpio_put(PICO_DEFAULT_LED_PIN, 1);
	util_alarm_set(TMP101_ALARM_ID, TMP101_ALARM_IRQ_ID, tmp101_alarm_irq, timer_hw->timerawl + TMP101_ALARM_INTERVAL);

	// todo
	//clocks_hw->sleep_en1 = 0;
	//clocks_hw->wake_en1 = 0;

	printf("sleep_en0=%08x\n", clocks_hw->sleep_en0);
	printf(" wake_en0=%08x\n", clocks_hw->wake_en0);
	printf("sleep_en1=%08x\n", clocks_hw->sleep_en1);
	printf(" wake_en1=%08x\n", clocks_hw->wake_en1);
	
	// main loop
	for(;;) __wfi();
	return 0;
}
