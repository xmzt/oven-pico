#include "nmtc.h"
#include "util.h"

//-----------------------------------------------------------------------------------------------------------------------
// display

void nmtc_init(PIO pio, uint sm, float sysHz, uint pin_db8_rs_rw_en, uint32_t func0) {
	float div = sysHz / nmtc_pio_ClkHz;

	// init pins
	for(uint i = 0; i < 11; i++)
 		pio_gpio_init(pio, pin_db8_rs_rw_en + i);
	pio_sm_set_consecutive_pindirs(pio, sm, pin_db8_rs_rw_en, 11, true);

	// init program
	uint offset = util_pio_program_load(pio, &nmtc_pio_program, /*pio_program_offset*/ 32);
	pio_sm_config c = nmtc_pio_program_get_default_config(offset);
	sm_config_set_clkdiv(&c, div);
	sm_config_set_in_pins(&c, pin_db8_rs_rw_en);
	sm_config_set_in_shift(&c, /*shift_right*/ true, /*autopush*/ false, /*push_threashold*/ 32);
	sm_config_set_jmp_pin(&c, pin_db8_rs_rw_en + 7);
	sm_config_set_out_pins(&c, pin_db8_rs_rw_en, 10);
	sm_config_set_out_shift(&c, /*shift_right*/ true, /*autopush*/ false, /*push_threashold*/ 32);
	sm_config_set_set_pins(&c, pin_db8_rs_rw_en + 10, 1);
	// todo? is this redundant
	sm_config_set_out_shift(&c, /*shift_right*/ true, /*autopush*/ false, /*push_threashold*/ 32);
	pio_sm_init(pio, sm, offset, &c);
	pio_sm_set_enabled(pio, sm, true);
	pio_sm_put_blocking(pio, sm, nmtc_pio_Fifo0);
	printf("dispNmtcPioInit offset=%u sm=%u div=%f\n", offset, sm, div);
	
	// set function with fixed delays
	nmtc_put(pio, sm, func0 | nmtc_pio_DelayInit0 << 10);
	nmtc_put(pio, sm, func0 | nmtc_pio_DelayInit1 << 10);
	nmtc_put(pio, sm, func0 | nmtc_pio_DelayInit2 << 10 | 1<<31);
	nmtc_get(pio, sm);

	// remove PIO code no longer needed
	offset = util_pio_program_clear(pio, offset, nmtc_pio_wrap_target);
	printf("dispNmtcPioInit offset=%u\n", offset);

	nmtc_put(pio, sm, NMTC_DISPLAY_INST | NMTC_DISPLAY_OFF);
	nmtc_put(pio, sm, NMTC_CLEAR_INST);
	nmtc_put(pio, sm, NMTC_ENTRY_MODE_INST | NMTC_ENTRY_MODE_INC);
	nmtc_put(pio, sm, NMTC_DISPLAY_INST | NMTC_DISPLAY_ON | NMTC_DISPLAY_CURSOR_ON);
}

void nmtc_test(PIO pio, uint sm) {
	char ch = 0;
	char chE = 0x10;
	char chE2 = 0x20;

	nmtc_put(pio, sm, NMTC_DDRAM_ADR_INST | NMTC_DDRAM_ADR_LINE1);
	for(;;) {
		nmtc_put(pio, sm, NMTC_DATA_WRITE_INST | ch);
		if(++ch == chE) {
			if(ch == chE2) goto phase1_from_phase0;
			nmtc_put(pio, sm, NMTC_DDRAM_ADR_INST | NMTC_DDRAM_ADR_LINE2);
			chE += 0x10;
		}
	}

	for(;;) {
		sleep_ms(300);
		nmtc_put(pio, sm, NMTC_DATA_WRITE_INST | ch);
		if(++ch == chE) {
			if(ch == chE2) {
			phase1_from_phase0:
				nmtc_put(pio, sm, NMTC_DDRAM_ADR_INST | NMTC_DDRAM_ADR_LINE1);
				chE += 0x10;
				chE2 = chE + 0x10;
			} else {
				nmtc_put(pio, sm, NMTC_DDRAM_ADR_INST | NMTC_DDRAM_ADR_LINE2);
				chE += 0x10;
			}
		}
	}
}
