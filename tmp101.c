#include "tmp101.h"

void tmp101_init_i2c(i2c_inst_t *i2c, uint pinVcc, uint pinSda, uint pinScl) {
    gpio_init(pinVcc);
    gpio_set_dir(pinVcc, GPIO_OUT);
	gpio_put(pinVcc, 1);
	i2c_init(i2c, 100000); // todo un-hardcode
	i2c_set_slave_mode(i2c, /*slave*/ false, /*slave_addr*/ 0);
	gpio_set_function(pinSda, GPIO_FUNC_I2C);
	gpio_set_function(pinScl, GPIO_FUNC_I2C);
	gpio_pull_up(pinSda);
	gpio_pull_up(pinScl);
}

// tlo and thi are actually signed int16_t, but this function treats them as uint16_t
void tmp101_init_chip(i2c_inst_t *i2c, uint8_t addr, uint config, uint16_t tlo, uint16_t thi) {
	uint8_t buf[2];
	
	tmp101_write_2(i2c, addr, TMP101_PTR_CONFIG, TMP101_CONFIG_RESOLUTION_12);
	tmp101_read_1(i2c, addr, buf);
	tmp101_write_3(i2c, addr, TMP101_PTR_TLO, tlo >> 8, tlo & 0xFF);
	tmp101_read_2(i2c, addr, buf);
	tmp101_write_3(i2c, addr, TMP101_PTR_THI, thi >> 8, thi & 0xFF);
	tmp101_read_2(i2c, addr, buf);
	tmp101_write_1(i2c, addr, TMP101_PTR_TEMP);
}
