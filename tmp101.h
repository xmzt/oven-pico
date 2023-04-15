#ifndef TMP101_H
#define TMP101_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#define TMP101_PTR_TEMP   0
#define TMP101_PTR_CONFIG 1
#define TMP101_PTR_TLO    2
#define TMP101_PTR_THI    3

#define TMP101_CONFIG_SHUTDOWN_DIS          (0<<0)
#define TMP101_CONFIG_SHUTDOWN_EN           (1<<0)
#define TMP101_CONFIG_THERMOSTAT_COMPARATOR (0<<1)
#define TMP101_CONFIG_THERMOSTAT_INTERRUPT  (1<<1)
#define TMP101_CONFIG_POLARITY_ALERT_LO     (0<<2)
#define TMP101_CONFIG_POLARITY_ALERT_HI     (1<<2)
#define TMP101_CONFIG_FAULT_QUEUE_1         (0b00<<3)
#define TMP101_CONFIG_FAULT_QUEUE_2         (0b01<<3)
#define TMP101_CONFIG_FAULT_QUEUE_4         (0b10<<3)
#define TMP101_CONFIG_FAULT_QUEUE_6         (0b11<<3)
#define TMP101_CONFIG_RESOLUTION_9          (0b00<<5) // 0.5C    40ms
#define TMP101_CONFIG_RESOLUTION_10         (0b01<<5) // 0.25C   80ms
#define TMP101_CONFIG_RESOLUTION_11         (0b10<<5) // 0.125C  160ms
#define TMP101_CONFIG_RESOLUTION_12         (0b11<<5) // 0.0625C 320ms
#define TMP101_CONFIG_ONESHOT_DIS           (0<<7)
#define TMP101_CONFIG_ONESHOT_EN            (1<<7)
#define TMP101_CONFIG_ALERT                 (1<<7)

//-----------------------------------------------------------------------------------------------------------------------
// temperature scales

inline static float tmp101_tc_of_buf(uint8_t buf[2]) {
	return (int16_t)(buf[0] << 8 | buf[1]) / 256.0;
}

inline static float tmp101_tf_of_tc(float tc) {
	return 1.8 * tc + 32.0;
}

//-----------------------------------------------------------------------------------------------------------------------
// initi

void tmp101_init_i2c(i2c_inst_t *i2c, uint pinVcc, uint pinSda, uint pinScl);

// tlo and thi are actually signed int16_t, but this function treats them as uint16_t
void tmp101_init_chip(i2c_inst_t *i2c, uint8_t addr, uint config, uint16_t tlo, uint16_t thi);

//-----------------------------------------------------------------------------------------------------------------------
// basic i/o

#if TMP101_DEBUG
inline static int tmp101_write_1(i2c_inst_t *i2c, uint8_t addr, uint8_t a) {
	int ret = i2c_write_blocking(i2c, addr, &a, 1, /*nostop*/ true);
	printf("tmp101_write_1 %02x [ret=%d]\n", a, ret);
	return ret;
}

inline static int tmp101_write_2(i2c_inst_t *i2c, uint8_t addr, uint8_t a, uint8_t b) {
	int ret = i2c_write_blocking(i2c, addr, (uint8_t[]) { a, b }, 2, /*nostop*/ true);
	printf("tmp101_write_2 %02x,%02x [ret=%d]\n", a, b, ret);
	return ret;
}

inline static int tmp101_write_3(i2c_inst_t *i2c, uint8_t addr, uint8_t a, uint8_t b, uint8_t c) {
	int ret = i2c_write_blocking(i2c, addr, (uint8_t[]) { a, b, c }, 3, /*nostop*/ true);
	printf("tmp101_write_3 %02x,%02x,%02x [ret=%d]\n", a, b, c, ret);
	return ret;
}

inline static int tmp101_read_1(i2c_inst_t *i2c, uint8_t addr, uint8_t *buf) {
	int ret = i2c_read_blocking(i2c, addr, buf, 1, /*nostop*/ true);
	printf("tmp101_read_1 %02x [ret=%d]\n", buf[0], ret);
	return ret;
}

inline static int tmp101_read_2(i2c_inst_t *i2c, uint8_t addr, uint8_t *buf) {
	int ret = i2c_read_blocking(i2c, addr, buf, 2, /*nostop*/ true);
	printf("tmp101_read_2 %02x,%02x [ret=%d]\n", buf[0], buf[1], ret);
	return ret;
}
#else
inline static int tmp101_write_1(i2c_inst_t *i2c, uint8_t addr, uint8_t a) {
	return i2c_write_blocking(i2c, addr, &a, 1, /*nostop*/ true);
}

inline static int tmp101_write_2(i2c_inst_t *i2c, uint8_t addr, uint8_t a, uint8_t b) {
	return i2c_write_blocking(i2c, addr, (uint8_t[]) { a, b }, 2, /*nostop*/ true);
}

inline static int tmp101_write_3(i2c_inst_t *i2c, uint8_t addr, uint8_t a, uint8_t b, uint8_t c) {
	return i2c_write_blocking(i2c, addr, (uint8_t[]) { a, b, c }, 3, /*nostop*/ true);
}

inline static int tmp101_read_1(i2c_inst_t *i2c, uint8_t addr, uint8_t *buf) {
	return i2c_read_blocking(i2c, addr, buf, 1, /*nostop*/ true);
}

inline static int tmp101_read_2(i2c_inst_t *i2c, uint8_t addr, uint8_t *buf) {
	return i2c_read_blocking(i2c, addr, buf, 2, /*nostop*/ true);
}
#endif // #if TMP101_DEBUG

#endif

