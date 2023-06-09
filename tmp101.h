#ifndef TMP101_H
#define TMP101_H

#include "base.h"

//-----------------------------------------------------------------------------------------------------------------------
// hardware, also in base.h

#define TMP101_TAR 0b1001001
#define TMP101_BAUD 100000
#define TMP101_CONFIG TMP101_CONFIG_RESOLUTION_12
#define TMP101_TLIM_LO 0x8000
#define TMP101_TLIM_HI 0x7FFF
#define TMP101_TEMP_REQ_ALARM_DUR 1000000
#define TMP101_VCC_OFF_ALARM_DUR 100000 // 100ms
#define TMP101_VCC_ON_ALARM_DUR 100000 // 100ms

//-----------------------------------------------------------------------------------------------------------------------
// constants

#define TMP101_PTR_TEMP   0
#define TMP101_PTR_CONFIG 1
#define TMP101_PTR_TLIM_LO   2
#define TMP101_PTR_TLIM_HI   3

#define TMP101_CONFIG_SHUTDOWN_DIS          (0 << 0)
#define TMP101_CONFIG_SHUTDOWN_EN           (1 << 0)
#define TMP101_CONFIG_THERMOSTAT_COMPARATOR (0 << 1)
#define TMP101_CONFIG_THERMOSTAT_INTERRUPT  (1 << 1)
#define TMP101_CONFIG_POLARITY_ALERT_LO     (0 << 2)
#define TMP101_CONFIG_POLARITY_ALERT_HI     (1 << 2)
#define TMP101_CONFIG_FAULT_QUEUE_1         (0b00 << 3)
#define TMP101_CONFIG_FAULT_QUEUE_2         (0b01 << 3)
#define TMP101_CONFIG_FAULT_QUEUE_4         (0b10 << 3)
#define TMP101_CONFIG_FAULT_QUEUE_6         (0b11 << 3)
#define TMP101_CONFIG_RESOLUTION_9          (0b00 << 5) // 0.5C    40ms
#define TMP101_CONFIG_RESOLUTION_10         (0b01 << 5) // 0.25C   80ms
#define TMP101_CONFIG_RESOLUTION_11         (0b10 << 5) // 0.125C  160ms
#define TMP101_CONFIG_RESOLUTION_12         (0b11 << 5) // 0.0625C 320ms
#define TMP101_CONFIG_ONESHOT_DIS           (0 << 7)
#define TMP101_CONFIG_ONESHOT_EN            (1 << 7)
#define TMP101_CONFIG_ALERT                 (1 << 7)

#define TMP101_STATE_0            0
#define TMP101_STATE_CONFIG       1
#define TMP101_STATE_TLIM_LO      2
#define TMP101_STATE_TLIM_HI      3

//-----------------------------------------------------------------------------------------------------------------------
// globals

extern uint32_t g_tmp101_temp;

//-----------------------------------------------------------------------------------------------------------------------
// temperature scales

inline static float tmp101_tc_of_raw(uint raw) {
	return raw / 256.0;
}

inline static float tmp101_tf_of_tc(float tc) {
	return 1.8 * tc + 32.0;
}

//-----------------------------------------------------------------------------------------------------------------------
// init

void tmp101_dma_b_isr(void);

void tmp101_init();

#endif
