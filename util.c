#include "util.h"

void util_i2c_set_baudrate_lt_1e6(i2c_hw_t *hw, uint baudrate) {
	// stolen near-verbatim from SDK
	uint freq_in = clock_get_hz(clk_sys);
		
    // TODO there are some subtleties to I2C timing which we are completely ignoring here
    uint period = (freq_in + baudrate / 2) / baudrate;
    uint lcnt = period * 3 / 5; // oof this one hurts
    uint hcnt = period - lcnt;

	// sda_tx_hold_count = freq_in [cycles/s] * 300ns * (1s / 1e9ns)
	// Reduce 300/1e9 to 3/1e7 to avoid numbers that don't fit in uint.
	// Add 1 to avoid division truncation.
	uint sda_tx_hold_count = ((freq_in * 3) / 10000000) + 1;

    hw->fs_scl_hcnt = hcnt;
    hw->fs_scl_lcnt = lcnt;
    hw->fs_spklen = lcnt < 16 ? 1 : lcnt / 16;
	hw->sda_hold = sda_tx_hold_count << I2C_IC_SDA_HOLD_IC_SDA_TX_HOLD_LSB
		| 0<<I2C_IC_SDA_HOLD_IC_SDA_RX_HOLD_LSB;
}
