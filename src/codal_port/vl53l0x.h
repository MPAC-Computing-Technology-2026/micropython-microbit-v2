/*
 * vl53l0x.h - pure-C VL53L0X time-of-flight driver (public API).
 *
 * Platform-agnostic. Port the I2C/timing hooks in vl53l0x_port.h per platform.
 * Single instance: state lives in file-scope globals in vl53l0x.c, so there is
 * one sensor per build. This matches both intended uses (the micro:bit and
 * ESP32 sensor-module tracks each drive one VL53L0X).
 *
 * Pure-C port of Pololu's VL53L0X Arduino library (MIT), itself based on ST's
 * VL53L0X API (STSW-IMG005) and datasheet.
 *
 * The bus must be initialised by the caller before vl53l0x_init() (the driver
 * never touches bus setup, since that is platform-specific):
 *   micro:bit: microbit_hal_i2c_init(...) / microbit.i2c
 *   ESP32:     Wire.begin(sda, scl); Wire.setClock(400000);
 */
#ifndef VL53L0X_H
#define VL53L0X_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VL53L0X_ADDRESS_DEFAULT 0x29   /* 7-bit */
#define VL53L0X_OUT_OF_RANGE    65535  /* returned by read_* on timeout */

/* Configure and calibrate the sensor. io_2v8 true selects 2V8 I/O mode.
 * Returns false if the model id is wrong (absent/miswired sensor) or
 * calibration fails. The bus must already be up. */
bool vl53l0x_init(bool io_2v8);

/* Change the 7-bit I2C address (writes the device register and updates the
 * driver's stored address). */
void vl53l0x_set_address(uint8_t new_addr);
uint8_t vl53l0x_get_address(void);

/* Read timeout in milliseconds applied to the blocking read/calibration waits.
 * 0 disables the timeout (waits forever). Default is 500 ms. */
void vl53l0x_set_timeout(uint16_t ms);
uint16_t vl53l0x_get_timeout(void);

/* True if a read timed out since the last call; clears the flag. */
bool vl53l0x_timeout_occurred(void);

/* Signal rate limit in MCPS (default 0.25). */
bool vl53l0x_set_signal_rate_limit(float limit_mcps);
float vl53l0x_get_signal_rate_limit(void);

/* Measurement timing budget in microseconds (longer = more accurate, min ~20000). */
bool vl53l0x_set_measurement_timing_budget(uint32_t budget_us);
uint32_t vl53l0x_get_measurement_timing_budget(void);

/* Continuous ranging. period_ms 0 = back-to-back, else inter-measurement ms. */
void vl53l0x_start_continuous(uint32_t period_ms);
void vl53l0x_stop_continuous(void);
uint16_t vl53l0x_read_range_continuous_mm(void);

/* Single-shot range in mm. Returns VL53L0X_OUT_OF_RANGE on timeout. */
uint16_t vl53l0x_read_range_single_mm(void);

/* Return code of the last I2C transaction (0 = ok); for debugging. */
int vl53l0x_last_status(void);

#ifdef __cplusplus
}
#endif

#endif /* VL53L0X_H */
