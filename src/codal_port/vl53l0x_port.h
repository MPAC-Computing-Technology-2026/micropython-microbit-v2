/*
 * vl53l0x_port.h - platform backend hooks for the VL53L0X driver.
 *
 * The driver (vl53l0x.c) is pure, platform-agnostic C. It reaches the outside
 * world only through the three functions below. Each platform supplies its own
 * implementation:
 *
 *   micro:bit (codal_port) -> modcomptech_vl53l0x.c   (micro:bit HAL)
 *   ESP32     (Arduino)    -> vl53l0x_esp32.cpp        (Wire)
 *
 * To add a backend, implement these three functions against your I2C peripheral
 * and link them in. Nothing else changes.
 *
 * I2C addresses are 7-bit. 'stop' true means emit a STOP condition.
 * The i2c functions return 0 on success, nonzero on error.
 */
#ifndef VL53L0X_PORT_H
#define VL53L0X_PORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int vl53l0x_i2c_write(uint8_t addr, const uint8_t *buf, size_t len, bool stop);
int vl53l0x_i2c_read(uint8_t addr, uint8_t *buf, size_t len, bool stop);
uint32_t vl53l0x_millis(void);

#ifdef __cplusplus
}
#endif

#endif /* VL53L0X_PORT_H */
