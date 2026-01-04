/**
 * @file i2c.h
 * @brief Helper functions for I2C register read/write in Zephyr.
 *
 * This module provides simple I2C read/write utilities for devices
 * described via Zephyr devicetree. It allows reading multiple registers,
 * writing single registers, and checking device readiness.
 */

#ifndef I2C_H
#define I2C_H

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <stdint.h>

/**
 * @brief Read multiple bytes from a device register over I2C.
 *
 * This function reads `len` bytes starting from the specified register
 * of the device. The device must be described via a devicetree `i2c_dt_spec`.
 *
 * @param dev Pointer to the I2C device descriptor (from devicetree).
 * @param reg Register address to start reading.
 * @param buf Pointer to the buffer to store the data.
 * @param len Number of bytes to read.
 * @return 0 on success, negative errno code on failure.
 */
int i2c_read_regs(const struct i2c_dt_spec *dev, uint8_t reg, uint8_t *buf, size_t len);

/**
 * @brief Write a single byte to a device register over I2C.
 *
 * This function writes one byte (`val`) to the specified register of the device.
 *
 * @param dev Pointer to the I2C device descriptor.
 * @param reg Register address to write to.
 * @param val Value to write.
 * @return 0 on success, negative errno code on failure.
 */
int i2c_write_reg(const struct i2c_dt_spec *dev, uint8_t reg, uint8_t val);

/**
 * @brief Check if a device is reachable on the I2C bus.
 *
 * Verifies that the I2C bus is ready and the device can be communicated with.
 *
 * @param dev Pointer to the I2C device descriptor.
 * @return 0 if ready, -ENODEV if the bus or device is not available.
 */
int i2c_dev_ready(const struct i2c_dt_spec *dev);

#endif // I2C_H
