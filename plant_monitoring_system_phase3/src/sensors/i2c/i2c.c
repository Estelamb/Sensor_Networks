/**
 * @file i2c.c
 * @brief Implementation of I2C register read/write helpers for Zephyr.
 *
 * This module provides simple functions to read multiple registers, write
 * a single register, and check if an I2C device is ready. Devices are
 * described using Zephyr devicetree `i2c_dt_spec`.
 */

#include "i2c.h"
#include <zephyr/sys/printk.h>

/**
 * @brief Read multiple bytes from a device starting at a given register.
 *
 * This function sends the register address first and then reads `len` bytes
 * into the provided buffer. Uses `i2c_write_read_dt` internally.
 *
 * @param dev Pointer to the I2C device descriptor.
 * @param reg Register address to start reading.
 * @param buf Buffer to store the read bytes.
 * @param len Number of bytes to read.
 * @return 0 on success, negative errno code on failure.
 */
int i2c_read_regs(const struct i2c_dt_spec *dev, uint8_t reg, uint8_t *buf, size_t len) {
    return i2c_write_read_dt(dev, &reg, 1, buf, len);
}

/**
 * @brief Write a single byte to a specific register of the I2C device.
 *
 * This function sends the register address followed by the byte value.
 * Uses `i2c_write_dt` internally.
 *
 * @param dev Pointer to the I2C device descriptor.
 * @param reg Register address to write to.
 * @param val Value to write.
 * @return 0 on success, negative errno code on failure.
 */
int i2c_write_reg(const struct i2c_dt_spec *dev, uint8_t reg, uint8_t val) {
    uint8_t data[2] = { reg, val };
    return i2c_write_dt(dev, data, sizeof(data));
}

/**
 * @brief Check if the I2C device is ready for communication.
 *
 * This function verifies that the device exists on the bus and is
 * reachable. Prints an error message if not ready.
 *
 * @param dev Pointer to the I2C device descriptor.
 * @return 0 if ready, -ENODEV if not reachable.
 */
int i2c_dev_ready(const struct i2c_dt_spec *dev) {
    if (!i2c_is_ready_dt(dev)) {
        printk("I2C device at address 0x%02X not ready\n", dev->addr);
        return -ENODEV;
    }
    return 0;
}
