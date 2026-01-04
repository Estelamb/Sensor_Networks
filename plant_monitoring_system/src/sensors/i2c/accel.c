/**
 * @file accel.c
 * @brief 3-axis accelerometer driver implementation over I2C.
 *
 * Provides initialization, reading of raw X/Y/Z data, and conversion
 * functions to g or m/s² units. Supports range selection and standby/active modes.
 */

#include "accel.h"
#include "i2c.h"
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>

/**
 * @brief Set accelerometer measurement range.
 *
 * @param dev Pointer to I2C device descriptor.
 * @param range Range value: ACCEL_2G, ACCEL_4G, ACCEL_8G.
 * @return 0 on success, negative errno code on failure.
 */
int accel_set_range(const struct i2c_dt_spec *dev, uint8_t range) {
    return i2c_write_reg(dev, ACCEL_REG_XYZ_DATA_CFG, range & 0x03);
}

/**
 * @brief Put accelerometer into standby mode.
 *
 * Standby mode allows configuration changes (range, filters, etc.).
 *
 * @param dev Pointer to I2C device descriptor.
 * @return 0 on success, negative errno code on failure.
 */
int accel_set_standby(const struct i2c_dt_spec *dev)
{
    uint8_t ctrl1;
    int ret = i2c_read_regs(dev, ACCEL_REG_CTRL1, &ctrl1, 1);
    if (ret < 0) return ret;

    ctrl1 &= ~0x01; // Clear ACTIVE bit
    return i2c_write_reg(dev, ACCEL_REG_CTRL1, ctrl1);
}

/**
 * @brief Put accelerometer into active mode.
 *
 * Active mode enables measurement and data output.
 *
 * @param dev Pointer to I2C device descriptor.
 * @return 0 on success, negative errno code on failure.
 */
int accel_set_active(const struct i2c_dt_spec *dev)
{
    uint8_t ctrl1;
    int ret = i2c_read_regs(dev, ACCEL_REG_CTRL1, &ctrl1, 1);
    if (ret < 0) return ret;

    ctrl1 |= 0x01; // Set ACTIVE bit
    return i2c_write_reg(dev, ACCEL_REG_CTRL1, ctrl1);
}

/**
 * @brief Initialize accelerometer.
 *
 * Checks device presence (WHO_AM_I), sets standby mode,
 * configures measurement range, and activates measurement mode.
 *
 * @param dev Pointer to I2C device descriptor.
 * @param range Measurement range: ACCEL_2G, ACCEL_4G, ACCEL_8G.
 * @return 0 on success, negative errno code on failure.
 */
int accel_init(const struct i2c_dt_spec *dev, uint8_t range) {
    printk("[ACCEL] - Initializing ACCEL...\n");

    int ret = i2c_dev_ready(dev);
    if (ret < 0) return ret;

    uint8_t whoami;
    ret = i2c_read_regs(dev, ACCEL_REG_WHO_AM_I, &whoami, 1);
    if (ret < 0 || whoami != ACCEL_WHO_AM_I_VALUE) {
        printk("[ACCEL] - ACCEL WHO_AM_I mismatch: 0x%02X\n", whoami);
        return -EIO;
    }

    printk("ACCEL detected at 0x%02X\n", dev->addr);

    if (accel_set_standby(dev) < 0) {
        printk("[ACCEL] - Failed to set ACCEL to Standby mode\n");
        return -EIO;
    }

    if (accel_set_range(dev, range) < 0) {
        printk("[ACCEL] - Failed to set range to %d\n", range);
        return -EIO;
    }

    printk("[ACCEL] - Accelerometer initialized successfully with range %dG\n", range);

    return accel_set_active(dev);
}

/**
 * @brief Read raw accelerometer X, Y, Z values.
 *
 * Each axis is 14-bit, left-aligned in MSB/LSB registers. This function
 * shifts right by 2 bits to obtain 14-bit signed values.
 *
 * @param dev Pointer to I2C device descriptor.
 * @param x Pointer to store X-axis raw value.
 * @param y Pointer to store Y-axis raw value.
 * @param z Pointer to store Z-axis raw value.
 * @return 0 on success, negative errno code on failure.
 */
int accel_read_xyz(const struct i2c_dt_spec *dev, int16_t *x, int16_t *y, int16_t *z) {
    uint8_t buf[6];
    int ret = i2c_read_regs(dev, ACCEL_REG_OUT_X_MSB, buf, 6);
    if (ret < 0) return ret;

    *x = (int16_t)(((int16_t)((buf[0] << 8) | buf[1])) >> 2);
    *y = (int16_t)(((int16_t)((buf[2] << 8) | buf[3])) >> 2);
    *z = (int16_t)(((int16_t)((buf[4] << 8) | buf[5])) >> 2);

    return 0;
}

/**
 * @brief Convert raw accelerometer value to g units.
 *
 * @param raw Raw 14-bit accelerometer reading.
 * @param range Measurement range used during initialization.
 * @param g_value Pointer to store converted value in g.
 */
void accel_convert_to_g(int16_t raw, uint8_t range, float *g_value) {
    float sensitivity = (range == ACCEL_2G) ? 4096.0f :
                        (range == ACCEL_4G) ? 2048.0f : 1024.0f;
    *g_value = (float)raw / sensitivity;
}

/**
 * @brief Convert raw accelerometer value to meters per second squared (m/s²).
 *
 * Uses standard gravity (9.80665 m/s²) for conversion.
 *
 * @param raw Raw 14-bit accelerometer reading.
 * @param range Measurement range used during initialization.
 * @param ms2_value Pointer to store converted value in m/s².
 */
void accel_convert_to_ms2(int16_t raw, uint8_t range, float *ms2_value) {
    float sensitivity = (range == ACCEL_2G) ? 4096.0f :
                        (range == ACCEL_4G) ? 2048.0f : 1024.0f;
    float g_value = (float)raw / sensitivity;
    *ms2_value = g_value * 9.80665f;
}
