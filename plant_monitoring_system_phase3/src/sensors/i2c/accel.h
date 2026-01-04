/**
 * @file accel.h
 * @brief I2C interface for a 3-axis accelerometer.
 *
 * This module provides initialization, raw data reading, and conversion
 * functions for a 3-axis accelerometer over I2C. Supports setting
 * measurement range and converting raw data to g or m/s².
 */

#ifndef ACCEL_H
#define ACCEL_H

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <stdint.h>

/* Device I2C address and identification */
#define ACCEL_I2C_ADDR          0x1D        /**< I2C address of the accelerometer */
#define ACCEL_REG_WHO_AM_I      0x0D        /**< WHO_AM_I register */
#define ACCEL_WHO_AM_I_VALUE    0x1A        /**< Expected WHO_AM_I value */

/* Power control registers */
#define ACCEL_REG_CTRL1         0x2A        /**< Control register 1 */
#define ACCEL_REG_CTRL2         0x2B        /**< Control register 2 */

/* Measurement range selection */
#define ACCEL_REG_XYZ_DATA_CFG  0x0E        /**< XYZ range configuration register */
#define ACCEL_2G                0x00        /**< ±2g range */
#define ACCEL_4G                0x01        /**< ±4g range */
#define ACCEL_8G                0x02        /**< ±8g range */

/* Output data registers */
#define ACCEL_REG_OUT_X_MSB     0x01        /**< X-axis MSB */
#define ACCEL_REG_OUT_X_LSB     0x02        /**< X-axis LSB */
#define ACCEL_REG_OUT_Y_MSB     0x03        /**< Y-axis MSB */
#define ACCEL_REG_OUT_Y_LSB     0x04        /**< Y-axis LSB */
#define ACCEL_REG_OUT_Z_MSB     0x05        /**< Z-axis MSB */
#define ACCEL_REG_OUT_Z_LSB     0x06        /**< Z-axis LSB */

/**
 * @brief Initialize the accelerometer device.
 *
 * Sets up the device over I2C and configures the measurement range.
 * Checks the WHO_AM_I register to verify device identity.
 *
 * @param dev Pointer to the I2C device descriptor.
 * @param range Measurement range: ACCEL_2G, ACCEL_4G, or ACCEL_8G.
 * @return 0 on success, negative errno code on failure.
 */
int accel_init(const struct i2c_dt_spec *dev, uint8_t range);

/**
 * @brief Read raw accelerometer values for X, Y, and Z axes.
 *
 * Reads the 16-bit signed values from the sensor output registers.
 *
 * @param dev Pointer to the I2C device descriptor.
 * @param x Pointer to store X-axis raw value.
 * @param y Pointer to store Y-axis raw value.
 * @param z Pointer to store Z-axis raw value.
 * @return 0 on success, negative errno code on failure.
 */
int accel_read_xyz(const struct i2c_dt_spec *dev, int16_t *x, int16_t *y, int16_t *z);

/**
 * @brief Convert raw accelerometer value to g units.
 *
 * @param raw Raw 16-bit accelerometer reading.
 * @param range Measurement range used during initialization.
 * @param g_value Pointer to store converted value in g.
 */
void accel_convert_to_g(int16_t raw, uint8_t range, float *g_value);

/**
 * @brief Convert raw accelerometer value to meters per second squared (m/s²).
 *
 * Uses Earth's gravity (9.80665 m/s²) for conversion.
 *
 * @param raw Raw 16-bit accelerometer reading.
 * @param range Measurement range used during initialization.
 * @param ms2_value Pointer to store converted value in m/s².
 */
void accel_convert_to_ms2(int16_t raw, uint8_t range, float *ms2_value);

#endif // ACCEL_H
