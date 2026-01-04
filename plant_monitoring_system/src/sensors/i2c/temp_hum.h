/**
 * @file temp_hum.h
 * @brief Interface for the Si7021 temperature and humidity sensor using Zephyr I2C API.
 *
 * Provides functions to initialize the sensor and read temperature (°C) and
 * relative humidity (%RH) via I2C.
 */

#ifndef TEMP_HUM_H
#define TEMP_HUM_H

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <stdint.h>

/* === Si7021 I2C Configuration === */
#define TH_I2C_ADDR          0x40  /**< Default I2C address of Si7021 */

/* === Si7021 command set === */
#define TH_WRITE_USER_REG      0xE6  /**< Write User Register 1 */
#define TH_READ_USER_REG       0xE7  /**< Read User Register 1 */
#define TH_MEAS_RH_HOLD        0xE5  /**< Measure Relative Humidity, Hold Master mode */
#define TH_MEAS_TEMP_HOLD      0xE3  /**< Measure Temperature, Hold Master mode */
#define TH_READ_TEMP_FROM_RH   0xE0  /**< Read Temperature from previous RH measurement */
#define TH_RESET               0xFE  /**< Soft reset command */

#define TH_RES_RH12_TEMP14   0x00  /**< RH:12-bit, Temp:14-bit */
#define TH_RES_RH8_TEMP12    0x01  /**< RH:8-bit,  Temp:12-bit */
#define TH_RES_RH10_TEMP13   0x80  /**< RH:10-bit, Temp:13-bit */
#define TH_RES_RH11_TEMP11   0x81  /**< RH:11-bit, Temp:11-bit */

/* === Function Prototypes === */

/**
 * @brief Initialize the Si7021 temperature and humidity sensor.
 *
 * Checks if the I2C device is ready and performs a soft reset.
 *
 * @param dev Pointer to a valid I2C device descriptor.
 * @param resolution Resolution setting for temperature and humidity measurements.
 * @return 0 on success, negative errno code on failure.
 */
int temp_hum_init(const struct i2c_dt_spec *dev, uint8_t resolution);

/**
 * @brief Read temperature in degrees Celsius from the sensor.
 *
 * Performs a measurement and converts the raw data to °C.
 *
 * @param dev Pointer to a valid I2C device descriptor.
 * @param temperature Pointer to store the temperature in °C.
 * @return 0 on success, negative errno code on failure.
 */
int temp_hum_read_temperature(const struct i2c_dt_spec *dev, float *temperature);

/**
 * @brief Read relative humidity in percent from the sensor.
 *
 * Performs a measurement and converts the raw data to %RH.
 *
 * @param dev Pointer to a valid I2C device descriptor.
 * @param humidity Pointer to store the relative humidity in %RH.
 * @return 0 on success, negative errno code on failure.
 */
int temp_hum_read_humidity(const struct i2c_dt_spec *dev, float *humidity);

#endif /* TEMP_HUM_H */
