/**
 * @file temp_hum.c
 * @brief Implementation of Si7021 temperature and humidity sensor driver for Zephyr.
 *
 * This module provides initialization, relative humidity (%RH), and temperature (°C)
 * reading functions via I2C using the Zephyr I2C API.
 */

#include "temp_hum.h"
#include "i2c.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <math.h>

/**
 * @brief Write a single command to the Si7021 sensor.
 *
 * @param dev Pointer to a valid I2C device descriptor.
 * @param cmd Command byte to send.
 * @return 0 on success, negative errno code on failure.
 */
static int temp_hum_write_cmd(const struct i2c_dt_spec *dev, uint8_t cmd)
{
    return i2c_write_dt(dev, &cmd, 1);
}

/**
 * @brief Read data from temp_hum after sending a command.
 *
 * @param dev Pointer to a valid I2C device descriptor.
 * @param cmd Command byte to initiate the read.
 * @param buf Pointer to buffer to store received bytes.
 * @param len Number of bytes to read.
 * @return 0 on success, negative errno code on failure.
 */
static int temp_hum_read_data(const struct i2c_dt_spec *dev, uint8_t cmd, uint8_t *buf, size_t len)
{
    return i2c_write_read_dt(dev, &cmd, 1, buf, len);
}

/**
 * @brief Initialize the temp_hum temperature and humidity sensor.
 *
 * Performs a soft reset and verifies that the I2C bus is ready.
 *
 * @param dev Pointer to a valid I2C device descriptor.
 * @param resolution Resolution setting for temperature and humidity measurements.
 * @return 0 on success, negative errno code on failure.
 */
int temp_hum_init(const struct i2c_dt_spec *dev, uint8_t resolution)
{
    printk("[TEMP_HUM] - Initializing Temp and Hum sensor...\n");

    if (!device_is_ready(dev->bus)) {
        printk("[TEMP_HUM] - I2C bus not ready\n");
        return -ENODEV;
    }

    // Reset sensor
    int ret = temp_hum_write_cmd(dev, TH_RESET);
    if (ret < 0) {
        printk("[TEMP_HUM] - Reset failed (%d)\n", ret);
        return ret;
    }

    k_msleep(50);

    // Write resolution to user register
    uint8_t write_buf[2] = { TH_WRITE_USER_REG, resolution };
    ret = i2c_write_dt(dev, write_buf, sizeof(write_buf)); 
    if (ret < 0) {
        printk("[TEMP_HUM] - Failed to write user register (%d)\n", ret);
        return ret;
    }

    printk("[TEMP_HUM] - Resolution set successfully (0x%02X)\n", resolution);
    printk("[TEMP_HUM] - Initialization complete\n");
    return 0;
}

/**
 * @brief Read relative humidity from the temp_hum sensor.
 *
 * Uses the Hold Master mode to measure humidity and converts the raw value
 * to %RH using the formula from the datasheet.
 *
 * @param dev Pointer to a valid I2C device descriptor.
 * @param humidity Pointer to float variable to store relative humidity (%RH).
 * @return 0 on success, negative errno code on failure.
 */
int temp_hum_read_humidity(const struct i2c_dt_spec *dev, float *humidity)
{
    uint8_t buf[2];
    int ret = temp_hum_read_data(dev, TH_MEAS_RH_HOLD, buf, sizeof(buf));
    if (ret < 0) {
        printk("[TEMP_HUM] - Failed to read humidity (%d)\n", ret);
        return ret;
    }

    uint16_t raw_rh = ((uint16_t)buf[0] << 8) | buf[1];
    float rh = ((125.0f * raw_rh) / 65536.0f) - 6.0f;

    if (rh < 0.0f) rh = 0.0f;
    if (rh > 100.0f) rh = 100.0f;

    *humidity = rh;
    return 0;
}

/**
 * @brief Read temperature from the temp_hum sensor in degrees Celsius.
 *
 * Uses the Hold Master mode to measure temperature and converts the raw
 * value to °C using the formula from the datasheet.
 *
 * @param dev Pointer to a valid I2C device descriptor.
 * @param temperature Pointer to float variable to store temperature (°C).
 * @return 0 on success, negative errno code on failure.
 */
int temp_hum_read_temperature(const struct i2c_dt_spec *dev, float *temperature)
{
    uint8_t buf[2];
    int ret = temp_hum_read_data(dev, TH_MEAS_TEMP_HOLD, buf, sizeof(buf));
    if (ret < 0) {
        printk("[TEMP_HUM] - Failed to read temperature (%d)\n", ret);
        return ret;
    }

    uint16_t raw_temp = ((uint16_t)buf[0] << 8) | buf[1];
    float temp = ((175.72f * raw_temp) / 65536.0f) - 46.85f;

    *temperature = temp;
    return 0;
}
