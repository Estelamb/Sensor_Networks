/**
 * @file color.h
 * @brief Interface for the TCS34725 RGB color sensor using Zephyr I2C API.
 *
 * This module provides initialization, configuration, and reading functions
 * for the TCS34725 color sensor. Supports gain, integration time, and
 * reading raw RGB and clear channel values.
 */

#ifndef COLOR_H
#define COLOR_H

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <stdint.h>

/* === TCS34725 I2C Configuration === */
#define COLOR_I2C_ADDR    0x29  /**< I2C device address for TCS34725 */
#define COLOR_COMMAND     0x80  /**< Command bit to access registers */
#define AUTO_INCREMENT    0x20  /**< Auto-increment register addresses */

/* === Register Addresses === */
#define COLOR_ENABLE      0x00  /**< Enable register */
#define COLOR_ATIME       0x01  /**< Integration time register */
#define COLOR_CONTROL     0x0F  /**< Gain control register */
#define COLOR_CLEAR_L     0x14  /**< Clear channel low byte */
#define COLOR_RED_L       0x16  /**< Red channel low byte */
#define COLOR_GREEN_L     0x18  /**< Green channel low byte */
#define COLOR_BLUE_L      0x1A  /**< Blue channel low byte */

/* === ENABLE register bits === */
#define ENABLE_PON        0x01  /**< Power ON */
#define ENABLE_AEN        0x02  /**< ADC Enable */

/* === Gain settings (CONTROL register) === */
#define GAIN_1X           0x00  /**< 1x gain */
#define GAIN_4X           0x01  /**< 4x gain */
#define GAIN_16X          0x02  /**< 16x gain */
#define GAIN_60X          0x03  /**< 60x gain */

/* === Integration time settings (ATIME register) === */
#define INTEGRATION_2_4MS   0xFF /**< 2.4 ms */
#define INTEGRATION_24MS    0xF6 /**< 24 ms */
#define INTEGRATION_101MS   0xD5 /**< 101 ms */
#define INTEGRATION_154MS   0xC0 /**< 154 ms */
#define INTEGRATION_700MS   0x00 /**< 700 ms */

/**
 * @brief Structure for storing raw color sensor data.
 */
typedef struct {
    uint16_t clear;  /**< Clear channel value */
    uint16_t red;    /**< Red channel value */
    uint16_t green;  /**< Green channel value */
    uint16_t blue;   /**< Blue channel value */
} ColorSensorData;

/* === Function Prototypes === */

/**
 * @brief Initialize the color sensor (I2C check, default settings).
 *
 * @param dev Pointer to the I2C device descriptor.
 * @return 0 on success, negative errno code on failure.
 */
int color_init(const struct i2c_dt_spec *dev, uint8_t gain, uint8_t atime);

/**
 * @brief Wake up the color sensor (enable ADC and power).
 *
 * @param dev Pointer to the I2C device descriptor.
 * @return 0 on success, negative errno code on failure.
 */
int color_wake_up(const struct i2c_dt_spec *dev);

/**
 * @brief Put the color sensor into sleep mode (disable ADC).
 *
 * @param dev Pointer to the I2C device descriptor.
 * @return 0 on success, negative errno code on failure.
 */
int color_sleep(const struct i2c_dt_spec *dev);

/**
 * @brief Read raw RGB and clear channel data from the sensor.
 *
 * @param dev Pointer to the I2C device descriptor.
 * @param data Pointer to store the raw color values.
 * @return 0 on success, negative errno code on failure.
 */
int color_read_rgb(const struct i2c_dt_spec *dev, ColorSensorData *data);

#endif /* COLOR_H */
