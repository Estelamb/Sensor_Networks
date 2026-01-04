/**
 * @file color.c
 * @brief Implementation for the TCS34725 RGB color sensor driver using Zephyr I2C API.
 *
 * Provides initialization, configuration, and reading of raw RGB and clear
 * channel data from the sensor.
 */

#include "color.h"
#include "i2c.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

/* === Internal helper functions === */

/**
 * @brief Write a single byte to a sensor register.
 *
 * @param dev Pointer to I2C device descriptor.
 * @param reg Register address to write.
 * @param val Value to write.
 * @return 0 on success, negative errno on failure.
 */
static int color_write_reg(const struct i2c_dt_spec *dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { COLOR_COMMAND | reg, val };
    return i2c_write_dt(dev, buf, sizeof(buf));
}

/**
 * @brief Read multiple bytes from sensor registers.
 *
 * Uses auto-increment to read sequential registers.
 *
 * @param dev Pointer to I2C device descriptor.
 * @param reg Starting register address.
 * @param buf Buffer to store read bytes.
 * @param len Number of bytes to read.
 * @return 0 on success, negative errno on failure.
 */
static int color_read_regs(const struct i2c_dt_spec *dev, uint8_t reg, uint8_t *buf, size_t len)
{
    uint8_t reg_cmd = COLOR_COMMAND | AUTO_INCREMENT | reg;
    return i2c_write_read_dt(dev, &reg_cmd, 1, buf, len);
}

/* === Public API === */

/**
 * @brief Initialize the TCS34725 sensor.
 *
 * Checks I2C bus readiness, powers on the sensor, and applies default
 * integration time and gain settings.
 *
 * @param dev Pointer to I2C device descriptor.
 * @return 0 on success, negative errno code on failure.
 */
int color_init(const struct i2c_dt_spec *dev, uint8_t gain, uint8_t atime)
{
    printk("[COLOR] - Initializing Color sensor...\n");

    if (!device_is_ready(dev->bus)) {
        printk("[COLOR] - I2C bus not ready\n");
        return -ENODEV;
    }

    /* Power on and enable ADC */
    if (color_wake_up(dev) < 0) {
        printk("[COLOR] - Failed to wake up sensor\n");
        return -EIO;
    }

    /* Apply default settings */
    color_write_reg(dev, COLOR_CONTROL, gain);
    color_write_reg(dev, COLOR_ATIME, atime);

    printk("[COLOR] - Color sensor initialized successfully\n");
    return 0;
}

/**
 * @brief Wake up the sensor (power on and enable ADC).
 *
 * @param dev Pointer to I2C device descriptor.
 * @return 0 on success, negative errno code on failure.
 */
int color_wake_up(const struct i2c_dt_spec *dev)
{
    int ret = color_write_reg(dev, COLOR_ENABLE, ENABLE_PON);
    if (ret < 0) return ret;

    k_msleep(3); /* Wait for power-on */

    ret = color_write_reg(dev, COLOR_ENABLE, ENABLE_PON | ENABLE_AEN);
    if (ret < 0) return ret;

    k_msleep(3);
    return 0;
}

/**
 * @brief Put the sensor into sleep mode (disable ADC and power off).
 *
 * @param dev Pointer to I2C device descriptor.
 * @return 0 on success, negative errno code on failure.
 */
int color_sleep(const struct i2c_dt_spec *dev)
{
    return color_write_reg(dev, COLOR_ENABLE, 0x00);
}

/**
 * @brief Read raw RGB and clear channel values from the sensor.
 *
 * @param dev Pointer to I2C device descriptor.
 * @param data Pointer to ColorSensorData struct to store values.
 * @return 0 on success, negative errno code on failure.
 */
int color_read_rgb(const struct i2c_dt_spec *dev, ColorSensorData *data)
{
    uint8_t buf[8];
    int ret = color_read_regs(dev, COLOR_CLEAR_L, buf, sizeof(buf));
    if (ret < 0) {
        printk("[COLOR SENSOR] - Failed to read RGB data (%d)\n", ret);
        return ret;
    }

    /* Combine low and high bytes */
    uint16_t clear = (buf[1] << 8) | buf[0];
    uint16_t red   = (buf[3] << 8) | buf[2];
    uint16_t green = (buf[5] << 8) | buf[4];
    uint16_t blue  = (buf[7] << 8) | buf[6];

    /* Avoid divide-by-zero */
    if (clear == 0) clear = 1;

    /* Store raw values */
    data->clear = clear;
    data->red   = red;
    data->green = green;
    data->blue  = blue;

    return 0;
}
