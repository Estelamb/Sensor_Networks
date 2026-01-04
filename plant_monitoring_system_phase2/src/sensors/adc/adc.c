/**
 * @file adc.c
 * @brief ADC driver implementation for multi-channel analog input using Zephyr.
 *
 * This module implements ADC initialization and data acquisition routines
 * for dynamic, per-channel configuration. It supports reading raw digital
 * values, normalized floating-point samples, and voltage readings in millivolts.
 */

#include "adc.h"
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>

/** @brief Static buffer used for single-sample ADC conversions. */
static int16_t sample_buffer[BUFFER_SIZE];

/**
 * @brief Initializes the specified ADC device.
 *
 * Verifies that the ADC device referenced in the configuration is ready
 * for operation. This function does not configure ADC channels; they are
 * set up dynamically during each read operation.
 *
 * @param cfg Pointer to the ADC configuration structure.
 * @retval 0 If the ADC device is ready.
 * @retval -ENODEV If the device is not available or not ready.
 */
int adc_init(const struct adc_config *cfg)
{
    printk("[ADC] - Initializing ADC device %s...\n", cfg->dev->name);

    if (!device_is_ready(cfg->dev)) {
        printk("[ADC] - ADC device %s is not ready\n", cfg->dev->name);
        return -ENODEV;
    }

    printk("[ADC] - ADC device %s initialized successfully\n", cfg->dev->name);
    return 0;
}

/**
 * @brief Reads a raw ADC sample from the configured channel.
 *
 * Dynamically sets up the specified ADC channel using the provided
 * configuration parameters, performs a single conversion, and returns
 * the unprocessed raw sample value.
 *
 * @param cfg Pointer to the ADC configuration structure.
 * @param raw_val Pointer to store the raw ADC sample.
 * @retval 0 If the conversion was successful.
 * @retval -EIO If the ADC read operation failed.
 * @retval Negative error code from @ref adc_channel_setup on setup failure.
 */
int adc_read_raw(const struct adc_config *cfg, int16_t *raw_val)
{
    struct adc_channel_cfg channel_cfg = {
        .gain             = cfg->gain,
        .reference        = cfg->ref,
        .acquisition_time = cfg->acquisition_time,
        .channel_id       = cfg->channel_id,
    };

    int ret = adc_channel_setup(cfg->dev, &channel_cfg);
    if (ret < 0) {
        printk("ADC channel setup failed (%d)\n", ret);
        return ret;
    }

    struct adc_sequence sequence = {
        .channels    = BIT(cfg->channel_id),
        .buffer      = sample_buffer,
        .buffer_size = sizeof(sample_buffer),
        .resolution  = cfg->resolution,
    };

    ret = adc_read(cfg->dev, &sequence);
    if (ret < 0) {
        printk("ADC read failed (%d)\n", ret);
        return ret;
    }

    *raw_val = sample_buffer[0];
    return 0;
}

/**
 * @brief Reads and normalizes an ADC sample (0.0–1.0 range).
 *
 * Performs a raw ADC conversion and converts the result into a normalized
 * floating-point value based on the configured resolution.
 *
 * @param cfg Pointer to the ADC configuration structure.
 * @return Normalized ADC value in the range [0.0, 1.0], or a negative
 *         error code if the conversion failed.
 */
float adc_read_normalized(const struct adc_config *cfg)
{
    int16_t raw_val = 0;
    int ret = adc_read_raw(cfg, &raw_val);
    if (ret < 0) {
        return ret;
    }

    return (float)raw_val / ((1 << cfg->resolution) - 1);
}

/**
 * @brief Reads the ADC value and converts it to millivolts.
 *
 * Performs a raw ADC conversion and scales the digital reading
 * according to the configured reference voltage and resolution.
 *
 * @param cfg Pointer to the ADC configuration structure.
 * @param out_mv Pointer to store the resulting voltage in millivolts.
 * @retval 0 If the voltage computation was successful.
 * @retval -EIO If the ADC read failed.
 * @retval Negative error code from @ref adc_read_raw on failure.
 */
int adc_read_voltage(const struct adc_config *cfg, int32_t *out_mv)
{
    int16_t raw_val = 0;
    int ret = adc_read_raw(cfg, &raw_val);
    if (ret < 0) {
        return ret;
    }

    *out_mv = ((int32_t)raw_val * cfg->vref_mv) / ((1 << cfg->resolution) - 1);
    return 0;
}
