/**
 * @file adc.h
 * @brief ADC interface layer for analog sensor sampling using Zephyr drivers.
 *
 * This module provides a hardware abstraction layer for analog-to-digital
 * conversion using Zephyr's ADC API. It defines configuration structures
 * and utility functions for initializing ADC channels and retrieving
 * raw, normalized, or voltage-converted readings.
 *
 * Each sensor using the ADC should have its own configuration instance.
 */

#ifndef ADC_H
#define ADC_H

#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>

/** @brief ADC sample buffer size (number of samples per read). */
#define BUFFER_SIZE 1

/**
 * @brief ADC channel configuration structure.
 *
 * Defines all parameters required to configure and operate an ADC channel
 * for sensor data acquisition. Multiple instances can be used for different
 * sensors sharing the same ADC peripheral.
 */
struct adc_config {
    const struct device *dev;      /**< Pointer to the ADC device instance. */
    uint8_t channel_id;            /**< ADC channel number (input pin). */
    uint8_t resolution;            /**< Conversion resolution in bits. */
    enum adc_gain gain;            /**< Programmable gain amplifier setting. */
    enum adc_reference ref;        /**< Voltage reference source for conversion. */
    uint32_t acquisition_time;     /**< Sampling acquisition time in microseconds. */
    int32_t vref_mv;               /**< Reference voltage in millivolts. */
};

/**
 * @brief Initializes the ADC hardware and specified channel.
 *
 * Configures the ADC driver for the given channel parameters
 * (resolution, gain, reference, and acquisition time).
 *
 * @param cfg Pointer to the ADC configuration structure.
 * @retval 0 If initialization was successful.
 * @retval -EINVAL If configuration parameters are invalid.
 * @retval -EIO If ADC device initialization failed.
 */
int adc_init(const struct adc_config *cfg);

/**
 * @brief Reads a raw ADC value from the configured channel.
 *
 * Performs a single ADC conversion and returns the raw digital value
 * as provided by the hardware (not scaled or converted).
 *
 * @param cfg Pointer to the ADC configuration structure.
 * @param raw_val Pointer to store the raw ADC output value.
 * @retval 0 If the read operation was successful.
 * @retval -EIO If the ADC read failed.
 */
int adc_read_raw(const struct adc_config *cfg, int16_t *raw_val);

/**
 * @brief Reads and normalizes the ADC value.
 *
 * Performs a conversion and returns the result as a normalized floating-point
 * value between 0.0 and 1.0 based on the ADC resolution and reference voltage.
 *
 * @param cfg Pointer to the ADC configuration structure.
 * @return Normalized ADC reading in the range [0.0, 1.0].
 */
float adc_read_normalized(const struct adc_config *cfg);

/**
 * @brief Reads the ADC value and converts it to millivolts.
 *
 * Performs a conversion and scales the result according to the configured
 * reference voltage and resolution to obtain the corresponding voltage value.
 *
 * @param cfg Pointer to the ADC configuration structure.
 * @param out_mv Pointer to store the computed voltage in millivolts.
 * @retval 0 If the voltage read was successful.
 * @retval -EIO If the ADC conversion failed.
 */
int adc_read_voltage(const struct adc_config *cfg, int32_t *out_mv);

#endif // ADC_H
