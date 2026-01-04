/**
 * @file main.h
 * @brief Shared definitions for the Plant Monitoring System.
 *
 * This header defines shared data structures and enumerations
 * used across the main application, sensors thread, and GPS thread.
 * It provides a unified context for system configuration and
 * sensor measurements.
 */

#ifndef MAIN_H
#define MAIN_H

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include "sensors/adc/adc.h"
#include "sensors/i2c/i2c.h"
#include "sensors/i2c/temp_hum.h"
#include "sensors/gps/gps.h"
#include "sensors/led/rgb_led.h"

/**
 * @struct system_context
 * @brief Shared system context between main, sensors, and GPS threads.
 *
 * This structure contains pointers to configuration objects, semaphores,
 * and shared state used to coordinate between the main, sensors, and GPS threads.
 */
struct system_context {
    struct adc_config *phototransistor; /**< Phototransistor ADC configuration. */
    struct i2c_dt_spec *temp_hum;       /**< Temperature and humidity sensor I2C specification. */
    struct gps_config *gps;             /**< GPS module configuration. */

    struct k_sem *main_sensors_sem;     /**< Semaphore for main-to-sensors synchronization. */
    struct k_sem *main_gps_sem;         /**< Semaphore for main-to-GPS synchronization. */
    struct k_sem *sensors_sem;          /**< Semaphore to trigger sensor measurement. */
    struct k_sem *gps_sem;              /**< Semaphore to trigger GPS measurement. */
};

/**
 * @struct system_measurement
 * @brief Shared sensor data between main, sensors, and GPS threads.
 *
 * Contains the most recent measurements for all sensors, stored
 * in atomic variables for thread-safe access.
 */
struct system_measurement {
    atomic_t brightness;    /**< Latest brightness (0–100%). */

    atomic_t temp;        /**< Latest temperature (°C). */
    atomic_t hum;         /**< Latest relative humidity (%RH). */

    atomic_t gps_lat;     /**< Latest GPS latitude (degrees). */
    atomic_t gps_lon;     /**< Latest GPS longitude (degrees). */
    atomic_t gps_alt;     /**< Latest GPS altitude (meters). */
};

#endif /* MAIN_H */
