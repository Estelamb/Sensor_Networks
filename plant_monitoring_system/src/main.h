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
#include "sensors/i2c/accel.h"
#include "sensors/i2c/temp_hum.h"
#include "sensors/i2c/color.h"
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
    struct adc_config *soil_moisture;   /**< Soil moisture ADC configuration. */

    struct i2c_dt_spec *accelerometer;  /**< Accelerometer I2C device specification. */
    uint8_t accel_range;                /**< Accelerometer full-scale range (e.g., 2G, 4G, 8G). */

    struct i2c_dt_spec *temp_hum;       /**< Temperature and humidity sensor I2C specification. */
    struct i2c_dt_spec *color;          /**< Color sensor I2C device specification. */
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
    atomic_t brightness;  /**< Latest ambient brightness (0–100%). */
    atomic_t moisture;    /**< Latest soil moisture (0–100%). */

    atomic_t accel_x;   /**< Latest X-axis acceleration. */
    atomic_t accel_y;   /**< Latest Y-axis acceleration. */
    atomic_t accel_z;   /**< Latest Z-axis acceleration. */

    atomic_t temp;        /**< Latest temperature (°C). */
    atomic_t hum;         /**< Latest relative humidity (%RH). */

    atomic_t red;         /**< Latest red color value (raw). */
    atomic_t green;       /**< Latest green color value (raw). */
    atomic_t blue;        /**< Latest blue color value (raw). */
    atomic_t clear;       /**< Latest clear color channel value (raw). */

    atomic_t gps_lat;     /**< Latest GPS latitude (degrees). */
    atomic_t gps_lon;     /**< Latest GPS longitude (degrees). */
    atomic_t gps_alt;     /**< Latest GPS altitude (meters). */
    atomic_t gps_sats;    /**< Latest number of satellites in view. */
    atomic_t gps_time;    /**< Latest GPS timestamp (float or encoded). */
};

#endif /* MAIN_H */
