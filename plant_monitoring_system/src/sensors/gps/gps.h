/**
 * @file gps.h
 * @brief GPS interface for UART-based NMEA parsing (GGA sentence support).
 *
 * This module provides a simple GPS helper for parsing NMEA GGA sentences
 * received through a UART interface. It includes initialization, interrupt
 * setup, and a blocking wait API to obtain the most recent parsed position.
 *
 * Functions:
 *  - @ref gps_init() to initialize the UART and enable ISR-based reception.
 *  - @ref gps_wait_for_gga() to wait for a parsed GGA sentence.
 *
 * Parsed data is returned as floating-point values in a @ref gps_data_t structure.
 */

#ifndef GPS_H_
#define GPS_H_

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief GPS configuration structure.
 *
 * Holds the device reference used for GPS communication.
 * The UART device must be resolved and provided by the caller.
 */
struct gps_config {
    const struct device *dev; /**< UART device instance used by the GPS module. */
};

/**
 * @brief Parsed GPS data from a GGA sentence.
 *
 * Contains geographic and fix-related data parsed from an NMEA GGA message.
 * All numeric fields are expressed as floats for simplicity.
 */
typedef struct {
    float lat;           /**< Latitude in decimal degrees. */
    float lon;           /**< Longitude in decimal degrees. */
    float alt;           /**< Altitude in meters above mean sea level. */
    int   sats;          /**< Number of satellites currently in use. */
    float hdop;          /**< Horizontal dilution of precision. */
    char  utc_time[16];  /**< UTC time (hhmmss.ss), null-terminated if available. */
} gps_data_t;

/**
 * @brief Initializes the GPS module UART and interrupt service routine.
 *
 * Configures the UART device for receiving NMEA data from the GPS module
 * and sets up an interrupt-based handler to process incoming bytes.
 *
 * @param cfg Pointer to the GPS configuration structure.
 * @retval 0 If initialization was successful.
 * @retval Negative error code if UART configuration or ISR setup failed.
 */
int gps_init(const struct gps_config *cfg);

/**
 * @brief Waits for the next parsed GGA sentence.
 *
 * Blocks until a new, valid GGA sentence has been parsed or until
 * the specified timeout expires. On success, the parsed data is
 * written into the provided @ref gps_data_t structure.
 *
 * @param out Pointer to store the parsed GPS data.
 * @param timeout Timeout duration (e.g. @c K_FOREVER, @c K_MSEC(2000), @c K_NO_WAIT).
 * @retval 0 If a valid GGA sentence was received and parsed successfully.
 * @retval -ETIMEDOUT If no valid GGA sentence was received before timeout.
 *
 * @note This function is typically used after calling @ref gps_init().
 */
int gps_wait_for_gga(gps_data_t *out, k_timeout_t timeout);

#endif /* GPS_H_ */
