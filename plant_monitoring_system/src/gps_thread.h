/**
 * @file gps_thread.h
 * @brief Interface for the GPS measurement thread.
 *
 * This header declares the interface for a Zephyr thread responsible for
 * acquiring and processing GPS data such as latitude, longitude, altitude,
 * satellite count, and UTC time.
 *
 * The thread operates periodically in @ref TEST_MODE and @ref NORMAL_MODE.
 * In other modes (e.g., @ref ADVANCED_MODE), GPS sampling is suspended until
 * manually triggered.
 */

#ifndef GPS_THREAD_H
#define GPS_THREAD_H

#include "main.h"

/**
 * @brief Start the GPS measurement thread.
 *
 * Initializes synchronization primitives (timer and semaphores) and creates
 * a dedicated Zephyr thread that periodically reads and parses GPS data from
 * the configured GPS module.
 *
 * The sampling cadence depends on the active system mode:
 *  - @ref TEST_MODE: fast measurement interval for debugging and validation.
 *  - @ref NORMAL_MODE: regular interval for periodic operation.
 *  - @ref ADVANCED_MODE: measurement suspended until reactivated.
 *
 * The thread stores scaled integer values in @ref system_measurement:
 *  - Latitude / Longitude: degrees × 1e6
 *  - Altitude: meters × 100
 *  - Time (UTC): integer in HHMMSS format
 *
 * @param ctx Pointer to a valid @ref system_context structure with configuration and semaphores.
 * @param measure Pointer to a valid @ref system_measurement structure to store GPS readings.
 */
void start_gps_thread(struct system_context *ctx, struct system_measurement *measure);

#endif /* GPS_THREAD_H */
