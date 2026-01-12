/**
 * @file sensors_thread.h
 * @brief Interface for the sensors measurement thread.
 *
 * This header declares the interface for a Zephyr thread responsible for
 * acquiring environmental data from analog and I²C sensors:
 *  - Ambient brightness (phototransistor, ADC)
 *  - Soil moisture (ADC)
 *  - Accelerometer (I²C)
 *  - Temperature & humidity (I²C)
 *  - Color sensor (I²C)
 */

#ifndef SENSORS_THREAD_H
#define SENSORS_THREAD_H

#include "main.h"

/**
 * @brief Start the sensors measurement thread.
 *
 * Initializes synchronization primitives (timer, semaphores) and launches a
 * dedicated Zephyr thread that periodically samples sensors and stores the
 * results into the provided @ref system_measurement structure.
 *
 * The function does not block; the thread runs asynchronously.
 *
 * @param ctx Pointer to a valid @ref system_context containing configuration and semaphores.
 * @param measure Pointer to a valid @ref system_measurement where sensor values will be stored.
 */
void start_sensors_thread(struct system_context *ctx, struct system_measurement *measure);

#endif /* SENSORS_THREAD_H */
