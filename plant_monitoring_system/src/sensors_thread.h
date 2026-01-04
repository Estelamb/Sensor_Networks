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
 *
 * The thread is active in @ref TEST_MODE and @ref NORMAL_MODE and performs
 * periodic sampling. In other modes (e.g., @ref ADVANCED_MODE) the thread
 * suspends sampling and waits to be re-triggered.
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
 * The sampling cadence depends on the system mode:
 *  - @ref TEST_MODE: faster cadence for rapid feedback.
 *  - @ref NORMAL_MODE: regular cadence for ongoing monitoring.
 *  - Other modes: sampling is paused until reactivated.
 *
 * The function does not block; the thread runs asynchronously.
 *
 * @param ctx Pointer to a valid @ref system_context containing configuration and semaphores.
 * @param measure Pointer to a valid @ref system_measurement where sensor values will be stored.
 */
void start_sensors_thread(struct system_context *ctx, struct system_measurement *measure);

#endif /* SENSORS_THREAD_H */
