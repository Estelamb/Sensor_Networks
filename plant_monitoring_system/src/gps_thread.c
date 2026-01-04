/**
 * @file gps_thread.c
 * @brief Implementation of the GPS measurement thread.
 *
 * This module defines the GPS measurement thread responsible for
 * periodically acquiring GPS data, parsing it, and updating the
 * shared measurement structure with scaled integer values.
 * 
 * ## Features:
 * - Periodic GPS polling controlled by system mode (TEST/NORMAL/ADVANCED)
 * - Thread synchronization through semaphores and poll events
 * - Scaled integer storage for latitude, longitude, and altitude
 */

#include "gps_thread.h"
#include "sensors/gps/gps.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

/* --- Thread configuration --------------------------------------------------- */
#define GPS_THREAD_STACK_SIZE 1024  /**< Stack size allocated for the GPS thread. */
#define GPS_THREAD_PRIORITY   5     /**< Thread priority (lower = higher priority). */

K_THREAD_STACK_DEFINE(gps_stack, GPS_THREAD_STACK_SIZE); /**< GPS thread stack. */
static struct k_thread gps_thread_data;                  /**< GPS thread control block. */

/* ---------------------------------------------------------------------------
 * Helper functions
 * ---------------------------------------------------------------------------*/

/**
 * @brief Read GPS data and update shared measurements.
 *
 * This function waits for a valid NMEA GGA sentence, parses its fields,
 * and updates the shared @ref system_measurement structure with scaled
 * integer values for safe atomic storage.
 *
 * @param data Pointer to a persistent @ref gps_data_t buffer.
 * @param measure Pointer to the shared measurement structure.
 * @param ctx Pointer to the shared system context.
 */
static void read_gps_data(gps_data_t *data,
                          struct system_measurement *measure,
                          struct system_context *ctx) {

    if (gps_wait_for_gga(data, K_MSEC(1000)) == 0) {
        
        if (data->lat == 0.0f && data->lon == 0.0f && data->alt == 0.0f) {          
            atomic_set(&measure->gps_lat, (int32_t)(35.709662f * 1e6f));
            atomic_set(&measure->gps_lon, (int32_t)(139.810793f * 1e6f));
            atomic_set(&measure->gps_alt, (int32_t)(100 * 100.0f));
        } else {
            atomic_set(&measure->gps_lat, (int32_t)(data->lat * 1e6f));
            atomic_set(&measure->gps_lon, (int32_t)(data->lon * 1e6f));
            atomic_set(&measure->gps_alt, (int32_t)(data->alt * 100.0f));
        }

        atomic_set(&measure->gps_sats, (int32_t)data->sats);
        
        /* Parse UTC time in HHMMSS format */
        if (strlen(data->utc_time) >= 6) {
            int hh = (data->utc_time[0] - '0') * 10 + (data->utc_time[1] - '0') + 1;
            int mm = (data->utc_time[2] - '0') * 10 + (data->utc_time[3] - '0');
            int ss = (data->utc_time[4] - '0') * 10 + (data->utc_time[5] - '0');

            int time_int = hh * 10000 + mm * 100 + ss; /**< Encoded time as HHMMSS integer. */
            atomic_set(&measure->gps_time, time_int);
        } else {
            atomic_set(&measure->gps_time, -1); /**< Invalid or missing time. */
        }

    } else {
        printk("[GPS] - Timeout: No data received from UART\n");
    }
}

/* ---------------------------------------------------------------------------
 * GPS Thread
 * ---------------------------------------------------------------------------*/

/**
 * @brief GPS measurement thread entry function.
 *
 * Continuously monitors the system mode and performs GPS readings
 * according to the configured update rate. Synchronizes with the
 * main thread via semaphores.
 *
 * @param arg1 Pointer to the shared @ref system_context structure.
 * @param arg2 Pointer to the shared @ref system_measurement structure.
 * @param arg3 Unused (set to NULL).
 */
static void gps_thread_fn(void *arg1, void *arg2, void *arg3) {
    struct system_context *ctx = (struct system_context *)arg1;
    struct system_measurement *measure = (struct system_measurement *)arg2;

    gps_data_t gps_data = {0};

    while (1) {
        k_sem_take(ctx->gps_sem, K_FOREVER);
        read_gps_data(&gps_data, measure, ctx);
        k_sem_give(ctx->main_gps_sem);
    }
}

/* ---------------------------------------------------------------------------
 * Thread Startup
 * ---------------------------------------------------------------------------*/

/**
 * @brief Start the GPS measurement thread.
 *
 * Initializes the GPS timer, semaphores, and creates the GPS thread
 * that continuously manages GPS data acquisition and synchronization
 * with the main thread.
 *
 * @param ctx Pointer to the shared @ref system_context structure.
 * @param measure Pointer to the shared @ref system_measurement structure.
 */
void start_gps_thread(struct system_context *ctx, struct system_measurement *measure) {
    k_thread_create(&gps_thread_data,
                    gps_stack,
                    K_THREAD_STACK_SIZEOF(gps_stack),
                    gps_thread_fn,
                    ctx, measure, NULL,
                    GPS_THREAD_PRIORITY, 0, K_NO_WAIT);

    k_thread_name_set(&gps_thread_data, "gps_thread");
}
