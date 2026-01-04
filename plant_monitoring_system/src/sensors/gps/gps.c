/**
 * @file gps.c
 * @brief GPS UART interrupt handler and NMEA GGA parser implementation.
 *
 * This module handles UART-based reception of NMEA sentences from a GPS module.
 * The UART interrupt service routine accumulates incoming data lines, detects
 * complete GGA (or GNGGA) sentences, parses them, and updates the shared
 * GPS data structure. Once new data is available, a semaphore is released
 * to notify waiting threads.
 *
 * The design prioritizes simplicity and robustness for embedded systems.
 */

#include "gps.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <stdlib.h>

#define BUF_SIZE 128      /**< Maximum NMEA sentence length. */
#define MAX_FIELDS 16     /**< Maximum number of comma-separated fields per sentence. */

static const struct device *uart_dev = NULL;
static char nmea_line[BUF_SIZE];
static uint8_t line_pos = 0;

/** @brief Internal storage for parsed GPS data. */
static gps_data_t parsed_data;
/** @brief Semaphore signaling when a valid GGA frame is available. */
static struct k_sem parsed_sem;

/**
 * @brief Converts an NMEA latitude/longitude string to decimal degrees.
 *
 * Converts a coordinate in NMEA format ("DDMM.MMMM" or "DDDMM.MMMM")
 * to standard decimal degrees, applying hemisphere correction based on
 * the direction character.
 *
 * @param nmea Pointer to the NMEA coordinate string.
 * @param dir Direction character ('N', 'S', 'E', or 'W').
 * @return Coordinate in decimal degrees. Returns 0.0f if the input is invalid.
 */
static float nmea_to_degrees(const char *nmea, char dir)
{
    if (!nmea || strlen(nmea) < 4) return 0.0f;

    float value = 0.0f;
    float decimal = 0.0f;
    bool seen_dot = false;
    float divisor = 10.0f;

    for (int i = 0; nmea[i]; i++) {
        char c = nmea[i];
        if (c >= '0' && c <= '9') {
            if (!seen_dot) {
                value = value * 10.0f + (c - '0');
            } else {
                decimal += (c - '0') / divisor;
                divisor *= 10.0f;
            }
        } else if (c == '.') {
            seen_dot = true;
        } else {
            break;
        }
    }

    value += decimal;

    //int deg_len = (dir == 'N' || dir == 'S') ? 2 : 3; /* Latitude: 2 digits; longitude: 3 */
    int degrees = (int)(value / 100.0f);
    float minutes = value - (degrees * 100.0f);
    float result = degrees + (minutes / 60.0f);

    if (dir == 'S' || dir == 'W') result = -result;
    return result;
}

/**
 * @brief Parses a single NMEA GGA sentence and extracts relevant fields.
 *
 * Extracts latitude, longitude, altitude, HDOP, number of satellites,
 * and UTC time from a GGA sentence. Populates a @ref gps_data_t structure
 * with the parsed values.
 *
 * @param line Pointer to the null-terminated GGA sentence string.
 * @param out Pointer to store the parsed GPS data.
 * @retval true If parsing succeeded and valid data was extracted.
 * @retval false If the sentence was invalid or incomplete.
 */
static bool parse_gga(const char *line, gps_data_t *out)
{
    char buf[BUF_SIZE];
    strncpy(buf, line, BUF_SIZE - 1);
    buf[BUF_SIZE - 1] = '\0';

    char *fields[MAX_FIELDS] = {0};
    char *p = buf;
    int idx = 0;

    fields[idx++] = p;
    while (*p && idx < MAX_FIELDS) {
        if (*p == ',') {
            *p = '\0';
            fields[idx++] = p + 1;
        }
        p++;
    }

    /* Expected GGA field layout:
     *  0 = $GPGGA or $GNGGA
     *  1 = UTC time (hhmmss.ss)
     *  2 = Latitude (DDMM.MMMM)
     *  3 = N/S
     *  4 = Longitude (DDDMM.MMMM)
     *  5 = E/W
     *  6 = Fix quality
     *  7 = Number of satellites
     *  8 = HDOP
     *  9 = Altitude (meters)
     */
    if (!fields[0] || !strstr(fields[0], "GGA")) return false;
    if (!fields[2] || !fields[3] || !fields[4] || !fields[5]) return false;

    out->lat = nmea_to_degrees(fields[2], fields[3][0]);
    out->lon = nmea_to_degrees(fields[4], fields[5][0]);
    out->alt = fields[9] ? (float)atof(fields[9]) : 0.0f;
    out->sats = fields[7] ? atoi(fields[7]) : 0;
    out->hdop = fields[8] ? (float)atof(fields[8]) : 0.0f;

    if (fields[1]) {
        strncpy(out->utc_time, fields[1], sizeof(out->utc_time) - 1);
        out->utc_time[sizeof(out->utc_time) - 1] = '\0';
    } else {
        out->utc_time[0] = '\0';
    }

    return true;
}

/**
 * @brief UART interrupt handler for GPS data reception.
 *
 * Reads incoming bytes from the UART FIFO, reconstructs complete NMEA
 * sentences, and triggers parsing for GGA or GNGGA messages. Upon successful
 * parsing, the global GPS data structure is updated and a semaphore is given
 * to signal waiting threads.
 *
 * @param dev Pointer to the UART device generating the interrupt.
 * @param user_data Optional user data pointer (unused).
 */
static void uart_isr(const struct device *dev, void *user_data)
{
    uint8_t c;

    while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
        if (uart_fifo_read(dev, &c, 1) == 1) {
            if (c == '$') {
                line_pos = 0;
                nmea_line[line_pos++] = (char)c;
            } else if (line_pos < (BUF_SIZE - 1)) {
                nmea_line[line_pos++] = (char)c;
            }

            if (c == '\n') {
                nmea_line[line_pos] = '\0';

                if (strstr(nmea_line, "$GPGGA") || strstr(nmea_line, "$GNGGA")) {
                    gps_data_t tmp;
                    if (parse_gga(nmea_line, &tmp)) {
                        memcpy(&parsed_data, &tmp, sizeof(gps_data_t));
                        k_sem_give(&parsed_sem);
                    }
                }

                line_pos = 0;
            }
        } else {
            break;
        }
    }
}

/**
 * @brief Initializes the GPS UART and enables the interrupt handler.
 *
 * Validates the provided configuration, verifies UART readiness, sets up
 * the ISR for GPS data reception, and enables RX interrupts.
 *
 * @param cfg Pointer to the GPS configuration structure.
 * @retval 0 If initialization succeeded.
 * @retval -EINVAL If configuration is invalid.
 * @retval -ENODEV If the UART device is not ready.
 */
int gps_init(const struct gps_config *cfg)
{
    printk("[GPS] - Initializing GPS UART...\n");

    if (!cfg || !cfg->dev) {
        printk("[GPS] - Invalid config\n");
        return -EINVAL;
    }

    uart_dev = cfg->dev;

    if (!device_is_ready(uart_dev)) {
        printk("[GPS] - GPS UART device not ready\n");
        return -ENODEV;
    }

    k_sem_init(&parsed_sem, 0, 1);
    uart_irq_callback_set(uart_dev, uart_isr);
    uart_irq_rx_enable(uart_dev);

    printk("[GPS] - GPS initialized successfully\n");
    return 0;
}

/**
 * @brief Waits for the next valid GGA sentence to be parsed.
 *
 * Blocks until a new GGA frame is available or the specified timeout expires.
 * On success, copies the latest parsed data into the provided buffer.
 *
 * @param out Pointer to store the parsed GPS data.
 * @param timeout Timeout duration (e.g. @c K_FOREVER, @c K_MSEC(2000), @c K_NO_WAIT).
 * @retval 0 If valid GPS data was received before timeout.
 * @retval -ETIMEDOUT If no new GGA sentence was parsed within the timeout period.
 * @retval -EINVAL If the output pointer is invalid.
 */
int gps_wait_for_gga(gps_data_t *out, k_timeout_t timeout)
{
    if (!out) return -EINVAL;

    int ret = k_sem_take(&parsed_sem, timeout);
    if (ret < 0) return ret;

    memcpy(out, &parsed_data, sizeof(gps_data_t));
    return 0;
}
