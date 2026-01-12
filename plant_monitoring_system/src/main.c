/**
 * @file main.c
 * @brief Plant Monitoring System main module.
 *
 * This module monitors plant conditions such as light, soil moisture,
 * temperature/humidity, acceleration, color, and GPS location and sends
 * the data to a dashboard via LoRaWAN.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/lorawan/lorawan.h>
#include <math.h>

#include "main.h"
#include "sensors_thread.h"
#include "gps_thread.h"

/* --- Sensors Configuration -------------------------------------------------------- */
#define ACCEL_RANGE ACCEL_2G      /**< Accelerometer full-scale range setting. */

#define COLOR_GAIN GAIN_4X        /**< Color sensor gain setting. */
#define COLOR_INTEGRATION_TIME INTEGRATION_154MS /**< Color sensor integration time in ms. */ 

#define TEMP_HUM_RESOLUTION TH_RES_RH12_TEMP14  /**< Temp/Hum sensor resolution setting. */

/* --- LoRaWAN Configuration -------------------------------------------------------- */
#define LORAWAN_DEV_EUI     { 0x7a, 0x39, 0x32, 0x35, 0x59, 0x37, 0x91, 0x94 }
#define LORAWAN_JOIN_EUI    { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x00, 0xFC, 0x4D }
#define LORAWAN_APP_KEY     { 0xf3, 0x1c, 0x2e, 0x8b, 0xc6, 0x71, 0x28, 0x1d, 0x51, 0x16, 0xf0, 0x8f, 0xf0, 0xb7, 0x92, 0x8f }

#define DELAY               K_MSEC(60000) /**< Data transmission interval (60s). */
#define JOIN_RETRY_DELAY    K_SECONDS(30) /**< Delay between network join attempts. */
#define NUM_MAX_RETRIES     30            /**< Maximum number of join retries. */

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
LOG_MODULE_REGISTER(plant_monitor_main);

/* --- Peripheral configuration ------------------------------------------------ */
/**
 * @brief Phototransistor (Light Sensor) ADC configuration.
 */
static struct adc_config pt = {
    .dev = DEVICE_DT_GET(DT_NODELABEL(adc1)),
    .channel_id = 5,
    .resolution = 12,
    .gain = ADC_GAIN_1,
    .ref = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .vref_mv = 3300,
};

/**
 * @brief Soil moisture sensor ADC configuration.
 */
static struct adc_config sm = {
    .dev = DEVICE_DT_GET(DT_NODELABEL(adc1)),
    .channel_id = 0,
    .resolution = 12,
    .gain = ADC_GAIN_1,
    .ref = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .vref_mv = 3300,
};

/**
 * @brief Accelerometer I2C configuration.
 */
static struct i2c_dt_spec accel = {
    .bus = DEVICE_DT_GET(DT_NODELABEL(i2c2)),
    .addr = ACCEL_I2C_ADDR,
};

/**
 * @brief Temperature and humidity sensor I2C configuration.
 */
static struct i2c_dt_spec th = {
    .bus = DEVICE_DT_GET(DT_NODELABEL(i2c2)),
    .addr = TH_I2C_ADDR,
};

/**
 * @brief Color sensor I2C configuration.
 */
static struct i2c_dt_spec color = {
    .bus = DEVICE_DT_GET(DT_NODELABEL(i2c2)),
    .addr = COLOR_I2C_ADDR,
};

/**
 * @brief GPS UART configuration.
 */
static struct gps_config gps = {
    .dev = DEVICE_DT_GET(DT_NODELABEL(usart1)),
};

/**
 * @brief RGB LED GPIO bus configuration.
 */
static struct bus_rgb_led rgb_leds = {
    .pins = {
        GPIO_DT_SPEC_GET(DT_ALIAS(red), gpios),
        GPIO_DT_SPEC_GET(DT_ALIAS(green), gpios),
        GPIO_DT_SPEC_GET(DT_ALIAS(blue), gpios)
    },
    .pin_count = BUS_SIZE,
};

/* --- Semaphores ----------------------------------------------------------- */
static K_SEM_DEFINE(main_sensors_sem, 0, 1); /**< Signal from sensors thread to main. */
static K_SEM_DEFINE(main_gps_sem, 0, 1);     /**< Signal from GPS thread to main. */
static K_SEM_DEFINE(sensors_sem, 0, 1);      /**< Trigger for sensors thread. */
static K_SEM_DEFINE(gps_sem, 0, 1);          /**< Trigger for GPS thread. */

/* --- Data ----------------------------------------------------------------- */
/**
 * @brief Shared system context.
 *
 * Holds references to peripheral configurations and semaphores used
 * for inter-thread coordination.
 */
static struct system_context ctx = {
    .phototransistor = &pt,
    .soil_moisture = &sm,
    .accelerometer = &accel,
    .accel_range = ACCEL_RANGE,
    .temp_hum = &th,
    .color = &color,
    .gps = &gps,
    .main_sensors_sem = &main_sensors_sem,
    .main_gps_sem = &main_gps_sem,
    .sensors_sem = &sensors_sem,
    .gps_sem = &gps_sem,
};

/**
 * @brief Shared measurements container (atomic storage).
 */
static struct system_measurement measure = {
    .brightness = ATOMIC_INIT(0),
    .moisture = ATOMIC_INIT(0),
    .accel_x = ATOMIC_INIT(0),
    .accel_y = ATOMIC_INIT(0),
    .accel_z = ATOMIC_INIT(0),
    .temp = ATOMIC_INIT(0),
    .hum = ATOMIC_INIT(0),
    .red = ATOMIC_INIT(0),
    .green = ATOMIC_INIT(0),
    .blue = ATOMIC_INIT(0),
    .clear = ATOMIC_INIT(0),
    .gps_lat = ATOMIC_INIT(0),
    .gps_lon = ATOMIC_INIT(0),
    .gps_alt = ATOMIC_INIT(0),
    .gps_sats = ATOMIC_INIT(0),
    .gps_time = ATOMIC_INIT(0),
};

/**
 * @brief LoRaWAN Uplink payload structure.
 */
struct __attribute__((packed)) main_measurement {
    // GPS Data (17 bytes)
    int32_t  lat;       // 4 bytes (Scaled by 1e6)
    int32_t  lon;       // 4 bytes (Scaled by 1e6)
    int32_t  alt;       // 4 bytes (Value * 100 in meters)
    uint8_t  time[3];   // 3 bytes (HH, MM, SS)
    uint8_t  sats;      // 1 byte  (Satellites in view)

    // Temperature and Humidity (4 bytes)
    int16_t  temp;      // 2 bytes (Celsius * 100)
    uint16_t hum;       // 2 bytes (Relative humidity * 10)

    // Light and Soil (4 bytes)
    uint16_t light;     // 2 bytes (Percentage * 10)
    uint16_t moisture;  // 2 bytes (Percentage * 10)

    // Color (3 bytes)
    uint8_t  r_norm;    // 1 byte
    uint8_t  g_norm;    // 1 byte
    uint8_t  b_norm;    // 1 byte

    // Accelerometer (3 bytes)
    int8_t   x_axis;    // 1 byte
    int8_t   y_axis;    // 1 byte 
    int8_t   z_axis;    // 1 byte
};

static struct main_measurement main_data;

/* --- LoRaWAN Callbacks and Helpers ---------------------------------------- */

static struct lorawan_join_config join_cfg;
static uint8_t dev_eui[] = LORAWAN_DEV_EUI;
static uint8_t join_eui[] = LORAWAN_JOIN_EUI;
static uint8_t app_key[] = LORAWAN_APP_KEY;

/**
 * @brief Downlink message callback.
 */
static void dl_callback(uint8_t port, uint8_t flags, int16_t rssi, int8_t snr, 
                        uint8_t len, const uint8_t *hex_data)
{
    LOG_INF("Downlink: Port %d, RSSI %ddB, SNR %ddBm", port, rssi, snr);
    if (hex_data) {
        LOG_HEXDUMP_INF(hex_data, len, "Payload: ");
        if (strncmp((const char *)hex_data, "OFF", len) == 0) rgb_led_off(&rgb_leds);
        else if (strncmp((const char *)hex_data, "Green", len) == 0) rgb_green(&rgb_leds);
        else if (strncmp((const char *)hex_data, "Red", len) == 0) rgb_red(&rgb_leds);
    }
}

/**
 * @brief LoRaWAN Data Rate change callback.
 */
static void lorwan_datarate_changed(enum lorawan_datarate dr)
{
    uint8_t unused, max_size;
    lorawan_get_payload_sizes(&unused, &max_size);
    LOG_INF("New Datarate: DR_%d, Max Payload Size: %d", dr, max_size);
}

/**
 * @brief Initializes the LoRaWAN stack and driver.
 * @return 0 on success, negative error code otherwise.
 */
static int init_lorawan(void)
{
    const struct device *lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
    int ret;

    if (!device_is_ready(lora_dev)) {
        LOG_ERR("LoRa device not ready");
        return -1;
    }

    struct lorawan_downlink_cb downlink_cb = {
        .port = LW_RECV_PORT_ANY,
        .cb = dl_callback
    };

#if defined(CONFIG_LORAMAC_REGION_EU868)
    ret = lorawan_set_region(LORAWAN_REGION_EU868);
    if (ret < 0) return ret;
#endif

    ret = lorawan_start();
    if (ret < 0) return ret;

    /* Configuration */
    lorawan_enable_adr(false); // Static/slow ADR disabled for mobile devices
    lorawan_register_downlink_callback(&downlink_cb);
    lorawan_register_dr_changed_callback(lorwan_datarate_changed);

    /* OTAA Configuration */
    join_cfg.mode = LORAWAN_ACT_OTAA;
    join_cfg.dev_eui = dev_eui;
    join_cfg.otaa.join_eui = join_eui;
    join_cfg.otaa.app_key = app_key;
    join_cfg.otaa.nwk_key = app_key;

    return 0;
}

/**
 * @brief Joins the LoRaWAN network using OTAA.
 * @return 0 on success, negative error code on timeout.
 */
static int join_lorawan(void)
{
    int ret;
    uint8_t retries = 0;

    LOG_INF("Attempting to join network via OTAA...");
    while ((ret = lorawan_join(&join_cfg)) < 0) {
        retries++;
        if (retries > NUM_MAX_RETRIES) {
            LOG_ERR("Maximum join retries reached. Stopping.");
            return -ETIMEDOUT;
        }
        LOG_WRN("Join attempt %d/%d failed (%d). Retrying in 30s...", retries, NUM_MAX_RETRIES, ret);
        k_sleep(JOIN_RETRY_DELAY);
    }
    LOG_INF("Join successful!");
    return 0;
}

/* --- Data Processing Helpers ---------------------------------------------- */

/**
 * @brief Formats raw sensor data into the transmission structure.
 */
static void get_measurements(void)
{
    // GPS Data
    main_data.lat = (int32_t)atomic_get(&measure.gps_lat);
    main_data.lon = (int32_t)atomic_get(&measure.gps_lon);
    main_data.alt = (int32_t)atomic_get(&measure.gps_alt);
    main_data.sats = (uint8_t)atomic_get(&measure.gps_sats);

    // Time Decompression: HHMMSS -> [HH, MM, SS]
    uint32_t full_time = (uint32_t)atomic_get(&measure.gps_time);
    main_data.time[0] = (uint8_t)(full_time / 10000);
    main_data.time[1] = (uint8_t)((full_time / 100) % 100);
    main_data.time[2] = (uint8_t)(full_time % 100);
    
    // Temperature and Humidity
    main_data.temp = (int16_t)atomic_get(&measure.temp);
    main_data.hum = (uint16_t)atomic_get(&measure.hum);

    // Soil and Light
    main_data.light = (uint16_t)atomic_get(&measure.brightness);
    main_data.moisture = (uint16_t)atomic_get(&measure.moisture);

    // Color Normalization (0-100%)
    uint32_t clear = atomic_get(&measure.clear);
    if (clear > 0) {
        main_data.r_norm = (uint8_t)((atomic_get(&measure.red)   * 100) / clear);
        main_data.g_norm = (uint8_t)((atomic_get(&measure.green) * 100) / clear);
        main_data.b_norm = (uint8_t)((atomic_get(&measure.blue)  * 100) / clear);
    }

    // Accelerometer data
    main_data.x_axis = (int8_t)(atomic_get(&measure.accel_x) / 10);
    main_data.y_axis = (int8_t)(atomic_get(&measure.accel_y) / 10);
    main_data.z_axis = (int8_t)(atomic_get(&measure.accel_z) / 10);
}

/**
 * @brief Prints current sensor status to the serial console.
 */
static void display_measurements(void)
{
    printk("-------------- SENSOR REPORT --------------\n");

    // 1. Soil Moisture
    printk("MOISTURE:  Raw: %ld | LoRa: %u | Value: %.1f%%\n",
           atomic_get(&measure.moisture), main_data.moisture, (double)main_data.moisture / 10.0);

    // 2. Light
    printk("LIGHT:     Raw: %ld | LoRa: %u | Value: %.1f%%\n",
           atomic_get(&measure.brightness), main_data.light, (double)main_data.light / 10.0);

    // 3. Temperature & Humidity
    printk("TEMP:      Raw: %ld | LoRa: %d | Value: %.2f C\n",
           atomic_get(&measure.temp), main_data.temp, (double)main_data.temp / 100.0);
    printk("HUMIDITY:  Raw: %ld | LoRa: %u | Value: %.2f%%\n",
           atomic_get(&measure.hum), main_data.hum, (double)main_data.hum / 100.0);

    // 4. GPS Location
    printk("LATITUDE:  Raw: %ld | LoRa: %d | Value: %.6f\n",
           atomic_get(&measure.gps_lat), main_data.lat, (double)main_data.lat / 1e6);
    printk("LONGITUDE: Raw: %ld | LoRa: %d | Value: %.6f\n",
           atomic_get(&measure.gps_lon), main_data.lon, (double)main_data.lon / 1e6);
    printk("ALTITUDE:  Raw: %ld | LoRa: %d | Value: %.2f m\n",
           atomic_get(&measure.gps_alt), main_data.alt, (double)main_data.alt / 100.0);

    // 5. GPS Sats & Time
    printk("GPS SATS:  Raw: %ld | LoRa: %u | Value: %u satellites\n",
           atomic_get(&measure.gps_sats), main_data.sats, main_data.sats);
    
    printk("GPS TIME:  Raw: %ld | LoRa: [%02d,%02d,%02d] | Value: %02d:%02d:%02d\n",
           atomic_get(&measure.gps_time), 
           main_data.time[0], main_data.time[1], main_data.time[2],
           main_data.time[0], main_data.time[1], main_data.time[2]);

    // 6. Color (Normalizado en LoRa y Value)
    printk("COLOR:     Raw R:%ld G:%ld B:%ld | LoRa R:%u%% G:%u%% B:%u%%\n",
           atomic_get(&measure.red), atomic_get(&measure.green), atomic_get(&measure.blue),
           main_data.r_norm, main_data.g_norm, main_data.b_norm);

    // 7. Accelerometer
    printk("ACCEL:     Raw X:%ld Y:%ld Z:%ld | Value X:%.1f Y:%.1f Z:%.1f m/s2\n",
           atomic_get(&measure.accel_x), atomic_get(&measure.accel_y), atomic_get(&measure.accel_z),
           (double)main_data.x_axis / 10.0, (double)main_data.y_axis / 10.0, (double)main_data.z_axis / 10.0);

    printk("------------------------------------------\n\n");
}

/* --- Main Application ----------------------------------------------------- */

int main(void)
{
    printk("==== Plant Monitoring System (ResIoT/LoRaWAN) ====\n");

    /* 1. Hardware Initialization */
    if (gps_init(&gps) || adc_init(&pt) || adc_init(&sm) || 
        accel_init(&accel, ACCEL_RANGE) || temp_hum_init(&th, TEMP_HUM_RESOLUTION) ||
        color_init(&color, COLOR_GAIN, COLOR_INTEGRATION_TIME) || 
        rgb_led_init(&rgb_leds) || rgb_led_off(&rgb_leds)) {
        LOG_ERR("Hardware initialization failed. Aborting.");
        return -1;
    }

    /* 2. LoRaWAN Stack Initialization */
    if (init_lorawan() < 0) {
        LOG_ERR("LoRaWAN stack initialization failed.");
        return -1;
    }

    /* 3. Thread Launch */
    start_sensors_thread(&ctx, &measure);
    start_gps_thread(&ctx, &measure);

    /* 4. Join Network */
    if (join_lorawan() < 0) {
        return -1;
    }

    /* 5. Main Loop: Sensor Sampling & LoRaWAN Transmission */
    while (1) {
        /* Request new readings from threads */
        k_sem_give(ctx.sensors_sem);
        k_sem_give(ctx.gps_sem);

        /* Wait for thread completion */
        k_sem_take(ctx.main_sensors_sem, K_FOREVER);
        k_sem_take(ctx.main_gps_sem, K_FOREVER);

        get_measurements();
        
        /* Send uplink message */
        int ret = lorawan_send(1, (uint8_t *)&main_data, sizeof(main_data), LORAWAN_MSG_UNCONFIRMED);
        if (ret < 0) {
            LOG_ERR("LoRaWAN transmission failed: %d", ret);
        } else {
            LOG_INF("Data packet sent successfully (%d bytes)", sizeof(main_data));
        }

        display_measurements(); 
        k_sleep(DELAY);
    }
}