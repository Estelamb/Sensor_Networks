/**
 * @file main.c
 * @brief Plant Monitoring System main module.
 *
 * This module monitors plant conditions such as light, soil moisture,
 * temperature/humidity, acceleration, color, and GPS location and sends
 * the data to a dashboard with LoRaWAN.
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
#define ACCEL_RANGE ACCEL_2G    /**< Accelerometer full-scale range setting. */

#define COLOR_GAIN GAIN_4X      /**< Color sensor gain setting. */
#define COLOR_INTEGRATION_TIME INTEGRATION_154MS /**< Color sensor integration time in milliseconds. */ 

#define TEMP_HUM_RESOLUTION TH_RES_RH12_TEMP14  /**< Temperature and humidity sensor resolution setting. */

/* --- LoRaWAN Configuration -------------------------------------------------------- */
/* Customize based on network configuration */
#define LORAWAN_DEV_EUI			{ 0x7a, 0x39, 0x32, 0x35, 0x59, 0x37, 0x91, 0x94 } // Use your own DEV_EUI
#define LORAWAN_JOIN_EUI		{ 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x00, 0xFC, 0x4D }
#define LORAWAN_APP_KEY			{ 0xf3, 0x1c, 0x2e, 0x8b, 0xc6, 0x71, 0x28, 0x1d, 0x51, 0x16, 0xf0, 0x8f, 0xf0, 0xb7, 0x92, 0x8f }

#define DELAY K_MSEC(60000)  /* 60 seconds */
#define MAX_PAYLOAD_SIZE   30
#define NUM_MAX_RETRIES    30

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
LOG_MODULE_REGISTER(lorawan_class_a);

/* --- Peripheral configuration ------------------------------------------------ */
/**
 * @brief Phototransistor ADC configuration.
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
 * @brief Soil moisture ADC configuration.
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
 * @brief RGB LED bus configuration.
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
static K_SEM_DEFINE(main_sensors_sem, 0, 1);
static K_SEM_DEFINE(main_gps_sem, 0, 1);
static K_SEM_DEFINE(sensors_sem, 0, 1);
static K_SEM_DEFINE(gps_sem, 0, 1);

/* --- Data ----------------------------------------------------------------- */
/**
 * @brief Shared system context.
 *
 * The @ref system_context structure holds references to the peripheral
 * configurations. The main, sensor, and GPS threads use this structure
 * to coordinate configuration and mode updates.
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
 * @brief Shared measurements between threads.
 *
 * The @ref system_measurement structure holds the current sensor readings
 * accessible to all threads.
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
 * @brief Main data measurement structure.
 */
struct __attribute__((packed)) main_measurement {
    // GPS Data (17 bytes)
    int32_t  lat;       // 4 bytes (Value * 1e6)
    int32_t  lon;       // 4 bytes (Value * 1e6)
    int32_t  alt;       // 4 bytes (Value * 100 in m)
    uint8_t  time[3];   // 3 bytes (Hours, Minutes, Seconds)
    uint8_t  sats;      // 1 byte  (0-255)

    // Temp and hum (4 bytes)
    int16_t  temp;      // 2 bytes (Value * 100)
    uint16_t hum;       // 2 bytes (Value * 100)

    // Brightness (2 bytes)
    uint16_t light;     // 2 bytes (Value * 10)

    // Plant Health (2 bytes)
    uint16_t moisture;  // 2 bytes (Value * 10)

    // Color Sensor (3 bytes)
    uint8_t  r_norm;    // 1 byte (%)
    uint8_t  g_norm;    // 1 byte (%)
    uint8_t  b_norm;    // 1 byte (%)

    // Accelerometer (3 bytes)
    int8_t   x_axis;    // 1 byte (Scaled to fit -128 to 127)
    int8_t   y_axis;    // 1 byte 
    int8_t   z_axis;    // 1 byte
};

/**
 * @brief Main measurement data initialization.
 */
static struct main_measurement main_data = {
    .lat = 0,
    .lon = 0,
    .alt = 0,
    .sats = 0,
    .time = {0, 0, 0},

    .temp = 0,
    .hum = 0,

    .light = 0,
    .moisture = 0,

    .r_norm = 0,
    .g_norm = 0,
    .b_norm = 0,

    .x_axis = 0,
    .y_axis = 0,
    .z_axis = 0,
};

/* --- Helper Functions ----------------------------------------------------- */

static void dl_callback(uint8_t port, uint8_t flags, int16_t rssi, int8_t snr, uint8_t len,
			const uint8_t *hex_data)
{
	LOG_INF("Port %d, Pending %d, RSSI %ddB, SNR %ddBm, Time %d", port,
		flags & LORAWAN_DATA_PENDING, rssi, snr, !!(flags & LORAWAN_TIME_UPDATED));
	if (hex_data) {
		LOG_HEXDUMP_INF(hex_data, len, "Payload: ");

        if (strncmp((const char *)hex_data, "OFF", len) == 0) {
			LOG_INF("Command received: Turn off LED");
			rgb_led_off(&rgb_leds);
		} 
		else if (strncmp((const char *)hex_data, "Green", len) == 0) {
			LOG_INF("Command received: Green Color");
			rgb_green(&rgb_leds);
		} 
		else if (strncmp((const char *)hex_data, "Red", len) == 0) {
			LOG_INF("Command received: Red Color");
			rgb_red(&rgb_leds);
		}
	}
}

static void lorwan_datarate_changed(enum lorawan_datarate dr)
{
	uint8_t unused, max_size;

	lorawan_get_payload_sizes(&unused, &max_size);
	LOG_INF("New Datarate: DR_%d, Max Payload %d", dr, max_size);
}

/**
 * @brief Retrieves the latest measurements from atomic variables.
 */
static void get_measurements()
{
    // GPS
    main_data.lat = (int32_t)atomic_get(&measure.gps_lat);
    main_data.lon = (int32_t)atomic_get(&measure.gps_lon);
    main_data.alt = (int32_t)atomic_get(&measure.gps_alt);
    main_data.sats = (uint8_t)atomic_get(&measure.gps_sats);

    // Time Compression: HHMMSS -> [HH, MM, SS]
    uint32_t full_time = (uint32_t)atomic_get(&measure.gps_time);
    main_data.time[0] = (uint8_t)(full_time / 10000);       // Hours
    main_data.time[1] = (uint8_t)((full_time / 100) % 100); // Minutes
    main_data.time[2] = (uint8_t)(full_time % 100);         // Seconds
    
    // Temp and hum
    main_data.temp = (int16_t)atomic_get(&measure.temp);
    main_data.hum = (uint16_t)atomic_get(&measure.hum);

    // Light and moisture
    main_data.light = (uint16_t)atomic_get(&measure.brightness);
    main_data.moisture = (uint16_t)atomic_get(&measure.moisture);

    // Color Normalization (Converted to 0-100%)
    uint32_t clear = atomic_get(&measure.clear);

    if (clear > 0) {
        main_data.r_norm = (uint8_t)((atomic_get(&measure.red) * 100) / clear);
        main_data.g_norm = (uint8_t)((atomic_get(&measure.green) * 100) / clear);
        main_data.b_norm = (uint8_t)((atomic_get(&measure.blue) * 100) / clear);
    }

    // Accelerometer (1 decimal)
    main_data.x_axis = (int8_t)(atomic_get(&measure.accel_x) / 10);
    main_data.y_axis = (int8_t)(atomic_get(&measure.accel_y) / 10);
    main_data.z_axis = (int8_t)(atomic_get(&measure.accel_z) / 10);
}

/**
 * @brief Displays the latest measurements via printk.
 */
static void display_measurements()
{
    printk("\n--- SENSOR REPORT ---\n");
    printk("SOIL MOISTURE: %.1f%%\n", (double)main_data.moisture / 10.0);

    printk("LIGHT: %.1f%%\n", (double)main_data.light / 10.0);

    printk("GPS: #Sats: %d | Time: %02d:%02d:%02d\n", 
            main_data.sats, 
            main_data.time[0], 
            main_data.time[1], 
            main_data.time[2]);

    printk("GPS POS: Lat: %.6f, Lon: %.6f, Alt: %.2f m\n",
            (double)main_data.lat / 1e6, 
            (double)main_data.lon / 1e6, 
            (double)main_data.alt / 100.0);

    printk("COLOR: R:%d%% G:%d%% B:%d%%\n", main_data.r_norm, main_data.g_norm, main_data.b_norm);

    printk("ACCELEROMETER: X: %.1f m/s2, Y: %.1f m/s2, Z: %.1f m/s2 \n",
            (double)main_data.x_axis / 10.0, 
            (double)main_data.y_axis / 10.0, 
            (double)main_data.z_axis / 10.0);
    
    printk("TEMP: %.2f C | HUM: %.2f %%\n", (double)main_data.temp / 100.0, (double)main_data.hum / 100.0);
    printk("---------------------\n");
}

/* --- Main Application -------------------------------------------------------- */
/**
 * @brief Main entry point for the brightness control system.
 *
 * Initializes peripherals (RGB LED, ADC, user button), starts the
 * brightness thread, and executes the LED update loop.
 *
 * Button input is interrupt-driven; all press logic is handled by ISR and workqueue.
 *
 * @return This function does not return under normal operation.
 */
int main(void)
{
    printk("==== Plant Monitoring System with ResIoT ====\n");
    printk("System ON\n\n");

    /* Initialize peripherals */
    if (gps_init(&gps)) {
        printk("GPS initialization failed - Program stopped\n");
        return -1;
    }
    if (adc_init(&pt)) {
        printk("Phototransistor initialization failed - Program stopped\n");
        return -1;
    }
    if (adc_init(&sm)) {
        printk("Soil moisture sensor initialization failed - Program stopped\n");
        return -1;
    }
    if (accel_init(&accel, ACCEL_RANGE)) {
        printk("Accelerometer initialization failed - Program stopped\n");
        return -1;
    }
    if (temp_hum_init(&th, TEMP_HUM_RESOLUTION)) {
        printk("Temperature/Humidity sensor initialization failed - Program stopped\n");
        return -1;
    }
    if (color_init(&color, COLOR_GAIN, COLOR_INTEGRATION_TIME)) {
        printk("Color sensor initialization failed - Program stopped\n");
        return -1;
    }
    if (rgb_led_init(&rgb_leds) || rgb_led_off(&rgb_leds)) {
        printk("RGB LED initialization failed - Program stopped\n");
        return -1;
    }

    /* Start measurement threads */
    start_sensors_thread(&ctx, &measure);
    start_gps_thread(&ctx, &measure);

    const struct device *lora_dev;
	struct lorawan_join_config join_cfg;
	uint8_t dev_eui[] = LORAWAN_DEV_EUI;
	uint8_t join_eui[] = LORAWAN_JOIN_EUI;
	uint8_t app_key[] = LORAWAN_APP_KEY;
	int ret;
	uint8_t joining_retries = 0 ;
	
	struct lorawan_downlink_cb downlink_cb = {
		.port = LW_RECV_PORT_ANY,
		.cb = dl_callback
	};

	lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
	if (!device_is_ready(lora_dev)) {
		LOG_ERR("%s: device not ready.", lora_dev->name);
		return 0;
	}

#if defined(CONFIG_LORAMAC_REGION_EU868)
	/* If more than one region Kconfig is selected, app should set region
	 * before calling lorawan_start()
	 */
	ret = lorawan_set_region(LORAWAN_REGION_EU868);
	if (ret < 0) {
		LOG_ERR("lorawan_set_region failed: %d", ret);
		return 0;
	}
#endif

	ret = lorawan_start();
	if (ret < 0) {
		LOG_ERR("lorawan_start failed: %d", ret);
		return 0;
	}
	
	lorawan_enable_adr(false); // enable adaptative data rate. not recommended for mobile (non-static position) devices

	lorawan_register_downlink_callback(&downlink_cb);
	lorawan_register_dr_changed_callback(lorwan_datarate_changed);

	join_cfg.mode = LORAWAN_ACT_OTAA;
	join_cfg.dev_eui = dev_eui;
	join_cfg.otaa.join_eui = join_eui;
	join_cfg.otaa.app_key = app_key;
	join_cfg.otaa.nwk_key = app_key;
	
	printf("\r\n DEV_EUI: ");
    for (int i = 0; i < sizeof(dev_eui); ++i) printf("%02x", dev_eui[i]);
    printf("\r\n APP_EUI: ");
    for (int i = 0; i < sizeof(join_eui); ++i) printf("%02x", join_eui[i]);
    printf("\r\n APP_KEY: ");
    for (int i = 0; i < sizeof(app_key); ++i) printf("%02x", app_key[i]);
    printf("\r\n");

	LOG_INF("Waiting 10 seconds to join...");
	k_sleep(K_SECONDS(10));  // Wait 30 seconds


	LOG_INF("Joining network over OTAA");
    while ((ret = lorawan_join(&join_cfg)) < 0) {
		joining_retries++;
		if (joining_retries > NUM_MAX_RETRIES){
			LOG_ERR("Number max of retries (%d/%d) reached, finishing program", joining_retries, NUM_MAX_RETRIES);
			return 0;
		}
		LOG_ERR("Join failed (%d), retrying in 30 seconds, try #%d...", ret, joining_retries);
		k_sleep(K_SECONDS(30));
	}
		
	LOG_INF("Starting to send data...");

    while (1) {
        k_sem_give(ctx.sensors_sem);
        k_sem_give(ctx.gps_sem);

        k_sem_take(ctx.main_sensors_sem, K_FOREVER);
        k_sem_take(ctx.main_gps_sem, K_FOREVER);

        get_measurements();
        
        ret = lorawan_send(1, (uint8_t *)&main_data, sizeof(main_data),
                           LORAWAN_MSG_UNCONFIRMED);

        if (ret < 0) {
            LOG_ERR("lorawan_send failed: %d", ret);
        } else {
            LOG_INF("Data sent! Size: %d bytes", sizeof(main_data));
        }

        display_measurements(); 

        k_sleep(DELAY);
    }
}
