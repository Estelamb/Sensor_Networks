# Plant Monitoring System

## Overview

This project implements a **Plant Monitoring System** using the Zephyr RTOS. It integrates multiple sensors and actuators to monitor environmental conditions, log measurements, and provide visual feedback. The system is modular, thread-safe, and supports different operating modes.

### Key Features

- Periodic acquisition of environmental sensor data:
  - Ambient light (phototransistor)
  - Temperature & humidity (Si7021)
  - GPS positioning
- Multi-threaded architecture using Zephyr threads:
  - **Sensors thread**: acquires ADC and I2C sensor data
  - **GPS thread**: reads GPS data periodically or on demand
- Thread-safe shared structures using atomic variables
- Synchronization using semaphores and timers
- Scaled integer representation of sensor values for atomic storage

---

## Architecture

The system is organized as a **multi-threaded embedded application** running on Zephyr RTOS, with clear separation between data acquisition, processing, and main control.  

### Threads

- **Main Thread**  
  - Responsible for system initialization and mode management.  
  - Triggers sensor and GPS measurements via semaphores.  
  - Reads the latest measurements from the shared `system_measurement` structure.

- **Sensors Thread**  
  - Reads ADC and I²C sensors.  
  - Sensors include:  
    - Phototransistor (ambient brightness)  
    - Temperature and humidity (Si7021)  
  - Stores readings atomically in `system_measurement`.

- **GPS Thread**  
  - Reads GPS data.  
  - Parses NMEA sentences and updates `system_measurement` with scaled integer values.  

### Shared Structures

- **`system_context`**  
  - Contains configuration pointers for all sensors and GPS.  
  - Stores semaphores for thread synchronization.  
  - Holds the current operating mode (`system_mode_t`) as an atomic variable.

- **`system_measurement`**  
  - Stores the latest measurements from all sensors.  
  - Values are stored as atomic variables for safe access across threads.  
  - Includes ADC percentages, accelerometer readings, temperature & humidity, color sensor values, and GPS coordinates.

### Synchronization

- **Timers** control measurement cadence depending on the current mode.  
- **Semaphores** allow threads to signal completion or trigger immediate measurement:  
  - `main_sensors_sem` and `main_gps_sem`: signal the main thread when new data is ready.  
  - `sensors_sem` and `gps_sem`: trigger sensor/GPS threads.  
- **Atomic variables** ensure safe concurrent access to shared measurements.

---

## Sensor Interfaces

### ADC Sensors

- Phototransistor (ambient brightness)
- Values are read via `adc_read_voltage()` and scaled to 0–100%

### I2C Sensors

#### Temperature & Humidity (Si7021)

- Provides %RH and °C
- Functions:
  - `temp_hum_init()`
  - `temp_hum_read_humidity()`
  - `temp_hum_read_temperature()`

---

## GPS Interface

- GPS data parsed from NMEA GGA sentences
- Fields stored in `system_measurement`:
  - Latitude / Longitude (degrees ×1e6)
  - Altitude (meters ×100)
  - Satellites in view
  - UTC time encoded as HHMMSS integer
- Thread uses a periodic timer in TEST/NORMAL mode, and waits for semaphore in ADVANCED mode
- Functions:
  - `start_gps_thread()`
  - `gps_wait_for_gga()`

---

## Thread Design

### Sensors Thread

- **Stack size**: 1024 bytes  
- **Priority**: 5  
- Reads:
  - ADC sensor (brightness)
  - Temperature & humidity
- Synchronization:
  - Uses a semaphore
- Stores data atomically for thread-safe access

### GPS Thread

- **Stack size**: 1024 bytes  
- **Priority**: 5  
- Reads GPS data
- Converts GPS floating point values to scaled integers for atomic storage

---

## Synchronization

- **Semaphores**:
  - `main_sensors_sem`: main thread waits for sensor update
  - `main_gps_sem`: main thread waits for GPS update
  - `sensors_sem`: trigger for sensor thread
  - `gps_sem`: trigger for GPS thread
- **Timers**:
  - Control measurement cadence depending on operating mode
- **Atomic variables**:
  - All sensor measurements stored atomically for thread-safe reads/writes

---

## Usage

1. Initialize sensors and GPS:

```c
system_context ctx;
system_measurement measure;

start_sensors_thread(&ctx, &measure);
start_gps_thread(&ctx, &measure);
```


2. Main loop waits for semaphore events and reads shared measurements:

```c
k_sem_take(&ctx.main_sensors_sem, K_FOREVER);
int brightness = atomic_get(&measure.brightness);
```

3. Switch system modes by updating ctx.mode atomically:

```c
atomic_set(&ctx.mode, NORMAL_MODE);
```

## Conclusion
This Plant Monitoring System is a modular, multi-threaded system designed for embedded platforms using Zephyr. It integrates multiple sensors, synchronizes data safely between threads, and supports different operating modes with adaptive behavior.