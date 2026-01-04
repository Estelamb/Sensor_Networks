# Plant Monitoring System with LoRaWAN

## Overview

This project implements a **Plant Monitoring System** using the Zephyr RTOS. It integrates multiple environmental sensors and a GPS module to monitor plant health and location, transmitting the collected data to a cloud dashboard via **LoRaWAN**. The system is designed with a multi-threaded, thread-safe architecture to ensure reliable performance on embedded platforms.



### Key Features

- **Environmental Sensing**: Real-time monitoring of light (Phototransistor), Soil Moisture, and Climate (Si7021 Temp/Hum).
- **Motion & Color Detection**: Monitoring of plant orientation (Accelerometer) and leaf reflectance (Color Sensor).
- **LoRaWAN Connectivity**: Wireless data transmission using OTAA (Over-The-Air Activation) with automatic retries and downlink command support.
- **Geolocation**: GPS tracking with NMEA parsing and scaled integer storage.
- **Multi-threaded Architecture**: 
  - **Main Thread**: Orchestrates the system, manages the LoRaWAN stack, and handles uplinks.
  - **Sensors Thread**: Handles high-priority I2C and ADC data acquisition.
  - **GPS Thread**: Manages UART communication and location parsing.
- **Thread-Safety**: Implementation of atomic variables for shared data and semaphores for precise task synchronization.

---

## Architecture

The system is organized into distinct functional layers, ensuring that time-critical LoRaWAN operations do not interfere with sensor sampling.

### Threading Model

- **Main Thread (Controller)**
  - Initializes hardware and the LoRaWAN stack.
  - Manages the Network Join procedure (OTAA).
  - Triggers child threads, aggregates data into a packed structure, and transmits LoRaWAN uplinks.
  - Handles LoRaWAN downlink callbacks (e.g., remote LED control).

- **Sensors Thread**
  - Interfaces with the ADC (Light/Moisture) and I2C (Temp/Hum, Color, Accelerometer).
  - Stores results in the shared `system_measurement` structure using atomic operations.

- **GPS Thread**
  - Manages UART buffering and NMEA sentence parsing.
  - Provides precise UTC time and coordinates scaled for integer-only transmission.

### Shared Data & Synchronization

- **`system_measurement`**: A central hub of atomic variables. Atomic storage allows the Main thread to read data while the Sensor/GPS threads are updating it without risking race conditions.
- **Semaphores**: Used to synchronize the "Produce-Consume" flow between the Main thread and acquisition threads.
  - `sensors_sem` / `gps_sem`: Triggered by Main to start acquisition.
  - `main_sensors_sem` / `main_gps_sem`: Triggered by threads to signal data is ready.

---

## LoRaWAN Integration

The system utilizes the Zephyr LoRaWAN subsystem to communicate with gateways.

### Connectivity Details
- **Activation**: OTAA (Over-The-Air Activation).
- **Region**: Configurable (e.g., EU868).
- **Payload Design**: Data is "packed" into a 29-byte binary structure to minimize airtime and power consumption.
- **Downlink Commands**: The system listens for specific string commands (`OFF`, `Green`, `Red`) to control an on-board RGB LED remotely.


---

## Sensor & GPS Interfaces

### Environmental & Motion
- **ADC Sensors**: Light and Soil Moisture scaled to 0.1% resolution.
- **I2C Sensors**:
  - **Si7021**: Provides temperature and humidity.
  - **Color Sensor**: Normalizes RGB values based on ambient "Clear" light.
  - **Accelerometer**: Monitors 3-axis motion, scaled to $m/s^2$.

### GPS Data Parsing
- **Format**: Latitude/Longitude degrees scaled by $10^6$ to maintain 6-decimal precision.
- **Time**: Encoded as an array of 3 bytes `[HH, MM, SS]`.

---

## Conclusion
This Plant Monitoring System provides a robust, professional-grade solution for remote environmental monitoring. By combining Zephyr's powerful RTOS capabilities with LoRaWAN's long-range communication, it offers a scalable architecture suitable for agricultural and industrial IoT applications.