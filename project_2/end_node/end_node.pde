/**
 * @file End_Node.pde
 * @brief End Node implementation for Sensor Networks Project.
 * * This node collects environmental and inertial data periodically and monitors
 * for critical events (Free Fall and Motion). Data is transmitted via XBee 
 * 802.15.4 to the Gateway Node.
 * * @author Alejandro Botas Bárcena
 * @author Javier Grimaldos Chavarría
 * @author Estela Mora Barba
 * @date February 2026
 */
#include <WaspSensorEvent_v30.h> 
#include <WaspXBee802.h>
#include <WaspFrame.h>

// Period definition (30 seconds)
#define SAMPLING_PERIOD "00:00:00:30" /**< Duty cycle period (30 seconds) */

// XBee
char RX_ADDRESS[] = "0013A20041951872"; /**< MAC Address of the Gateway Node */
char WASPMOTE_ID[] = "node_01";          /**< Local identifier for this node */
uint8_t panID[2] = {0x12, 0x34};        /**< Network ID (Must match Gateway) */
uint8_t channel = 0x0F;                 /**< 2.4GHz Frequency Channel */
uint8_t encryptionMode = 0;             /**< AES Encryption toggle */
char encryptionKey[] = "Orange";        /**< Shared security key */
uint8_t error;                          /**< Variable for error handling */

// Global variables for sensor data
float node_temp, node_humd, node_pres, node_x_acc, node_y_acc, node_z_acc;
uint8_t node_batteryLevel;
pirSensorClass pir(SOCKET_1);           /**< PIR Sensor instance on Socket 1 */
uint8_t node_motion = 0;                /**< Flag for motion status */

/**
 * @brief Initializes the XBee 802.15.4 module.
 * * Sets the network parameters (Channel, PAN ID, Encryption) and writes 
 * them to the module's permanent memory.
 */
void initXBee() {
  USB.println(F("Initializing XBee"));
  xbee802.ON();

  // 1. Set channel
  xbee802.setChannel(channel);
  if (xbee802.error_AT == 0)
  {
    USB.print(F("1. Channel set OK to: 0x"));
    USB.printHex(xbee802.channel);
    USB.println();
  }

  // 2. Set PANID
  xbee802.setPAN(panID);
  if (xbee802.error_AT == 0)
  {
    USB.print(F("2. PAN ID set OK to: 0x"));
    USB.printHex(xbee802.PAN_ID[0]);
    USB.printHex(xbee802.PAN_ID[1]);
    USB.println();
  }

  // 3. Set encryption mode
  xbee802.setEncryptionMode(encryptionMode);
  if (xbee802.error_AT == 0)
  {
    USB.print(F("3. AES encryption mode: "));
    USB.println(xbee802.encryptMode, DEC);
  }

  // 4. Set encryption key
  xbee802.setLinkKey(encryptionKey);
  if (xbee802.error_AT == 0)  USB.println(F("4. AES encryption key set OK"));

  // 5. Write values
  xbee802.writeValues();
  if (xbee802.error_AT == 0) USB.println(F("5. Changes stored OK"));

  USB.println(F("XBee Initialized"));
}

/**
 * @brief Sends a data packet via XBee to the Gateway.
 * * @param data Pointer to the byte array to send.
 * @param dataLength Number of bytes to transmit.
 */
void sendXBee(uint8_t* data, uint16_t dataLength) {
  // Switch red LED on before attempting data transmission 
  Utils.setLED(LED0, LED_ON); 
  USB.println("Sending XBee Message ...");

  // Send XBee packet to Gateway
  error = xbee802.send(RX_ADDRESS, data, dataLength);
  
  // Check Data Transmission
  if (error == 0) {
    USB.println(F("XBee Message Sent OK"));
    USB.println("");
  } else {
    USB.print(F("XBee Send Error: "));
    USB.println(error, DEC);
    USB.println("");
  }

  // Switch red LED off when transmission is completed 
  Utils.setLED(LED0, LED_OFF); 
}

/**
 * @brief Processes hardware interruptions from Accelerometer and PIR sensor.
 * * If a Free Fall (ACC_INT) or Motion (SENS_INT) is detected, it builds an 
 * alarm payload and transmits it immediately.
 */
void handleAlarms() {
  char alarmPayload[40];

  // Detect Free-Fall interruption
  if (intFlag & ACC_INT)
  {    
    USB.println(F("-------------------------------"));
    USB.println(F("-- ALARM: FREE FALL DETECTED --"));
    USB.println(F("-------------------------------"));
    USB.println("");
      
    intFlag &= ~(ACC_INT); 

    // Send Alarm
    sprintf(alarmPayload, "A=FREE FALL;ID=%s", WASPMOTE_ID);
    sendXBee((uint8_t*)alarmPayload, strlen(alarmPayload));
  }

  // Detect Motion (PIR) interruption
  if (intFlag & SENS_INT)
  {
    Events.detachInt();
    Events.loadInt();
    if (pir.getInt())
    {
      USB.println(F("----------------------------"));
      USB.println(F("-- ALARM: MOTION DETECTED --"));
      USB.println(F("----------------------------"));
      USB.println("");

      // Send Alarm
      sprintf(alarmPayload, "A=MOTION;ID=%s", WASPMOTE_ID);
      sendXBee((uint8_t*)alarmPayload, strlen(alarmPayload));
    }

    intFlag &= ~(SENS_INT);
  }

  // Re-arm interruptions before going back to sleep
  ACC.ON(); 
  ACC.setFF();
  Events.attachInt();
}

/**
 * @brief Powers down modules and enters Deep Sleep.
 * * The system wakes up via RTC alarm (periodic) or External Interrupt (alarms).
 */
void sleepMode() {
  USB.println(F("Entering low power state..."));
  
  // Turn off communication module to conserve energy 
  xbee802.OFF(); 
  
  // Enter Deep Sleep for 30s or until an interruption occurs
  PWR.deepSleep(SAMPLING_PERIOD, RTC_OFFSET, RTC_ALM1_MODE1, SENSOR_ON); // SENSOR_ON keeps the board powered for PIR and ACC interruptions
  
  // After waking up, restore necessary modules
  USB.ON();
  xbee802.ON();
  PWR.clearInterruptionPin();
}

/**
 * @brief Captures data from all local sensors.
 */
void readSensors() {
  // Turn green LED on during the sensor measurement process 
  Utils.setLED(LED1, LED_ON);
    
  // Turn on the sensor board
  Events.ON();
  
  // Collect Measurements 
  node_temp = Events.getTemperature();
  node_humd = Events.getHumidity();
  node_pres = Events.getPressure();
  node_batteryLevel = PWR.getBatteryLevel();
  
  // Read Accelerometer values 
  ACC.ON();
  node_x_acc = (float)ACC.getX() * 0.0098;
  node_y_acc = (float)ACC.getY() * 0.0098;
  node_z_acc = (float)ACC.getZ() * 0.0098;

  // Turn green LED off after measurement
  Utils.setLED(LED1, LED_OFF); 
}

/**
 * @brief Prints sensor data to Serial USB for local debugging.
 */
void displayData(const char* device, float temp, float humd, float pres, uint8_t bat, float x, float y, float z) {
  USB.print(F("--------- ")); USB.print(device); USB.println(F(" PERIODICAL MEASUREMENTS ---------"));
  USB.print(F("Temperature: ")); USB.printFloat(temp, 2); USB.println(F(" C"));
  USB.print(F("Humidity: ")); USB.printFloat(humd, 2);  USB.println(F(" %"));
  USB.print(F("Pressure: ")); USB.printFloat(pres, 2); USB.println(F(" Pa"));
  USB.print(F("Battery Level: ")); USB.print(bat, DEC); USB.println(F(" %"));
  USB.print(F("Acceleration (m/s2): "));
  USB.print(F("X: ")); USB.printFloat(x, 2); 
  USB.print(F(", Y: ")); USB.printFloat(y, 2);
  USB.print(F(", Z: ")); USB.printFloat(z, 2); USB.println("");
  USB.println("---------------------------------------------------"); USB.println("");
}

/**
 * @brief Packages and transmits periodic sensor readings.
 * * Formats data into a "Key=Value" string suitable for Gateway parsing.
 */
void sendPeriodicData() {
  char payload[100];
  
  // Floats to Strings
  char t_str[10], h_str[10], p_str[15], x_str[10], y_str[10], z_str[10];

  Utils.float2String(node_temp, t_str, 2);
  Utils.float2String(node_humd, h_str, 2);
  Utils.float2String(node_pres, p_str, 2);
  Utils.float2String(node_x_acc, x_str, 2);
  Utils.float2String(node_y_acc, y_str, 2);
  Utils.float2String(node_z_acc, z_str, 2);

  sprintf(payload, "T=%s;H=%s;P=%s;B=%d;X=%s;Y=%s;Z=%s;", 
          t_str, 
          h_str, 
          p_str, 
          (int)node_batteryLevel, 
          x_str, 
          y_str, 
          z_str);

  sendXBee((uint8_t*)payload, strlen(payload));
}

/**
 * @brief Initial configuration for End Node hardware.
 */
void setup() {
  USB.ON();
  USB.println(F("End Node Initializing..."));

  // Accelerometer Setup
  ACC.ON();
  ACC.setFF();
  USB.println(F("Accelerometer Initialized"));

  // PIR Setup
  Events.ON();
  node_motion = pir.readPirSensor();
  USB.print(F("Waiting for PIR stabilization ..."));
  
  while (node_motion == 1) // Wait for PIR signal stabilization
  {
    USB.print(F("..."));
    delay(1000);
    node_motion = pir.readPirSensor();    
  }
  
  Events.attachInt(); // Enable interruptions from the board

  USB.println("");
  USB.println(F("PIR Stabilized"));

  // XBee Setup
  frame.setID( WASPMOTE_ID );
  initXBee();
  
  USB.println(F("End Node Setup completed"));
}

/**
 * @brief Main logic loop. 
 * * Handles alarms, periodic data reporting, and power management.
 */
void loop() {
  // 1. ALARM HANDLING (Maximum Priority)
  if (intFlag & (ACC_INT | SENS_INT)) handleAlarms(); // Check if the wake-up cause was a sensor interruption
  
  // 2. PERIODICAL MEASUREMENTS (Every 30 seconds)
  if (intFlag & RTC_INT) // Check if the wake-up cause was the RTC alarm
  {
    intFlag &= ~(RTC_INT); // Clear RTC flag
    readSensors();
    displayData("END NODE", node_temp, node_humd, node_pres, node_batteryLevel, node_x_acc, node_y_acc, node_z_acc);
    sendPeriodicData();
  }

  // 3. POWER SAVING (Low Power State) 
  sleepMode();
}
