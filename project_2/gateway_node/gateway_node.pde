/**
 * @file Gateway_Node.pde
 * @brief Gateway Node implementation for Sensor Networks Project.
 * This node acts as a bridge between an XBee 802.15.4 network and the internet 
 * via WiFi, publishing sensor data and alarms to a ThingSpeak MQTT broker.
 * @author Alejandro Botas Bárcena
 * @author Javier Grimaldos Chavarría
 * @author Estela Mora Barba
 * @date February 2026
 */

#include <WaspSensorEvent_v30.h>
#include <WaspXBee802.h>
#include <WaspFrame.h>
#include <WaspWIFI_PRO.h>

#include <Countdown.h>
#include <FP.h>
#include <MQTTFormat.h>
#include <MQTTLogging.h>
#include <MQTTPacket.h>
#include <MQTTPublish.h>
#include <MQTTSubscribe.h>
#include <MQTTUnsubscribe.h>

// Period definition (30 seconds)
#define SAMPLING_PERIOD "00:00:00:30"

// XBee
char WASPMOTE_ID[] = "gateway";
uint8_t panID[2] = {0x12, 0x34};
uint8_t channel = 0x0F;
uint8_t encryptionMode = 0;
char encryptionKey[] = "Orange";
uint8_t error_xbee;

// WiFi
char ESSID[] = "A56 de Estela";
char PASSW[] = "4hk2fbthruumfqf";
uint8_t authtype = WPA2; 
uint8_t socket = SOCKET1;
uint16_t socket_handle = 0;
uint8_t error_wifi;

// ThingSpeak MQTT Broker Settings
char HOST[]        = "mqtt3.thingspeak.com"; 
char REMOTE_PORT[] = "1883";
char LOCAL_PORT[]  = "3000";

// ThingSpeak MQTT Credentials
char TS_USERNAME[] = "DDolNSE8FTABLx0jFDQACwQ"; 
char TS_PASSWORD[] = "IGE1kdT8q5uEcb+Ardj9LAHV"; 
char TS_CLIENTID[] = "DDolNSE8FTABLx0jFDQACwQ";

// ThingSpeak MQTT Topics
char MQTT_TOPIC_NODE[]  = "channels/3256681/publish";
char MQTT_TOPIC_GATEWAY[]  = "channels/3256684/publish";
char MQTT_TOPIC_ALARMS[]  = "channels/3256686/publish";

// Global variables for Gateway Node data
float gateway_temp, gateway_humd, gateway_pres, gateway_x_acc, gateway_y_acc, gateway_z_acc;
uint8_t gateway_batteryLevel;
int alarm_fall_gateway = 0, alarm_motion_gateway = 0;

// Global variables for End Node data
float node_temp, node_humd, node_pres, node_x_acc, node_y_acc, node_z_acc;
uint8_t node_batteryLevel;
int alarm_fall_node = 0;
int alarm_motion_node = 0;

/**
 * @brief Initializes the XBee 802.15.4 module with predefined parameters.
 * * Configures the channel, PAN ID, and encryption settings, then saves them to 
 * the module's non-volatile memory.
 */
void initXBee() {
  USB.println(F("Initializing XBee ..."));
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
 * @brief Listens for incoming XBee packets.
 * * Uses a timeout to avoid blocking local alarm monitoring. If a packet is 
 * received, it triggers the parsing logic.
 */
void receiveXBee() {
  // Switch red LED on before attempting data reception 
  Utils.setLED(LED0, LED_ON); 

  // Listen for packets (short timeout to keep the loop responsive to local alarms)
  error_xbee = xbee802.receivePacketTimeout(5000);

  if (error_xbee == 0) {
    USB.println("");
    USB.println(F("XBee message received!"));
    
    // The data is stored in xbee802._payload
    // The length is in xbee802._length
    
    parseMessage((char*)xbee802._payload);
  } else if (error_xbee == 1) {
    // Timeout - no message received, this is normal
  } else {
    USB.print(F("XBee Receive Error: "));
    USB.println(error_xbee, DEC);
  }

  // Red LED OFF
  Utils.setLED(LED0, LED_OFF);
}

/**
 * @brief Initializes the WiFi PRO module and connects to the AP.
 * @return true if connection is successful, false otherwise.
 */
bool initWiFi() {
  USB.println(F("Initializing WiFi ..."));

  // 1. Switch ON the module
  error_wifi = WIFI_PRO.ON(socket);
  if (error_wifi == 0) {
    USB.println(F("1. WiFi switched ON"));
  } else {
    USB.println(F("1. WiFi ON failed"));
    return false;
  }

  // 2. Set ESSID
  error_wifi = WIFI_PRO.setESSID(ESSID);
  if (error_wifi == 0) {
    USB.print(F("2. WiFi set ESSID OK to: "));
    USB.println(ESSID);
  } else {
    USB.println(F("2. Error setting SSID"));
    return false;
  }

  // 3. Set Password and Authentication
  error_wifi = WIFI_PRO.setPassword(authtype, PASSW);
  if (error_wifi == 0) {
    USB.println(F("3. WiFi Password set OK"));
  } else {
    USB.println(F("3. Error setting password"));
    return false;
  }

  // 4. Join Network (Connection Loop)
  USB.print(F("4. Joining network ..."));
  unsigned long start = millis();
  bool status = false;

  while ((millis() - start) < 10000) { 
    USB.print(F(".")); 
    
    if (WIFI_PRO.isConnected()) {
      status = true;
      break;
    }
    delay(2000); // Wait 2 seconds between checks
  }
  
  USB.println("");

  // 5. Check Connection and Get IP
  if (status) {
    USB.println(F("5. WiFi connected successfully"));
    if (WIFI_PRO.getIP() == 0) {
      USB.print(F("   IP Address: "));
      USB.println(WIFI_PRO._ip);
    }
    USB.println(F("WiFi Initialized"));
    return true;
  } else {
    USB.println(F("5. Failed to connect to WiFi (Timeout)"));
    return false;
  }
}

/**
 * @brief Verifies if WiFi is still connected and attempts reconnection if not.
 * @return true if connected or successfully reconnected.
 */
bool wifiCheckConnection() {
    if (WIFI_PRO.isConnected()) {
        return true;
    } else {
        USB.println(F("WiFi connection lost. Reconnecting..."));
        return initWiFi(); // Intenta reconectar antes de fallar
    }
}

/**
 * @brief Formats sensor data into a ThingSpeak-compatible string and publishes via MQTT.
 * * @param topic The MQTT topic to publish to.
 * @param temp Temperature value.
 * @param hum Humidity value.
 * @param pres Pressure value.
 * @param batt Battery percentage.
 * @param x,y,z Accelerometer values.
 * @return true if sending was successful.
 */
bool mqttPublish(char* topic, float temp, float hum, float pres, int batt, float x, float y, float z) {
  MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
  MQTTString topicString = MQTTString_initializer;
  unsigned char buf[350];
  char payload[250];
  int buflen = sizeof(buf);

  // Temperature
  int t_e = (int)temp;
  int t_d = abs((int)(temp * 100) % 100);
  // Humidity
  int h_e = (int)hum;
  int h_d = abs((int)(hum * 100) % 100);
  // Pressure
  long p_long = (long)pres;
  // Acceleration
  int x_e = (int)x; int x_d = abs((int)(x * 100) % 100);
  int y_e = (int)y; int y_d = abs((int)(y * 100) % 100);
  int z_e = (int)z; int z_d = abs((int)(z * 100) % 100);

  // 1. Payload (ThingSpeak Format)
    snprintf(payload, sizeof(payload), 
             "field1=%d.%02d&field2=%d.%02d&field3=%ld&field4=%d&field5=%d.%02d&field6=%d.%02d&field7=%d.%02d",
             t_e, t_d, h_e, h_d, p_long, batt, x_e, x_d, y_e, y_d, z_e, z_d);

    // 2. Connection Management
    if (!WIFI_PRO.isConnected()) return false;

    if (WIFI_PRO.setTCPclient(HOST, REMOTE_PORT, LOCAL_PORT) != 0) {
        WIFI_PRO.closeSocket(WIFI_PRO._socket_handle);
        return false;
    }
    socket_handle = WIFI_PRO._socket_handle;

    // 3. MQTT Packet
    data.clientID.cstring = TS_CLIENTID;
    data.username.cstring = TS_USERNAME;
    data.password.cstring = TS_PASSWORD;
    data.keepAliveInterval = 60;
    data.cleansession = 1;

    int len = MQTTSerialize_connect(buf, buflen, &data);
    topicString.cstring = topic;
    len += MQTTSerialize_publish(buf + len, buflen - len, 0, 0, 0, 0, topicString, (unsigned char*)payload, strlen(payload));

    // 4. Send and Close
    error_wifi = WIFI_PRO.send(socket_handle, buf, len);
    WIFI_PRO.closeSocket(socket_handle);
    
    // Avoid "Socket in use"
    LOCAL_PORT[3] = (LOCAL_PORT[3] == '9') ? '0' : (LOCAL_PORT[3] + 1);

    if (error_wifi == 0) {
        USB.print(F("ThingSpeak OK: ")); USB.println(payload);
    }
    
    return (error_wifi == 0);
}

/**
 * @brief Sends current alarm status flags to the dedicated ThingSpeak alarm channel.
 * * Resets alarm flags locally upon successful delivery.
 */
void sendAlarmMQTT() {
  MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
  MQTTString topicString = MQTTString_initializer;
  unsigned char buf[250];
  char payload[150];
  int buflen = sizeof(buf);

  // 1. Build Payload
  snprintf(payload, sizeof(payload), 
           "field1=%d&field2=%d&field3=%d&field4=%d",
           alarm_fall_node, 
           alarm_motion_node, 
           alarm_fall_gateway, 
           alarm_motion_gateway);

  // 2. WiFi Management
  if (!wifiCheckConnection()) {
    USB.println(F("WiFi connection failed. Aborting Alarm."));
    return;
  }

  // 3. Open TCP Socket
  WIFI_PRO.closeSocket(socket_handle); 
  delay(100);
  
  if (WIFI_PRO.setTCPclient(HOST, REMOTE_PORT, LOCAL_PORT) != 0) {
    USB.print(F("Error opening socket on port: "));
    USB.println(LOCAL_PORT);
    
    // Rotate port immediately on failure so the next attempt uses a fresh one
    LOCAL_PORT[3] = (LOCAL_PORT[3] == '9') ? '0' : (LOCAL_PORT[3] + 1);
    return;
  }
  socket_handle = WIFI_PRO._socket_handle;

  // 4. MQTT Connect
  data.clientID.cstring = TS_CLIENTID;
  data.username.cstring = TS_USERNAME;
  data.password.cstring = TS_PASSWORD;
  data.keepAliveInterval = 60;
  data.cleansession = 1;

  int len = MQTTSerialize_connect(buf, buflen, &data);

  // 5. MQTT Publish
  topicString.cstring = MQTT_TOPIC_ALARMS;
  len += MQTTSerialize_publish(buf + len, buflen - len, 0, 0, 0, 0, 
                               topicString, (unsigned char*)payload, strlen(payload));

  // 6. Send Data
  error_wifi = WIFI_PRO.send(socket_handle, buf, len);
  delay(500);

  if (error_wifi == 0) {
    USB.print(F("ALARM ThingSpeak OK: ")); USB.println(payload);
    // Solo reseteamos los flags si el envío fue exitoso
    alarm_fall_node = 0; 
    alarm_motion_node = 0;
    alarm_fall_gateway = 0; 
    alarm_motion_gateway = 0;
  } else {
    USB.println(F("Fail to send the alarm"));
  }

  // 7. Close Socket and Change Port
  WIFI_PRO.closeSocket(socket_handle);
  LOCAL_PORT[3] = (LOCAL_PORT[3] == '9') ? '0' : (LOCAL_PORT[3] + 1);
  
  delay(1000);
}

/**
 * @brief Parses incoming XBee messages and triggers actions based on content.
 * * Supports two message types:
 * 1. Alarms: Identifies "FALL" or "MOTION" events from remote nodes.
 * 2. Periodic Data: Extracts 7 sensor values using a key-value delimiter logic.
 * * @param message The raw string payload received from XBee.
 */
void parseMessage(char* message) {
  USB.print(F("Parsing: "));
  USB.println(message);

  // 1. Check if it is an ALARM message
  // Format: A=FREE FALL;ID=node_01 or A=MOTION;ID=node_01
  if (strncmp(message, "A=", 2) == 0) {
    char alarmType[10];
    sscanf(message, "A=%[^;]", alarmType);
    USB.println(F("-------------------------------"));
    USB.print(F("-- REMOTE ALARM: ")); USB.print(alarmType); USB.println(" DETECTED --");
    USB.println(F("-------------------------------"));
    USB.println("");

    // Determine which alarm it is
    if (strstr(message, "FALL")) alarm_fall_node = 1;
    if (strstr(message, "MOTION")) alarm_motion_node = 1;
    
    // Send to ThingSpeak
    sendAlarmMQTT();
  } 

  // 2. Check if it is a PERIODIC message
  // Received Format Example: T=19.45;H=63.37;P=93995.30;B=85;X=-0.40;Y=-0.13;Z=10.53;
  else if (strncmp(message, "T=", 2) == 0) {
    float t, h, p, x, y, z;
    int b;
    
    // Create a local copy because strtok modifies the original string
    char tempMessage[150];
    strncpy(tempMessage, message, sizeof(tempMessage));

    // Initialize pointer for tokenizing using '=' and ';' as delimiters
    char* ptr = strtok(tempMessage, "=;"); 
    
    int count = 0;
    while (ptr != NULL) {
      // The format is Key=Value. strtok gives us "T", then "19.45", then "H", etc.
      // We jump over the key (T, H, P...) and capture the value
      ptr = strtok(NULL, "=;"); 
      
      if (ptr != NULL) {
        // Convert ASCII to float/int based on the sequence
        if (count == 0) t = atof(ptr);      // Temp
        else if (count == 1) h = atof(ptr); // Humid
        else if (count == 2) p = atof(ptr); // Press
        else if (count == 3) b = atoi(ptr); // Batt
        else if (count == 4) x = atof(ptr); // Acc X
        else if (count == 5) y = atof(ptr); // Acc Y
        else if (count == 6) z = atof(ptr); // Acc Z
        count++;
      }
      // Move to the next Key (T, H, P...)
      ptr = strtok(NULL, "=;"); 
    }

    // Ensure we successfully parsed all 7 expected fields
    if (count == 7) {
      node_temp = t;
      node_humd = h;
      node_pres = p;
      node_batteryLevel = (uint8_t)b;
      node_x_acc = x;
      node_y_acc = y;
      node_z_acc = z;

      // Local debug output
      displayData("END NODE", node_temp, node_humd, node_pres, node_batteryLevel, node_x_acc, node_y_acc, node_z_acc);

      mqttPublish(MQTT_TOPIC_NODE, 
                  node_temp, 
                  node_humd, 
                  node_pres, 
                  (int)node_batteryLevel, 
                  node_x_acc, 
                  node_y_acc, 
                  node_z_acc);
      
    } else {
      // Log error if the number of parsed values is incorrect
      USB.print(F("Error parsing: expected 7 values, found "));
      USB.println(count);
    }
  }
}

/**
 * @brief Handles local hardware interruptions (Free Fall) from the accelerometer.
 */
void handleAlarms() {
  char alarmPayload[40];
   
  USB.println(F("-------------------------------"));
  USB.println(F("-- ALARM: FREE FALL DETECTED --"));
  USB.println(F("-------------------------------"));
  USB.println("");
      
  intFlag &= ~(ACC_INT); 

  // Send Alarm to ThingSpeak
  alarm_fall_gateway = 1;
  sendAlarmMQTT();

  // Re-arm interruptions
  PWR.clearInterruptionPin();
  ACC.ON(); 
  ACC.setFF();
  Events.attachInt();
}

/**
 * @brief Reads data from the Gateway's local sensors (Temp, Hum, Pres, Accel, Battery).
 */
void readSensors() {
  // Turn green LED on during the sensor measurement process 
  Utils.setLED(LED1, LED_ON);
    
  // Turn on the sensor board
  Events.ON();
  
  // Collect Measurements 
  gateway_temp = Events.getTemperature();
  gateway_humd = Events.getHumidity();
  gateway_pres = Events.getPressure();
  gateway_batteryLevel = PWR.getBatteryLevel();
  
  // Read Accelerometer values 
  ACC.ON();
  gateway_x_acc = (float)ACC.getX() * 0.0098;
  gateway_y_acc = (float)ACC.getY() * 0.0098;
  gateway_z_acc = (float)ACC.getZ() * 0.0098;

  // Turn green LED off after measurement
  Utils.setLED(LED1, LED_OFF);  
}

/**
 * @brief Prints formatted sensor data to the USB Serial monitor for debugging.
 */
void displayData(const char* device, float temp, float humd, float pres, uint8_t bat, float x, float y, float z) {
  USB.println("");
  USB.print(F("--------- ")); USB.print(device); USB.println(F(" PERIODICAL MEASUREMENTS ---------"));
  USB.print(F("Temperature: ")); USB.printFloat(temp, 2); USB.println(F(" C"));
  USB.print(F("Humidity: ")); USB.printFloat(humd, 2);  USB.println(F(" %"));
  USB.print(F("Pressure: ")); USB.printFloat(pres, 2); USB.println(F(" Pa"));
  USB.print(F("Battery Level: ")); USB.print(bat, DEC); USB.println(F(" %"));
  USB.print(F("Acceleration (m/s2): "));
  USB.print(F("X: ")); USB.printFloat(x, 2); 
  USB.print(F(", Y: ")); USB.printFloat(y, 2);
  USB.print(F(", Z: ")); USB.printFloat(z, 2); USB.println("");
  USB.println("---------------------------------------------------"); 
  USB.println("");
}

/**
 * @brief Standard setup function. Configures hardware modules and initial states.
 */
void setup() {
  USB.ON();
  USB.println(F("Gateway Node Initializing..."));

  // Accelerometer Setup
  ACC.ON();
  ACC.setFF();
  USB.println(F("Accelerometer Initialized"));

  Events.ON();  
  Events.attachInt(); // Enable interruptions from the board

  // XBee Setup
  frame.setID( WASPMOTE_ID );
  initXBee();

  // WiFi Setup
  initWiFi();

  // RTC Setup
  RTC.ON();
  RTC.setAlarm1(SAMPLING_PERIOD, RTC_OFFSET, RTC_ALM1_MODE1);
  
  USB.println(F("Gateway Node Setup completed"));
}

/**
 * @brief Main execution loop. Manages WiFi health, listens for XBee, and handles RTC events.
 */
void loop() {
  wifiCheckConnection();
  
  // 1. MONITOR REMOTE DATA (From End Node) 
  receiveXBee();
  
  // 2. ALARM HANDLING (Maximum Priority)
  if (intFlag & ACC_INT) handleAlarms();
  
  // 3. PERIODICAL MEASUREMENTS (Every 30 seconds)
  if (intFlag & RTC_INT) // Check if the wake-up cause was the RTC alarm
  {
    intFlag &= ~(RTC_INT); // Clear RTC flag
    readSensors();
    displayData("GATEWAY NODE", gateway_temp, gateway_humd, gateway_pres, gateway_batteryLevel, gateway_x_acc, gateway_y_acc, gateway_z_acc);

    mqttPublish(MQTT_TOPIC_GATEWAY, 
                gateway_temp, 
                gateway_humd, 
                gateway_pres, 
                (int)gateway_batteryLevel, 
                gateway_x_acc, 
                gateway_y_acc, 
                gateway_z_acc);
    
    PWR.clearInterruptionPin();
    RTC.setAlarm1(SAMPLING_PERIOD, RTC_OFFSET, RTC_ALM1_MODE1);
  }
}
