/*
 * Project #2 - Sensor Networks
 * Gateway Node Implementation
 * Authors: Alejandro Botas Bárcena, Javier Grimaldos Chavarría, Estela Mora Barba
 */

#include <WaspSensorEvent_v30.h> // Sensor Measurements
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

// MQTT Broker Settings [cite: 40, 62]
char HOST[]        = "mqtt.mucs.es"; 
char REMOTE_PORT[] = "1883";
char LOCAL_PORT[]  = "3000";
char MQTT_TOPIC[]  = "orange/channels/YOUR_CHANNEL_ID/publish";

char WASPMOTE_ID[] = "gateway_01";
uint8_t socket = SOCKET0;
uint16_t socket_handle = 0;
uint8_t error_wifi;

char mqtt_payload[150];

// Global variables for sensor data
float gateway_temp, gateway_humd, gateway_pres;
uint8_t gateway_batteryLevel;
int gateway_x_acc, gateway_y_acc, gateway_z_acc;

pirSensorClass pir(SOCKET_1);
uint8_t gateway_motion = 0;

// Variable to store XBee reception status
uint8_t error_xbee;

// Function to publish data via MQTT
void publishMQTT(char* payload_str) {
  uint8_t error_wifi = WIFI_PRO.ON(socket);
  if (error_wifi == 0) {
    if (WIFI_PRO.isConnected()) {
      error_wifi = WIFI_PRO.setTCPclient(HOST, REMOTE_PORT, LOCAL_PORT);
      if (error_wifi == 0) {
        socket_handle = WIFI_PRO._socket_handle;
        
        // MQTT Serialization
        MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
        MQTTString topicString = MQTTString_initializer;
        unsigned char buf[250];
        int buflen = sizeof(buf);
        
        data.clientID.cstring = (char*)"gateway_orange";
        data.keepAliveInterval = 30;
        data.cleansession = 1;
        
        int len = MQTTSerialize_connect(buf, buflen, &data);
        topicString.cstring = MQTT_TOPIC;
        
        len += MQTTSerialize_publish(buf + len, buflen - len, 0, 0, 0, 0, 
                                     topicString, (unsigned char*)payload_str, strlen(payload_str));
        len += MQTTSerialize_disconnect(buf + len, buflen - len);
        
        WIFI_PRO.send(socket_handle, buf, len);
        WIFI_PRO.closeSocket(socket_handle);
        USB.println(F("MQTT Publish OK"));
      }
    }
    WIFI_PRO.OFF(socket);
  }
}

void setup() {
  USB.ON();
  USB.println(F("Gateway Node Initializing..."));

  //////////////////////////////////////////////////////////////////////////////
  // ACCELEROMETER SETUP
  //////////////////////////////////////////////////////////////////////////////
  ACC.ON();
  ACC.setFF();

  //////////////////////////////////////////////////////////////////////////////
  // PIR SETUP
  //////////////////////////////////////////////////////////////////////////////
  Events.ON();
    
  // Wait for PIR signal stabilization
  gateway_motion = pir.readPirSensor();
  USB.println(F("Waiting for PIR stabilization ..."));
  
  while (gateway_motion == 1)
  {
    USB.println(F("..."));
    delay(1000);
    gateway_motion = pir.readPirSensor();    
  }
  
  // Enable interruptions from the board
  Events.attachInt();

  //////////////////////////////////////////////////////////////////////////////
  // XBEE SETUP
  //////////////////////////////////////////////////////////////////////////////
  xbee802.ON();
  frame.setID(WASPMOTE_ID);

  //////////////////////////////////////////////////////////////////////////////
  // RTC SETUP
  //////////////////////////////////////////////////////////////////////////////
  RTC.ON();
  RTC.setAlarm1(SAMPLING_PERIOD, RTC_OFFSET, RTC_ALM1_MODE1);
  
  USB.println(F("End Node Setup completed"));
}

void loop() {
  //////////////////////////////////////////////////////////////////////////////
  // 1. MONITOR REMOTE DATA (From End Node) 
  //////////////////////////////////////////////////////////////////////////////

  // Red LED ON during data processing 
  Utils.setLED(LED1, LED_ON); 
  
  // Listen for packets (short timeout to keep the loop responsive to local alarms)
  error_xbee = xbee802.receivePacketTimeout(2000);

  if (error_xbee == 0) {
    // Variables to store parsed values from remote node
    char* token;
    char remote_payload[xbee802._length + 1];
    
    // Remote sensor variables
    // Remote sensor variables
    float r_temp = 0.0, r_hum = 0.0, r_pres = 0.0;
    int r_bat = 0, r_x = 0, r_y = 0, r_z = 0, alarm_value = 0;
    char r_alarms[30];
    bool remote_fall = false, remote_pir = false;

    // Copy payload to a string for parsing
    memcpy(remote_payload, xbee802._payload, xbee802._length);
    remote_payload[xbee802._length] = '\0';

    // 1. Manual Parsing Logic (strtok)
    // The frame looks like: <=>#node_01#TCA:25.50#HUMA:50.0#...
    token = strtok(remote_payload, "#"); // Skip the header <=> and ID node_01
    token = strtok(NULL, "#");           // First sensor TCA
    
    while (token != NULL) {
      if (strstr(token, "TCA:")) r_temp = atof(token + 4);
      else if (strstr(token, "HUMA:")) r_hum = atof(token + 5);
      else if (strstr(token, "PS:"))   r_pres = atof(token + 3);
      else if (strstr(token, "BAT:"))  r_bat = atoi(token + 4);
      else if (strstr(token, "ACC:")) sscanf(token + 4, "%d;%d;%d", &r_x, &r_y, &r_z);
      else if (strstr(token, "STR:FALL_ALARM")) remote_fall = true;
      else if (strstr(token, "STR:PIR_ALARM"))  remote_pir = true;
      token = strtok(NULL, "#");
    }

    // Determine the final consolidated alarm string 
    if (remote_fall && remote_pir) {
        strcpy(r_alarms, "FALL and MOTION");
        alarm_value = 3; 
    } else if (remote_fall) {
        strcpy(r_alarms, "FALL");
        alarm_value = 2; 
    } else if (remote_pir) {
        strcpy(r_alarms, "MOTION");
        alarm_value = 1; 
    } else {
        strcpy(r_alarms, "NONE");
        alarm_value = 0; 
    }

    // 2. Display End Node Measurements
    USB.println(F("--------- END NODE PERIODICAL MEASUREMENTS ---------"));
    USB.print(F("Alarms: ")); USB.println(r_alarms);
    USB.print(F("Temperature: ")); USB.printFloat(r_temp, 2); USB.println(F(" C"));
    USB.print(F("Humidity: ")); USB.printFloat(r_hum, 1);  USB.println(F(" %"));
    USB.print(F("Pressure: ")); USB.printFloat(r_pres, 2); USB.println(F(" Pa"));
    USB.print(F("Battery Level: ")); USB.print(r_bat, DEC); USB.println(F(" %"));
    USB.print(F("Acceleration (m/s2): "));
    USB.print(F("X: ")); USB.print(r_x); 
    USB.print(F(", Y: ")); USB.print(r_y);
    USB.print(F(", Z: ")); USB.println(r_z);
    USB.println(F("---------------------------------------------------"));
    USB.println("");

    // 3. Send to Cloud (ThingSpeak MQTT Format)
    snprintf(mqtt_payload, sizeof(mqtt_payload), 
             "field1=%.2f&field2=%.2f&field3=%.2f&field4=%d&field5=%d&field6=%d&field7=%d&field8=%d", 
             r_temp, r_hum, r_pres, r_bat, r_x, r_y, r_z, alarm_value);
    publishMQTT(mqtt_payload);
  }

  // Red LED OFF
  Utils.setLED(LED1, LED_OFF);
  
  //////////////////////////////////////////////////////////////////////////////
  // 2. ALARM HANDLING (Maximum Priority)
  //////////////////////////////////////////////////////////////////////////////
  
  // Check if the wake-up cause was a sensor interruption
  if (intFlag & (ACC_INT | SENS_INT))
  {
    bool fallDetected = false;
    bool motionDetected = false;

    // Detect Free-Fall interruption
    if (intFlag & ACC_INT)
    {
      fallDetected = true;
      
      USB.println(F("-------------------------------"));
      USB.println(F("-- ALARM: FREE FALL DETECTED --"));
      USB.println(F("-------------------------------"));
      USB.println("");

      publishMQTT((char*)"field8=1"); // Using field 8 for fall alarms
      
      intFlag &= ~(ACC_INT); 
    }

    // Detect Presence (PIR) interruption
    if (intFlag & SENS_INT)
    {
      Events.detachInt();
      Events.loadInt();
      if (pir.getInt())
      {
        motionDetected = true;
        
        USB.println(F("----------------------------"));
        USB.println(F("-- ALARM: MOTION DETECTED --"));
        USB.println(F("----------------------------"));
        USB.println("");

        publishMQTT((char*)"field7=1"); // Using field 7 for motion alarms
      }
      intFlag &= ~(SENS_INT);
    }

    // Re-arm interruptions before going back to sleep
    ACC.ON(); 
    ACC.setFF();
    Events.attachInt();
  }
  
  //////////////////////////////////////////////////////////////////////////////
  // 3. PERIODICAL MEASUREMENTS (Every 30 seconds)
  //////////////////////////////////////////////////////////////////////////////
  
  // Check if the wake-up cause was the RTC alarm
  if (intFlag & RTC_INT)
  {
    intFlag &= ~(RTC_INT); // Clear RTC flag

    // Turn green LED on during the sensor measurement process 
    Utils.setLED(LED0, LED_ON);
    
    // Turn on the sensor board
    Events.ON();
  
    // Collect measurements 
    gateway_temp = Events.getTemperature();
    gateway_humd = Events.getHumidity();
    gateway_pres = Events.getPressure();
    gateway_batteryLevel = PWR.getBatteryLevel();
  
    // Read Accelerometer values 
    ACC.ON();
    gateway_x_acc = ACC.getX();
    gateway_y_acc = ACC.getY();
    gateway_z_acc = ACC.getZ();

    // Display values on the serial interface (USB)
    USB.println("--------- GATEWAY NODE PERIODICAL MEASUREMENTS ---------");
    USB.print(F("Temperature: ")); USB.printFloat(gateway_temp, 2); USB.println(F(" C"));
    USB.print(F("Humidity: ")); USB.printFloat(gateway_humd, 1);  USB.println(F(" %"));
    USB.print(F("Pressure: ")); USB.printFloat(gateway_pres, 2); USB.println(F(" Pa"));
    USB.print(F("Battery Level: ")); USB.print(gateway_batteryLevel, DEC); USB.println(F(" %"));
    USB.print(F("Acceleration (m/s2): "));
    USB.print(F("X: ")); USB.print(gateway_x_acc); 
    USB.print(F(", Y: ")); USB.print(gateway_y_acc);
    USB.print(F(", Z: ")); USB.println(gateway_z_acc);
    USB.println("---------------------------------------------------------"); USB.println("");

    // Publish local data to ThingSpeak
    snprintf(mqtt_payload, sizeof(mqtt_payload), 
             "field1=%.2f&field2=%.2f&field3=%.2f&field4=%d&field5=%d&field6=%d&field7=%d&field8=%d", 
             gateway_temp, gateway_humd, gateway_pres, gateway_batteryLevel, gateway_x_acc, gateway_y_acc, 
             gateway_z_acc, alarm_value);
    publishMQTT(mqtt_payload);
  
    // Turn green LED off after measurement
    Utils.setLED(LED0, LED_OFF);  

    RTC.setAlarm1(SAMPLING_PERIOD, RTC_OFFSET, RTC_ALM1_MODE1);
  }

  PWR.clearInterruptionPin();
}
