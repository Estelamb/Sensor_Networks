/*
 * Project #2 - Sensor Networks
 * Phase: 2.1 End Node Implementation
 * Authors: Alejandro Botas Bárcena, Javier Grimaldos Chavarría, Estela Mora Barba
 */

#include <WaspSensorEvent_v30.h> // Sensor Measurements
#include <WaspXBee802.h>
#include <WaspFrame.h>

// Period definition (30 seconds)
#define SAMPLING_PERIOD "00:00:00:30"

// Destination MAC address (Gateway)
char RX_ADDRESS[] = "0013A200406E5DC5";
char WASPMOTE_ID[] = "node_01";

// Global variables for sensor data
float node_temp, node_humd, node_pres;
uint8_t node_batteryLevel;
int node_x_acc, node_y_acc, node_z_acc;

pirSensorClass pir(SOCKET_1);
uint8_t node_motion = 0;

// Variable to store XBee reception status
uint8_t error;

void setup() {
  USB.ON();
  USB.println(F("End Node Initializing..."));

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
  node_motion = pir.readPirSensor();
  USB.println(F("Waiting for PIR stabilization ..."));
  
  while (node_motion == 1)
  {
    USB.println(F("..."));
    delay(1000);
    node_motion = pir.readPirSensor();    
  }
  
  // Enable interruptions from the board
  Events.attachInt();

  //////////////////////////////////////////////////////////////////////////////
  // XBEE SETUP
  //////////////////////////////////////////////////////////////////////////////
  xbee802.ON();
  frame.setID(WASPMOTE_ID);
  
  USB.println(F("End Node Setup completed"));
}

void loop() {
  //////////////////////////////////////////////////////////////////////////////
  // 1. ALARM HANDLING (Maximum Priority)
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
      }
      intFlag &= ~(SENS_INT);
    }

    // TRANSMIT ALARM: Send as soon as possible
    if (fallDetected || motionDetected)
    {
      // Switch red LED on before attempting data transmission 
      Utils.setLED(LED1, LED_ON); 
      
      frame.createFrame(ASCII);
      
      // Prioritize: If several alarms occur, send them in a single message
      if (fallDetected) frame.addSensor(SENSOR_STR, (char*)"FALL_ALARM");
      if (motionDetected) frame.addSensor(SENSOR_STR, (char*)"PIR_ALARM");
      
      // Send XBee packet to Gateway
      error = xbee802.send(RX_ADDRESS, frame.buffer, frame.length);
      
      // Switch red LED off when transmission is completed 
      Utils.setLED(LED1, LED_OFF); 
    }

    // Re-arm interruptions before going back to sleep
    ACC.ON(); 
    ACC.setFF();
    Events.attachInt();
  }
  
  //////////////////////////////////////////////////////////////////////////////
  // 2. PERIODICAL MEASUREMENTS (Every 30 seconds)
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
    node_temp = Events.getTemperature();
    node_humd = Events.getHumidity();
    node_pres = Events.getPressure();
    node_batteryLevel = PWR.getBatteryLevel();
  
    // Read Accelerometer values 
    ACC.ON();
    node_x_acc = ACC.getX();
    node_y_acc = ACC.getY();
    node_z_acc = ACC.getZ();

    // Display values on the serial interface (USB)
    USB.println("--------- END NODE PERIODICAL MEASUREMENTS ---------");
    USB.print(F("Temperature: ")); USB.printFloat(node_temp, 2); USB.println(F(" C"));
    USB.print(F("Humidity: ")); USB.printFloat(node_humd, 1);  USB.println(F(" %"));
    USB.print(F("Pressure: ")); USB.printFloat(node_pres, 2); USB.println(F(" Pa"));
    USB.print(F("Battery Level: ")); USB.print(node_batteryLevel, DEC); USB.println(F(" %"));
    USB.print(F("Acceleration (m/s2): "));
    USB.print(F("X: ")); USB.print(node_x_acc); 
    USB.print(F(", Y: ")); USB.print(node_y_acc);
    USB.print(F(", Z: ")); USB.println(node_z_acc);
    USB.println("---------------------------------------------------"); USB.println("");
  
    // Turn green LED off after measurement
    Utils.setLED(LED0, LED_OFF); 

    // PERIODIC TRANSMISSION
    // Switch red LED on before attempting data transmission 
    Utils.setLED(LED1, LED_ON); 
    
    frame.createFrame(ASCII);
    frame.addSensor(SENSOR_IN_TEMP, node_temp);
    frame.addSensor(SENSOR_HUMA, node_humd);
    frame.addSensor(SENSOR_PS, node_pres);
    frame.addSensor(SENSOR_BAT, node_batteryLevel);
    frame.addSensor(SENSOR_ACC, node_x_acc, node_y_acc, node_z_acc);
    
    error = xbee802.send(RX_ADDRESS, frame.buffer, frame.length);
    
    // Switch red LED off when transmission is completed 
    Utils.setLED(LED1, LED_OFF); 
  }

  //////////////////////////////////////////////////////////////////////////////
  // 3. POWER SAVING (Low Power State) 
  //////////////////////////////////////////////////////////////////////////////
  
  USB.println(F("Entering low power state..."));
  
  // Turn off communication module to conserve energy 
  xbee802.OFF(); 
  
  // Enter Deep Sleep for 30s or until an interruption occurs
  // SENSOR_ON keeps the board powered for PIR and ACC interruptions
  PWR.deepSleep(SAMPLING_PERIOD, RTC_OFFSET, RTC_ALM1_MODE1, SENSOR_ON);
  
  // After waking up, restore necessary modules
  USB.ON();
  xbee802.ON();
  PWR.clearInterruptionPin();
}
