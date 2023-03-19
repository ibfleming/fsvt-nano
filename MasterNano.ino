/*  ==================================================
    MASTER ARDUINO NANO
    Hardware Modules:
    - HC-10 Bluetooth Module (Master)
    - DFRobot Gravity: Analog TDS Sensor/Meter
    Analog read-in from TDS Sensor and is send over BT to the Android app.
    RX of BT module receives TDS Sensor data and transmit to Android app.
    ================================================== */

#include <SoftwareSerial.h>
#include <EEPROM.h>
#include "GravityTDS.h"

#define TDS_PIN A0
#define LATENCY 1000

SoftwareSerial BTSerial(2, 3);
GravityTDS gTDS;

float temperature = 25;   // May not need...
float masterTDS = 0;
float slaveTDS = 0;

/*  ==================================================
    SETUP
    ================================================== */
void setup() {
  // Start the serial communication with the BT module.
  BTSerial.begin(9600);
  
  // Start the serial communcation with the Gravity TDS.
  Serial.begin(115200);
  gTDS.setPin(TDS_PIN);
  gTDS.setAref(5.0);
  gTDS.setAdcRange(1024);
  gTDS.setTemperature(temperature);
  gTDS.begin();
  
  // Run the AT Commands for Master device:
  // AT+ROLE
  // AT+CMODE
  // AT+CON <MAC of HC-06>
  
  BTSerial.listen();
}

/*  ==================================================
    LOOP
    ================================================== */
void loop() {
  gTDS.update();                
  masterTDS = gTDS.getTdsValue();
  
  // Receive the data from the Slave Nano.
  while( BTSerial.available() ) {
    BTSerial.readBytes((byte *)&slaveTDS, sizeof(float) );
    break;
  }
  
  BTSerial.print("Master TDS: ")
  BTSerial.print(masterTDS, 0);
  BTSerial.println(" ppm");
  
  BTSerial.print("Slave TDS: ")
  BTSerial.print(slaveTDS, 0)
  BTSerial.println(" ppm");
  
  delay(LATENCY); // 1 sec delay
}
