/*  ==================================================
    SLAVE ARDUINO NANO
    Hardware Modules:
    - HC-06 Bluetooth Module (Slave)
    - DFRobot Gravity: Analog TDS Sensor/Meter
    Analog read-in from TDS Sensor and transmits over BT to
    Master Nano.
    ================================================== */

#include <SoftwareSerial.h>
#include <EEPROM.h>
#include "GravityTDS.h"

#define TDS_PIN A0
#define LATENCY 1000

SoftwareSerial BTSerial(2, 3);
GravityTDS gTDS;

float temperature = 25;   // May not need...
float tdsVal = 0;

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
  
  gTDS.begin();
  
  // Wait for the BT connection from the Master.
  while( !BTSerial.available() ) {
    delay(10);
  }
  
  // Connection between master/slave is established!
  BTSerial.print("Connection established!\n");
}

/*  ==================================================
    LOOP
    ================================================== */
void loop() {
  gTDS.update();
  tdsVal = gTDS.getTdsValue();
  
  /* Print Slave TDS prior to transmitting to BT
  Serial.print("Slave TDS: ");
  Serial.print(tdsValue, 0);
  Serial.println("ppm");
  */
  
  // Send the TDS value to the Master Nano over BT
  BTSerial.write((byte *)&tdsVal, sizeof(float));
  
  delay(LATENCY); // 1 sec delay
}
