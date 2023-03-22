/*  ==================================================
    MASTER ARDUINO NANO (v2)
    Hardware Modules:
    - HC-10 Bluetooth Module (Master)
    - DFRobot Gravity: Analog TDS Sensor/Meter
    Analog read-in from TDS Sensor and is send over BT to the Android app.
    RX of BT module receives TDS Sensor data and transmit to Android app.
    ================================================== */

    // Any reference to timing/clock cycling in milliseconds doesn't account for latency of the B.T. serial communcation and the Arduino's processor itself (lag, etc.).

#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <Wire.h>
#include "GravityTDS.h"

#define DEBUG true      // Debugging mode!
#define ON  HIGH
#define OFF LOW
#define ATMODE true     // Enter AT for BT!

// PINS
#define rxPin 4         // D4, Receive Pin (RX) 
#define txPin 5         // D5, Transmit Pin (TX)
#define tdsPin 14       // A0 (D14), TDS Meter Pin, Analog Pin!
#define BTSet 10        // D10, Set Pin for BT, AT Mode

SoftwareSerial BT(rxPin, txPin);      // Open BT Serial, RX | TX
GravityTDS gravityTDS;                  // DFRobot Calibration object.
float tdsValue_1 = 0;

int flag = 0;
int LED = 8;

// DEFINITIONS
float fetchTDS();

/*  ==================================================
    SETUP
    ================================================== */
void setup() {

  if( ATMODE ) {
    pinMode(BTSet, OUTPUT);
    digitalWrite(BTSet, OFF);
    Serial.begin(9600);
    BT.begin(38400);
    delay(50);
    Serial.println("Enter AT commands:");
  }
  else {
    Serial.begin(9600);         // Open serial communication.
    BT.begin(9600);             // Open BT communication.
    pinMode(LED, OUTPUT);
    gravityTDS.setPin(tdsPin);
    gravityTDS.begin();
    delay(50);
    Serial.println("Ready to connect!");
  }
}

/*  ==================================================
    LOOP
    ================================================== */
void loop() {
  if( ATMODE ) {
    if( BT.available() ) {
      Serial.write(BT.read());
    }
    if( Serial.available() ) {
      BT.write(Serial.read());
    }
  }
  else {
    if( BT.available() ) {
      flag = BT.read();
      if( flag == 1 ) {
        digitalWrite(LED, ON);
        Serial.println("LED On");
      }
      else if( flag == 0 ) {
        digitalWrite(LED, OFF);
        Serial.println("LED Off");
      }
    }
  }
}

/*  ==================================================
    fetchTDS
    Calculates the TDS reading from the meter/probe.
    Utilizes the built-in library from GravityTDS.
    ================================================== */
float fetchTDS() {
  gravityTDS.update();
  float tdsValue = gravityTDS.getTdsValue();
  if( DEBUG ) { 
    Serial.print("TDS: ");
    Serial.print(tdsValue, 0);
    Serial.println("ppm");
  }
  delay(1000);
  return tdsValue;
}

