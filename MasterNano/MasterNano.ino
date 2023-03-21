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

// PINS
#define rxPin 18        // D18 (A4), Receive Pin (RX) for I2C buss
#define txPin 19        // D19 (A5), Transmit Pin (TX) for I2c bus
#define tdsPin A1       // A1, TDS Meter Pin, Analog Pin!

SoftwareSerial HC12(rxPin, txPin);                  // Open Software Serial between Nano and HM-10 BT module on I2C lane.
GravityTDS gravityTDS;                              // DFRobot Calibration object.
float tdsValue_1 = 0;

// DEFINITIONS
void fetchTDS();

/*  ==================================================
    SETUP
    ================================================== */
void setup() {
  Serial.begin(115200);         // Open serial communication.
  gravityTDS.setPin(tdsPin);
  gravityTDS.begin();
}

/*  ==================================================
    LOOP
    ================================================== */
void loop() {
  tdsValue_1 = fetchTDS();  // Get the TDS value and average voltage of that reading.
}

/*  ==================================================
    fetchTDS
    Calculates the TDS reading from the meter/probe.
    Utilizes the built-in library from GravityTDS.
    ================================================== */
void fetchTDS() {
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

