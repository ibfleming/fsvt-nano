#include <GravityTDS.h>

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

#define DEBUG false      // Debugging mode!
#define ON  HIGH
#define OFF LOW
#define ATMODE true     // Enter AT for BT!

// PINS
#define HMrxPin 10         // D4, Receive Pin (RX) 
#define HMtxPin 11         // D5, Transmit Pin (TX)
#define HCrxPin 5         // D4, Receive Pin (RX) 
#define HCtxPin 6         // D5, Transmit Pin (TX)
#define tdsPin 14       // A0 (D14), TDS Meter Pin, Analog Pin!

SoftwareSerial HM10(HMrxPin, HMtxPin);      // Open BT Serial, RX | TX
SoftwareSerial HC12(HCrxPin, HCtxPin);      // Open BT Serial, RX | TX
GravityTDS gravityTDS;                  // DFRobot Calibration object.

float tdsTemp;
char tdsValue_1[4];

unsigned long prevTime = 0;

// DEFINITIONS
void flushBuffers();
void clearSerialBuffer(SoftwareSerial& serial);
void clearHardwareBuffer(HardwareSerial& serial);
float fetchTDS();

/*  ==================================================
    SETUP
    ================================================== */
void setup() {
  Serial.begin(9600);
  HM10.begin(9600);
  HC12.begin(38400);
  clearHardwareBuffer(Serial);
  clearSerialBuffer(HM10);
  clearSerialBuffer(HC12);
}

/*  ==================================================
    LOOP
    ================================================== */
void loop() {

  if( Serial.available() ) {
    HM10.write(Serial.read());
  }

  // Send Probe 1 Data every 1000 ms.
  unsigned long currTime = millis();
  if( currTime -  prevTime >= 1000UL ) {
    prevTime = currTime;
    tdsTemp = fetchTDS();
    dtostrf(tdsTemp, 3, 0, tdsValue_1);
    HM10.print("Probe 1: ");
    HM10.write(tdsValue_1);
    HM10.println();
    delay(50);
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
  return tdsValue;
}

void clearSerialBuffer(SoftwareSerial& serial) {
  while (serial.available() > 0) {
    char incomingByte = serial.read();
  }
}

void clearHardwareBuffer(HardwareSerial& serial) {
  while (serial.available() > 0) {
    char incomingByte = serial.read();
  }
}

