/*  ==================================================
    SLAVE ARDUINO NANO
    Hardware Modules:
    - HC-12 Bluetooth Transceiver
    - DFRobot Gravity: Analog TDS Sensor/Meter
    Analog read-in from TDS Sensor and transmits over BT to
    Master Nano.
    ================================================== */

#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <Wire.h>
#include "GravityTDS.h"

#define DEBUG false         // Debugging mode.
#define HCrxPin 5           // D4, Receive Pin (RX) => BT TX
#define HCtxPin 6           // D5, Transmit Pin (TX) => BT RX
#define setPin 2            // D2, Set Pin for BT AT-Commands.

/*  ==================================================
    Globals
    ================================================== */
SoftwareSerial HC12(HCrxPin, HCtxPin);      // Open Software Serial, RX | TX
GravityTDS gravityTDS;                      // DFRobot object.

float tdsTemp;
char tdsValue_2[4];                         // Range is 0-9999?
byte incomingByte;
String readBuffer = "";
unsigned long prevTime = 0;

/*  ==================================================
    Function Definitions
    ================================================== */
float fetchTDS();
void clearSerialBuffer(SoftwareSerial& serial);
void clearHardwareBuffer(HardwareSerial& serial);
void applyHC12Settings(HardwareSerial& hSerial, SoftwareSerial& sSerial);

/*  ==================================================
    Setup
    ================================================== */
void setup() {
  Serial.begin(38400);
  HC12.begin(38400);
  clearHardwareBuffer(Serial);
  clearSerialBuffer(HC12);
  pinMode(setPin, OUTPUT);
  applyHC12Settings(Serial, HC12);
}

/*  ==================================================
    Loop
    ================================================== */
void loop() {
  while( Serial.available() ) {
    HC12.write(Serial.read());
  }

  unsigned long currTime = millis();
  if( currTime - prevTime >= 1000UL ) {
    prevTime = currTime;
    tdsTemp = fetchTDS();
    dtostrf(tdsTemp, 3, 0, tdsValue_2);
    HC12.print("Probe 2: ");
    HC12.write(tdsValue_2);
    HC12.println();
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

/*  ==================================================
    clearSerialBuffer
    ================================================== */
void clearSerialBuffer(SoftwareSerial& serial) {
  while (serial.available() > 0) {
    char incomingByte = serial.read();
  }
}

/*  ==================================================
    clearHardwareBuffer
    ================================================== */
void clearHardwareBuffer(HardwareSerial& serial) {
  while (serial.available() > 0) {
    char incomingByte = serial.read();
  }
}

/*  ==================================================
    applyHC12Settings - Apply these on setup. Values aren't
    saved in memory.
    ================================================== */
void applyHC12Settings(HardwareSerial& hSerial, SoftwareSerial& sSerial) {
  digitalWrite(setPin, LOW);
  delay(50);
  HC12.print("AT+C033");
  delay(50);
  HC12.print("AT+B38400");
  delay(50);
  HC12.print("AT+V");
  delay(50);
  Serial.println("[Second Nano, HC-12] AT Command Output:");
  while( HC12.available() ) {   
    Serial.write(HC12.read());
  }
  Serial.println();
  digitalWrite(setPin, HIGH);
  delay(50);
  clearHardwareBuffer(hSerial);
  clearSerialBuffer(sSerial);
  delay(50);
}
