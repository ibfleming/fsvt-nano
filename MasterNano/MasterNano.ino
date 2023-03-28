/*  ==================================================
    MASTER ARDUINO NANO (v2)
    Hardware Modules:
    - HM-10 Bluetooth Module (Master)
    - HC-12 Bluetooth Transceiver
    - DFRobot Gravity: Analog TDS Sensor/Meter
    Analog read-in from TDS Sensor and is send over BT to the Android app.
    RX of BT module receives TDS Sensor data and transmit to Android app.
    ================================================== */

#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <Wire.h>
#include "GravityTDS.h"

#define DEBUG false         // Debugging mode!
#define HMrxPin 10          // D10, Receive Pin (RX) => BT TX
#define HMtxPin 11          // D11, Transmit Pin (TX) => BT RX
#define HCrxPin 5           // D4, Receive Pin (RX) => BT TX
#define HCtxPin 6           // D5, Transmit Pin (TX) => BT RX
#define setPin 2            // D2, Set Pin for BT AT-Commands.

/*  ==================================================
    Globals
    ================================================== */
SoftwareSerial HM10(HMrxPin, HMtxPin);      // Open Software Serial for HM10, RX | TX
SoftwareSerial HC12(HCrxPin, HCtxPin);      // Open Software Serial for HC12, RX | TX
GravityTDS gravityTDS;                      // DFRobot GravityTDS object.

float tdsTemp;
char tdsValue_1[4];
unsigned long prevTime = 0;

/*  ==================================================
    Function Definitions
    ================================================== */
float fetchTDS();
void clearSoftwareBuffer(SoftwareSerial& serial);
void clearHardwareBuffer(HardwareSerial& serial);
void applyHC12Settings(HardwareSerial& Serial, SoftwareSerial& HC12);

/*  ==================================================
    Setup
    ================================================== */
void setup() {
  Serial.begin(9600);
  //HM10.begin(9600);
  HC12.begin(9600);
  pinMode(setPin, OUTPUT);
  clearHardwareBuffer(Serial);
  clearSoftwareBuffer(HC12);
  //clearSoftwareSerial(HM10)
  applyHC12Settings(Serial, HC12);
}

/*  ==================================================
    Loop
    ================================================== */
void loop() {

  if( HC12.available() ) {
    Serial.write(HC12.read());
  }

  /*
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
  */

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
    clearSoftwareBuffer
    ================================================== */
void clearSoftwareBuffer(SoftwareSerial& serial) {
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
    applyHC12Settings
    ================================================== */
void applyHC12Settings(HardwareSerial& Serial, SoftwareSerial& HC12) {
  digitalWrite(setPin, LOW);
  delay(50);
  HC12.print("AT+V");
  delay(250);
  HC12.print("AT+DEFAULT");
  delay(250);

  Serial.println("[MASTER] AT Command Output:");
  while( HC12.available() ) {   
    Serial.write(HC12.read());
  }
  Serial.println();

  digitalWrite(setPin, HIGH);
  delay(100);

  clearHardwareBuffer(Serial);
  clearSoftwareBuffer(HC12);
}