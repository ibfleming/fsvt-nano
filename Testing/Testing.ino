/*  ==================================================
    MASTER ARDUINO NANO TESTING
    Hardware Modules:
    - HC-10 Bluetooth Module (Master)
    - DFRobot Gravity: Analog TDS Sensor/Meter
    Analog read-in from TDS Sensor and is send over BT to the Android app.
    RX of BT module receives TDS Sensor data and transmit to Android app.
    ================================================== */

#define rxPin 18        // D18 (A4), Receive Pin (RX) for I2C buss
#define txPin 19        // D19 (A5), Transmit Pin (TX) for I2c bus

#include <SoftwareSerial.h>

SoftwareSerial HC12(rxPin, txPin);

void setup() {
  Serial.begin(9600);
  HC12.begin(9600);

}

void loop() {
  while( HC12.available() ) {
    Serial.write(HC12.read());
  }
  while( Serial.available() ) {
    HC12.write(Serial.read());
  }
}
