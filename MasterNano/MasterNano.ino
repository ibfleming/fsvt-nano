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
#define CALIBRATE false  // Calibration mode!

// PINS
#define rxPin 18        // D18 (A4), Receive Pin (RX) for I2C buss
#define txPin 19        // D19 (A5), Transmit Pin (TX) for I2c bus
#define tdsPin A1       // A1, TDS Meter Pin, Analog Pin!

// For TDS (from basic example)
#define VREF 5.0        // Analog Reference Voltage of the ADC
#define SCOUNT 30       // Sum of Sample Point

// GLOBALS
// For fetchTDS() => Must be global to account for program timing.               
int analogBuffer[SCOUNT];                                           // Array buffer for storing analog values
int analogBufferTemp[SCOUNT];                                       // Indexes for buffers.
int analogBufferIndex = 0, copyIndex = 0;                           // Indexes for buffers.
float averageVoltage = 0, tdsValue = 0, temperature = 25.0;           // Voltage, TDS, and temperature value.

SoftwareSerial HC12(rxPin, txPin);                  // Open Software Serial between Nano and HM-10 BT module on I2C lane.
GravityTDS gravityTDS;                              // DFRobot Calibration object.

// DEFINITIONS
int getMedianNum(int bArray[], int iFilterLen);
float calculateTDS(float cVal);
void fetchTDS();

/*  ==================================================
    SETUP
    ================================================== */
void setup() {
  Serial.begin(115200);         // Open serial communication.     

  if( CALIBRATE ) {
    gravityTDS.setPin(tdsPin);
    gravityTDS.setAref(5.0);
    gravityTDS.setAdcRange(1024);
    gravityTDS.begin();
  }
  else {
    pinMode(tdsPin, INPUT);
  }
}

/*  ==================================================
    LOOP
    ================================================== */
void loop() {
  if( CALIBRATE ) {
    gravityTDS.setTemperature(temperature);
    gravityTDS.update();
    tdsValue = gravityTDS.getTdsValue();
    Serial.print("[CALIBRATION] TDS: ");
    Serial.print(tdsValue, 0);
    Serial.println("ppm");
    delay(1000);
  }
  else {
    fetchTDS();  // Get the TDS value and average voltage of that reading. Approximately 840ms (or a multiple) has to pass until a value is actually received, else equal 0.
  }
}

/*  ==================================================
    getMedianNum :: From DFRobot Basic Example...
    "Reads the analog value more stable by the median
    filtering algorithm and convert to voltage value"
    ================================================== */
int getMedianNum(int bArray[], int iFilterLen) {
   int bTab[iFilterLen];
   for ( byte i = 0; i < iFilterLen; i++ ) { bTab[i] = bArray[i]; }

   int i, j, bTemp;
   for ( j = 0; j < iFilterLen - 1; j++ ) {
      for ( i = 0; i < iFilterLen - j - 1; i++ )  {
         if ( bTab[i] > bTab[i + 1] )  {
            bTemp = bTab[i];
            bTab[i] = bTab[i + 1];
            bTab[i + 1] = bTemp;
         }
      }
   }
   if ( (iFilterLen & 1) > 0 ) { bTemp = bTab[(iFilterLen - 1) / 2]; }
   else { bTemp = ( bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1] ) / 2; }
   return bTemp;
}

/*  ==================================================
    getTDSVal :: From DFRobot Basic Example...
    Calculates the TDS reading from the meter/probe.
    "Convert voltage value to tds value."
    ================================================== */
float calculateTDS(float cVal) {
   float tds =( 133.42 * cVal * cVal * cVal - 255.86 * cVal * cVal + 857.39 * cVal) * 0.5; // Calculates the ppm using voltage inputs.
   return tds;
}

/*  ==================================================
    getTDSVal :: From DFRobot Basic Example...
    Calculates the TDS reading from the meter/probe.
    Various conversions for the value, etc.
    ================================================== */
void fetchTDS() {
   static unsigned long analogSampleTimepoint = millis();
   if( millis() - analogSampleTimepoint > 40U )    // 40ms
   {
     analogSampleTimepoint = millis();
     analogBuffer[analogBufferIndex] = analogRead(tdsPin);
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT) { analogBufferIndex = 0; }
   }   
   static unsigned long printTimepoint = millis();
   if( millis() - printTimepoint > 800U )    // 800ms
   {
      printTimepoint = millis();
      for( copyIndex = 0; copyIndex < SCOUNT; copyIndex++ ) { analogBufferTemp[copyIndex] = analogBuffer[copyIndex]; }
      averageVoltage = ( getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF ) / 1024.0;
      float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);    
      float compensationVoltage = averageVoltage/compensationCoefficient;  
      tdsValue = calculateTDS(compensationVoltage);
      
      if( DEBUG ) { 
        Serial.print("TDS: ");
        Serial.print(tdsValue, 0);
        Serial.print("ppm   ");
        Serial.print("Voltage: ");
        Serial.print(averageVoltage, 2);
        Serial.println("V");
      }
   }
}

