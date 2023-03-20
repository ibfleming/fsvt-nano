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

#define DEBUG true

// PINS
#define rxPin 18        // D18, Receive Pin (RX) for I2C buss
#define txPin 19        // D19, Transmit Pin (TX) for I2c bus
#define tdsPin A1       // A1, TDS Meter Pin

// For TDS (from basic example)
#define VREF 5.0        // Analog Reference Voltage of the ADC
#define SCOUNT 30       // Sum of Sample Point

// GLOBALS
struct TDSData {
  float tdsValue;       // The TDS value, calculated in getTDSVal().
  float voltage;        // Average voltage, calculated in getTDSVal().
} TDSData;

SoftwareSerial BTSerial(rxPin, txPin);        // Open Software Serial between Nano and HM-10 BT module on I2C lane.
unsigned long ProgramClock = 0;               // Timer/Clock of entire program, in milliseconds

// DEFINITIONS
int getMedianNum(int bArray[], int iFilterLen);
void fetchTDS(TDSData * TDS, unsigned long clock);
float calculateTDS(float cVal);

/*  ==================================================
    SETUP
    ================================================== */
void setup() {
  // Open serial communication.
  Serial.begin(115200);
  pinMode(tdsPin, INPUT);
}

/*  ==================================================
    LOOP
    ================================================== */
void loop() {
  // millis() => Returns the number of milliseconds passed since the Arduino board began running the current program
  unsigned long clock = millis(); // Real operations of the program begin upon loop entry. Get the initial time when this begins.
  TDSData * TDS = new TDSData;
  // TDS->tdsValue = -1;
  // TDS->voltage = -1;

  fetchTDS(TDS, clock);  // Get the TDS value and average voltage of that reading. Approximately 840ms have passed.


  if( DEBUG ) {
    Serial.print("TDS Value: ");
    Serial.print(TDS->tdsValue, 0);
    Serial.print("ppm\t");
    Serial.print("Voltage: ");
    Serial.print(TDS->voltage, 2);
    Serial.println("V");
  }

  delete TDS;
}
/*  ==================================================
    getMedianNum :: From DFRobot Basic Example...
    "Reads the analog value more stable by the median
    filtering algorithm and convert to voltage value"
    ================================================== */
int getMedianNum(int bArray[], int iFilterLen) {
    int bTab[iFilterLen], i, j, bTemp;

    for( byte i = 0; i < iFilterLen; i++ ) {
      bTab[i] = bArray[i];
    }

    for( j = 0; j < iFilterLen - 1; j++ ) {
      for( i = 0; i < iFilterLen - j - 1; i++ ) {
        if( bTab[i] > bTab[i + 1] ) {
          bTemp = bTab[i];
          bTab[i] = bTab[i + 1];
          bTab[i + 1] = bTemp;
        }
      }
    }

    if( (iFilterLen & 1) > 0 ) {
      bTemp = bTab[(iFilterLen - 1) / 2];
    }
    else {
      bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
    }

    return bTemp;
}

/*  ==================================================
    getTDSVal :: From DFRobot Basic Example...
    Calculates the TDS reading from the meter/probe.
    Various conversions for the value, etc.
    ================================================== */
void fetchTDS(TDSData * TDS, unsigned long clock) {
  int TDSBuffer[SCOUNT], TDSBufferTemp[SCOUNT];             // Array buffers for incoming TDS data.
  int TDSBufferIndex = 0, tempIndex = 0;                          // Indexes for buffers.
  float avgV = 0, tdsVal = 0, temperature = 25.0;           // Voltage, TDS, and temperature value.

  static unsigned long analogSampleTimepoint = clock;       // Get initial timepoint when we entered the loop.
  if( millis() - analogSampleTimepoint > 40U ) {            // Every 40 milliseconds, read the analog value from the ADC. 40U is essentially the refresh rate of data fetching.
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);       //read the analog value and store into the buffer
    analogBufferIndex++;
    if( analogBufferIndex == SCOUNT ) { analogBufferIndex = 0; }
  }

  static unsigned long printTimepoint = millis();             // Get initial timepoint (at this point, we know approximately that 40ms has passed)     
  if( millis() - printTimepoint > 800U ) {                    // Every 800 milliseconds, conver the analog value from the ADC.
     //printTimepoint = millis();                             // Assigning to a static variable? This value should be 840ms after if-statement. [DON'T BELIEVE THIS DOES ANYTHING...]
     for( tempIndex = 0; tempIndex < SCOUNT; tempIndex++ ) { TDSBufferTemp[tempIndex] = TDSBuffer[tempIndex]; }
     avgV = getMedianNum(TDSBufferTemp, SCOUNT) * (float)VREF / 1024.0;           // See function description for use-case here...
     float compensationCoefficient = 1.0 + 0.02 * ( temperature - 25.0 );         // Temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
     float compensationVolatge = averageVoltage / compensationCoefficient;        // Temperature compensation.
     tdsVal = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; //convert voltage value to tds value

     TDS->tdsValue = tdsVal;
     TDS->voltage = avgV;
  }

  // At this point, approximately 840ms has passed. So, we are 260ms from reaching a full second cycle.
  return tdsVal; 
}

/*  ==================================================
    getTDSVal :: From DFRobot Basic Example...
    Calculates the TDS reading from the meter/probe.
    Various conversions for the value, etc.
    ================================================== */
float calculateTDS(float cVal) {
  //(133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5;
}
