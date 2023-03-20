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

#define DEBUG true  // Debugging mode!

// PINS
#define rxPin 18        // D18, Receive Pin (RX) for I2C buss
#define txPin 19        // D19, Transmit Pin (TX) for I2c bus
#define tdsPin A1       // A1, TDS Meter Pin, Analog Pin!

// For TDS (from basic example)
#define VREF 5.0        // Analog Reference Voltage of the ADC
#define SCOUNT 30       // Sum of Sample Point

// GLOBALS
struct TDSData {
  float tdsValue;       // The TDS value, calculated in getTDSVal().
  float voltage;        // Average voltage, calculated in getTDSVal().
};

// For fetchTDS() => Must be global to account for program timing.
int TDSBuffer[SCOUNT], TDSBufferTemp[SCOUNT];                   // Array buffers for incoming TDS data.
int TDSBufferIndex = 0, tempIndex = 0;                          // Indexes for buffers.
float avgV = 0, tdsVal = 0, temperature = 25.0;                 // Voltage, TDS, and temperature value.

SoftwareSerial BTSerial(rxPin, txPin);                  // Open Software Serial between Nano and HM-10 BT module on I2C lane.
static unsigned long programClock = millis();           // Timer/Clock of entire program, in milliseconds.
unsigned long timeCount = 1000;                              // Counter for a single second that has passed. (1000ms = 1s)
// millis() => Returns the number of milliseconds passed since the Arduino board began running the current program

// DEFINITIONS
int getMedianNum(int bArray[], int iFilterLen);
float calculateTDS(float cVal);
void fetchTDS(TDSData * TDS);

/*  ==================================================
    SETUP
    ================================================== */
void setup() {
  Serial.begin(115200);         // Open serial communication.     
  pinMode(tdsPin, INPUT);
}

/*  ==================================================
    LOOP
    ================================================== */
void loop() {
  TDSData * TDS = new TDSData;
  // TDS->tdsValue = -1;
  // TDS->voltage = -1;
  fetchTDS(TDS);  // Get the TDS value and average voltage of that reading. Approximately 840ms (or a multiple) has to pass until a value is actually received, else equal 0.
  unsigned long tdsClockIteration = millis() - programClock;   // This value represents the amount of time that this function finished executing.

  if( DEBUG ) {
    if( TDS->tdsValue != -1 && TDS->voltage != -1 ) {
      Serial.print("(");
      Serial.print(tdsClockIteration);
      Serial.print("ms): ");    
      Serial.print("TDS Value: ");
      Serial.print(TDS->tdsValue, 0);
      Serial.print("ppm\t");
      Serial.print("Voltage: ");
      Serial.print(TDS->voltage, 2);
      Serial.println("V");
    }
  }

  // This might be helpful in getting a whole number of the amount of time that has passed during the measuring process that we can send to the app...
  // If the programClock (ms) is over the first single second, increment the timeCount by another second...
  if ( programClock > timeCount ) { 
    timeCount += 1000;
  }
  delete TDS;
  programClock = millis();  // programClock is initialized in Global scope. After iteration of each loop, get the current timepoint of this "sample" iteration...
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
    "Convert voltage value to tds value."
    ================================================== */
float calculateTDS(float cVal) {
  float tds = (133.42 * cVal * cVal * cVal - 255.86 * cVal * cVal + 857.39 * cVal) * 0.5; // Calculates the ppm using voltage inputs.
  return tds;
}

/*  ==================================================
    getTDSVal :: From DFRobot Basic Example...
    Calculates the TDS reading from the meter/probe.
    Various conversions for the value, etc.
    ================================================== */
void fetchTDS(TDSData * TDS) {
  static unsigned long analogSampleTimepoint = millis();       // Get initial timepoint when we entered the loop (lookClock).
  if( millis() - analogSampleTimepoint > 40U ) {            // Every 40 milliseconds, read the analog value from the ADC. 40U is essentially the refresh rate of data fetching [Make it a variable?]
    analogSampleTimepoint = millis();                       // Assigning to a static variable? This value should be ~40ms after if-statement.
    TDSBuffer[TDSBufferIndex] = analogRead(tdsPin);         // Read the analog value and store into the buffer. ### WHERE THE RAW DATA GETS READ FROM THE DFROBOT GRAVITY TDS ###
    TDSBufferIndex++;
    if( TDSBufferIndex == SCOUNT ) { TDSBufferIndex = 0; }
  }

  static unsigned long printTimepoint = millis();             // Get initial timepoint (at this point, we know approximately that 40ms has passed).     
  if( millis() - printTimepoint > 800U ) {                    // Every 800 milliseconds, convert the analog value from the ADC. 800U might be a variable that should be changed/dynamic.
     printTimepoint = millis();                               // Assigning to a static variable? This value should be 840ms after if-statement.
     for( tempIndex = 0; tempIndex < SCOUNT; tempIndex++ ) { TDSBufferTemp[tempIndex] = TDSBuffer[tempIndex]; }
     avgV = getMedianNum(TDSBufferTemp, SCOUNT) * (float)VREF / 1024.0;           // See function description for use-case here...
     float compensationCoefficient = 1.0 + 0.02 * ( temperature - 25.0 );         // Temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
     float compensationVoltage = avgV / compensationCoefficient;                  // Temperature compensation.
     tdsVal = calculateTDS(compensationVoltage);

     TDS->tdsValue = tdsVal;
     TDS->voltage = avgV;
  }
  // At this point, approximately 840ms has passed. So, we are 260ms from reaching a full second cycle. This might be important in calculating refresh speeds.
}

