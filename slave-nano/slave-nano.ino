/*
   Slave NANO v3.3 (10/20/2023)
   by Ian B. Fleming 
   Email: ianfleming678@gmail.com

   HC12 Configuration:
   Channel: 33?
   Baud: 38400?
*/

#include <SoftwareSerial.h>

#define TDS_Pin A0
#define BATT_Pin A3
#define TdsFactor 0.5
#define DEBUG 1

// Enable debugging print-outs to Serial Monitor
#if DEBUG == 0
	#define debug(x) Serial.print(x)
	#define debugln(x) Serial.println(x)
#else
	#define debug(x)
	#define debugln(x)
#endif

enum STATE {
   RUN,
   STOP,
};

SoftwareSerial HC12(10, 11);  //HC12 TX to Pin 10 (RX), HC12 RX to Pin 11 (TX)
const size_t SIZE = 5;

/*******************************************
 * TDS Properties
 *******************************************/

float temperature = 25.0;
float aRef = 5.0;
float adcRange = 1024.0;
float kVal = 0.56;

/*
   kVal -> This is our calibration number, calibrated 
   in a standard buffer solution of 751ppm (1 tsp salt in 1 cup water).
*/

/*******************************************/

STATE state = STOP;
char tds[SIZE];
char batt[SIZE];

/*******************************************
 * Setup()
 *******************************************/

void setup() {

   Serial.begin(38400);
   while( !Serial ) {}

   HC12.begin(19200); 
   while( !HC12 ) {}

   clear_all_serials();
   pinMode(TDS_Pin, INPUT);   // TDS pin is open as input
   pinMode(BATT_Pin, INPUT);  // Battery pin is open as input

   debugln("Slave NANO (v3.3)");
   debugln("[PROGRAM READY]");
}

/*******************************************
 * Loop()
 *******************************************/

void loop() {
   switch(state) {
      case STOP:
         if( read_cmd() == 'S' ) {
            exec_start();
         }
         check_battery();
         break;

      case RUN:
         if( read_cmd() == 'E' ) {
            exec_stop();
            break;
         }
         exec();
         break;

      default: 
         break;
   }
}

/*******************************************
 * Primary Procedures
 *******************************************/

void exec() {
   // (1) Read the data
   static unsigned long read_time = millis();
   if( millis() - read_time > 250 ) {
      read_tds();                         // (1) Read TDS Probe 2           
      read_time = millis();
   }
}

void exec_stop() {
   clear_all_serials();
   memset(tds, 0, SIZE);
   memset(batt, 0, SIZE);
   state = STOP;

   // Debug
   debugln("[STOP]");
}


void exec_start() {
   clear_all_serials();
   state = RUN;

   // Debug
   debugln("[RUN]");
}

void check_battery() {
   memset(batt, 0, SIZE);
   float voltage = analogRead(BATT_Pin) * (5.0 / 1023.0);
   snprintf_P(batt, SIZE, "%u#", (unsigned int)voltage);
   HC12.write(batt);
}

/*******************************************
 * Read Operations
 *******************************************/

void read_tds() {
   memset(tds, 0, SIZE);
   snprintf(tds, SIZE, "%u$", (unsigned int)getTDS());
   HC12.write(tds);
}

char read_cmd() {
   if( HC12.available() ) {
      return HC12.read();
   }
   return 0;
}

/*******************************************
 * Serial Procedures (wipe data)
 *******************************************/

void clear_serial(Stream& s) {
   while( s.available() ) {
      s.read();
   }
}

void clear_all_serials() {
   clear_serial(Serial);
   clear_serial(HC12);
}

/*******************************************
 * Read and calculate TDS value
 *******************************************/
 
 float getTDS() {
   float analogValue = analogRead(TDS_Pin);
   float voltage = analogValue / adcRange * aRef;
   float ecVal = (133.42 * voltage * voltage * voltage - 255.86 * voltage * voltage + 857.39 * voltage) * kVal;
   float ecVal25 = ecVal / (1.0 + 0.02 * (temperature - 25.0));
   float tdsVal = ecVal25 * TdsFactor;
   return tdsVal;
 }
