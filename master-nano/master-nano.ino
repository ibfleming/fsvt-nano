/*
   Master NANO v3.3 (10/20/2023)
   by Ian B. Fleming 
   Email: ianfleming678@gmail.com

   HM10 Configuration:
   Name: FSVT-BLE
   Baud: 38400 

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

SoftwareSerial HM10(5, 6);    //HM10 TX to Pin 5 (RX), HM10 RX to Pin 6 (TX)
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
char tds_1[SIZE];
char tds_2[SIZE];
char batt_1[SIZE];
char batt_2[SIZE];

/*******************************************
 * Setup()
 *******************************************/

void setup() {

   Serial.begin(38400);
   while( !Serial ) {}

   HC12.begin(19200); 
   while( !HC12 ) {}

   HM10.begin(38400);
   while( !HM10 ) {}

   clear_all_serials();
   pinMode(TDS_Pin, INPUT);   // TDS pin is open as input
   pinMode(BATT_Pin, INPUT);  // Battery pin is open as input

   debugln("Master NANO (v3.3)");
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

      read_master_tds();                  // (1) Read TDS Probe 1
      read_slave_tds(read_time);          // (2) Read TDS Probe 2 via HC12 transmission
      write_data();                       // (3) Send to application
      read_time = millis();

      // Debug
      debug("P1: "); debug(tds_1); debug(" P2: "); debugln(tds_2);
   }
}

void exec_stop() {
   acknowledge();                         // (1) Send acknowledgement to application
   write_stop();                          // (2) Write stop command to HC12
   clear_all_serials();                   // (3) Clear all the serials
   memset(tds_1, 0, SIZE);                 // (4) Wipe TDS buffers
   memset(tds_2, 0, SIZE);
   state = STOP;

   // Debug     
   debugln("[STOP]");
}

void exec_start() {
   write_start();                         // (1) Write start command to HC12
   clear_all_serials();                   // (2) Clear all the serials
   memset(batt_1, 0, SIZE);
   memset(batt_2, 0, SIZE);
   state = RUN;

   // Debug
   debugln("[RUN]");
}

void check_battery() {
   // (1) Read the battery levels
   static unsigned long battery_time = millis();
   if( millis() - battery_time > 500 ) {

      read_master_batt();
      read_slave_batt();
      write_battery();
      battery_time = millis();

      // Debug
      debug("B1: "); debug(batt_1); debug(" B2: "); debugln(batt_2);
   }
}

/*******************************************
 * Write Operations
 *******************************************/

void acknowledge() {
   delay(50);
   HM10.write('A');
   debugln("# Sent acknowledgement #");
}

void write_stop() {
   // Repeat write three times -> ensures HC12 has received w/o acknowledge
   for( int i = 0; i < 3; i++ ) {
      delay(10);
      HC12.write('E');
   }
}

void write_start() {
   HC12.write('S');
}

void write_data() {
   char msg[2 * SIZE + 1];
   snprintf(msg, sizeof(msg), "%s:%s", tds_1, tds_2);
   HM10.write(msg);
}

void write_battery() {
   char msg[2 * SIZE +1];
   snprintf(msg, sizeof(msg), "%sV%s", batt_1, batt_2);
   HM10.write(msg);
}

/*******************************************
 * Read Operations
 *******************************************/

void read_master_tds() {
   memset(tds_1, 0, SIZE);
   snprintf(tds_1, SIZE, "%u", (unsigned int)getTDS());
}

void read_master_batt() {
   memset(batt_1, 0, SIZE);
   float voltage = analogRead(BATT_Pin) * (5.0 / 1023.0);
   snprintf_P(batt_1, SIZE, "%u", (unsigned int)voltage);
}

void read_slave_tds(unsigned long read_time) {
   size_t idx = 0;
   memset(tds_2, 0, SIZE);

   HC12.listen();
   delay(250);
   while(HC12.available() && idx < (SIZE - 1)) {
      char c = HC12.read();
      if( c == '$' ) {
         tds_2[idx] = '\0';
         return;
      }
      if( isdigit(c) ) {
         tds_2[idx++] = c;
      }
   }

   // IF nothing is read or artifacts
   if( idx == 0 ) {
      debugln("# Invalid TDS2 #");
      tds_2[idx++] = '0'; tds_2[idx] = '\0'; // TDS will default to 0.
      return;
   }
}

void read_slave_batt() {
   size_t idx = 0;

   HC12.listen();
   delay(250);
   while(HC12.available() && idx < (SIZE - 1)) {
      char c = HC12.read();
      if( c == '#' ) {
         batt_2[idx] = '\0';
         return;
      }
      if( isdigit(c) ) {
         batt_2[idx++] = c;
      }
   }

   // IF nothing is read or artifacts
   if( idx == 0 ) {
      debugln("# Invalid Battery (Probe 2) ");
      // batt_2[idx++] = '0'; batt_2[idx] = '\0';
      // Use the last battery level? Don't memset batt_2...
      return;
   }
}

char read_cmd() {
   HM10.listen();
   if( HM10.available() ) {
      return HM10.read();
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
   clear_serial(HM10);
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
