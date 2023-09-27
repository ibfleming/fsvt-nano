#include <SoftwareSerial.h>
#include <Wire.h>
#include <GravityTDS.h>

#define TDS_Pin A0
#define DEBUG 0

#if DEBUG == 0
	#define debug(x) Serial.print(x)
	#define debugln(x) Serial.println(x)
#else
	#define debug(x)
	#define debugln(x)
#endif

typedef enum STATE {
	SLEEP,
	RUN,
} STATE;

GravityTDS TDS;
SoftwareSerial HC12(10, 11);	//HC12 TX, HC12 RX
SoftwareSerial HM10(5, 6);		//HM10 TX, HM10 RX

/* ------------------------------------------------- */

const size_t SIZE = 4;
STATE program_state = SLEEP;
char tds_1[SIZE];
char tds_2[SIZE];

/* ------------------------------------------------- */

void setup() {
	Serial.begin(9600); 
  while( !Serial ) {}
  clear_serials(Serial);
	debugln("<MASTER SERIAL READY>");

	HC12.begin(19200); 
  while( !HC12 ) {}
  clear_serials(HC12);
	debugln("<HC12 READY>");

	HM10.begin(9600);
  while( !HM10 ) {}
  clear_serials(HM10);
	debugln("<HM10 READY>");

	TDS.setPin(TDS_Pin);
	TDS.begin();
}

void loop() {
  if( program_state == SLEEP ) {
    if( fetch_command() == 'S' ) {
      HC12.write('S');
      program_state = RUN;
      run();
    }
  }
}

/* ------------------------------------------------- */

void run() {
  debugln("### PROGRAM RUNNING ###");
  while( program_state == RUN ) {

    static unsigned long fetch_millis = millis();
    if( millis() - fetch_millis > 500 ) {
      fetch_tds();
      fetch_millis = millis();
    }

    if( fetch_command() == 'E' ) {
      HC12.write('E');
      clear_serials(Serial);
      clear_serials(HC12);
      clear_serials(HM10);
      program_state = SLEEP;
      debugln("### PROGRAM STOPPING ###");    
    }

  }
}

/* ------------------------------------------------- */

void fetch_tds() {
  // 1. Get Master TDS
  get_master_tds();
  // 2. Get Slave TDS
  get_slave_tds();
  // 3. Combine values and send to APP
  send_to_app();
  // DEBUG
  debug("P1: ");
  debugln(tds_1);
  debug("P2: ");
  debugln(tds_2);
}

/* ------------------------------------------------- */

void get_master_tds() {
  TDS.update();
  snprintf(tds_1, SIZE, "%u", (size_t)TDS.getTdsValue());
}

/* ------------------------------------------------- */

// Might need to reimplement, String objects aren't particularly reliable in Arduino
void get_slave_tds() {
  HC12.listen();
  while( !HC12.isListening() ) {} 
  
  delay(250);
  if( HC12.available() ) {
    size_t idx = 0;

    while( idx < SIZE - 1 ) {
      char c = HC12.read();
      if( c == '$' ) {
        break;
      }
      tds_2[idx++] = c;
    }
    tds_2[idx] = '\0';
  }
}

/* ------------------------------------------------- */

void send_to_app() {
  char tds_msg[SIZE * 2]; // Potentially need to add +1 or more memory to this.
  char* dest = tds_msg;
  const char sep[] = ":";

  strcpy(dest, tds_1);  dest += strlen(tds_1);
  strcat(dest, sep);    dest += strlen(sep);
  strcat(dest, tds_2);  dest += strlen(tds_2);
  strcat(dest, "\n");   dest += strlen("\n");

  HM10.write(tds_msg, (size_t)(dest - tds_msg));
}

/* ------------------------------------------------- */

char fetch_command() {
  HM10.listen();
  if( HM10.isListening() && HM10.available() ) {
    return HM10.read();
  }
  return 0;
}

/* ------------------------------------------------- */

void clear_serials(Stream& stream) {
  while( stream.available() ) {
    stream.read();
  }
}
