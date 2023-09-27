#include <SoftwareSerial.h>
#include <GravityTDS.h>

#define TDS_Pin A0
#define DEBUG 0

// MACRO DEFINTION FOR PRINT DEBUGGING
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

/* ------------------------------------------------- */

const size_t SIZE = 4;
STATE program_state = SLEEP;
char tds_2[SIZE];

/* ------------------------------------------------- */

void setup() {
	Serial.begin(9600); 
  while( !Serial ) {}
  clear_serials(Serial);
	debugln("<SLAVE SERIAL READY>");

	HC12.begin(19200); 
  while( !HC12 ) {}
  clear_serials(HC12);
	debugln("<HC12 READY>");

	TDS.setPin(TDS_Pin);
	TDS.begin();
}

void loop() {
  if( program_state == SLEEP ) {
    if( fetch_command() == 'S' ) {
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
    if( millis() - fetch_millis > 250 ) {
      fetch_tds();
      fetch_millis = millis();
    }

    if( fetch_command() == 'E' ) {
      clear_serials(Serial);
      clear_serials(HC12);
      program_state = SLEEP;
      debugln("### PROGRAM STOPPING ###");     
    }

  } 
}

/* ------------------------------------------------- */

void fetch_tds() {
  TDS.update();
  snprintf(tds_2, SIZE + 1, "%u$", (size_t)TDS.getTdsValue()); // Might need to add +1???
  HC12.write(tds_2, SIZE + 1);
}

char fetch_command() {
  if( HC12.available() ) {
    return HC12.read();
  }
  return 0;
}

/* ------------------------------------------------- */

void clear_serials(Stream& stream) {
  while( stream.available() ) {
    stream.read();
  }
}
