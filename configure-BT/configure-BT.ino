#include <AltSoftSerial.h>
#include <SoftwareSerial.h>
SoftwareSerial BTSerial(5, 6);

char c = ' ';
boolean NL = true;

void setup() 
{
  Serial.begin(9600);
  Serial.print("Sketch:   ");   Serial.println(__FILE__);
  Serial.print("Uploaded: ");   Serial.println(__DATE__);
  Serial.println(" ");

  BTSerial.begin(9600);
  Serial.println("BTSerial started at 9600");
}

void loop() {
  if( BTSerial.available() ) {
    c = BTSerial.read();
    Serial.write(c);
  }

  if( Serial.available() ) {
    c = Serial.read();

    if( c != 10 & c != 13 ) {
      BTSerial.write(c);
    }

    if (NL) { Serial.print("\r\n>");  NL = false; }
    Serial.write(c);
    if (c==10) { NL = true; }
  }
}