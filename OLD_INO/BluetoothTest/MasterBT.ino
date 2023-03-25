#include <SoftwareSerial.h>

SoftwareSerial HC12(10, 11); // RX, TX
float tdsValue_1 = 0;
float tdsValue_2 = 0;

void setup() {
  Serial.begin(115200);   
  HC12.begin(115200); 
}

void loop() {
  tdsValue_1 = analogRead(A0);
  HC12.write(tdsValue_1);
  delay(100); // Buffer Latency between sending data
  if ( HC12.available() ) {
    tdsValue_2 = HC12.read();
    HC12.write(tdsValue_2);
  }
  delay(1000);
}
