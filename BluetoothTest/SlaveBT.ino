#include <SoftwareSerial.h>

SoftwareSerial HC12(10, 11); // RX, TX
float tdsValue = 0;

void setup() {
  Serial.begin(115200);   
  HC12.begin(115200); 
}

void loop() {
  delay(100);
  tdsValue = analogRead(A0); // read TDS sensor value
  HC12.write(tdsValue); // send TDS data to master
  delay(1000);
}
