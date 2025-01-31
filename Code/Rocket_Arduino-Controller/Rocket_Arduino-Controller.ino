/*

Controller side code for arduino 

*/

#include <SoftwareSerial.h>

unsigned long prevTime;

byte incoming;

SoftwareSerial HC12(10, 11); // HC-12 TX Pin, HC-12 RX Pin

void setup() {
  Serial.begin(9600);             // Serial port to computer
  HC12.begin(9600);               // Serial port to HC12
  /*pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  digitalWrite(9, HIGH);
  digitalWrite(8, HIGH);
  delay(1);
  digitalWrite(9, LOW);*/

}

void loop() {
  while (Serial.available()) {      // If Serial monitor has data
    incoming = Serial.read(); 
    HC12.write(incoming);
    Serial.write(incoming);
  }
  while (HC12.available()) {
    Serial.write(HC12.read());
  }
  /*if (millis()-prevTime) {
    digitalWrite(9, HIGH);
    delay(1);
    digitalWrite(9, LOW);
    prevTime = millis();

  }*/
}