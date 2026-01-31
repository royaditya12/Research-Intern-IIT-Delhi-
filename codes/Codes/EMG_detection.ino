#include <Wire.h>


int emgPin = A0;

void setup() {
  // put your setup code here, to run once:
  //pinMode(emgPin, INPUT);
  Serial.begin(115200);

}

void loop() {
  //float start_time = millis();
  float emgVal = analogRead(emgPin);
  Serial.println(emgVal);
  //Serial.print(",");
  //Serial.print(",");
  //Serial.println("800");
  //Serial.print(",");
  //float time_sample = millis() - start_time;
  //Serial.println(time_sample);
  delay(10);
}

