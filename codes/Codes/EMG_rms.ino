#include <Wire.h>


int emgPin = A0;

const int N = 10;
int idx = 0;
int data_buffer[N] = {0};

void setup() {
  // put your setup code here, to run once:
  //pinMode(emgPin, INPUT);
  Serial.begin(115200);

}

void loop() {
  //float start_time = millis();
  float emgVal = analogRead(emgPin);
  data_buffer[idx] = int(emgVal);
  int rmsVal = int(computeRMS(data_buffer));
  idx = (idx + 1) % N ;
  Serial.print(emgVal);
  Serial.print(",");
  Serial.println(rmsVal);
  //Serial.print(",");
  //Serial.print(",");
  //Serial.println("800");
  //Serial.print(",");
  //float time_sample = millis() - start_time;
  //Serial.println(time_sample);
  delay(10);
}
float computeRMS(int *buffer) {
  float sumSq = 0;
  for (int i = 0; i < N; i++) {
    sumSq += buffer[i] * buffer[i];
  }
  return sqrt(sumSq / N);
}
