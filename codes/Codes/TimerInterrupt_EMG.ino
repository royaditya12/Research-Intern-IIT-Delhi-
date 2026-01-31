#include <TimerOne.h>
#define SAMPLE_RATE 1000

const int sensorPin = A0;
volatile bool sampleFlag = false;  // Flag set by timer ISR
unsigned long timestamp = 0;       // For sampled time
bool logging = true;               // Enable/disable logging

void setup() {
  Serial.begin(250000);
  Timer1.initialize(1000000/SAMPLE_RATE);          // 1ms interval (1000 Hz)
  Timer1.attachInterrupt(timerISR);  // Lightweight ISR
}

void timerISR() {
  if (logging) {
    sampleFlag = true;               // Just set a flag
  }
}

void loop() {
  if (sampleFlag) {
    sampleFlag = false;             // Clear the flag immediately

    unsigned long t_us = micros();  // Capture timestamp as close to flag as possible
    int sensorValue = analogRead(sensorPin);
    float voltage = sensorValue * (5.0 / 1023.0);

    Serial.print(t_us / 1000000.0, 4);  // Convert to seconds and print
    Serial.print(',');
    Serial.println(sensorValue);
    
  }
}