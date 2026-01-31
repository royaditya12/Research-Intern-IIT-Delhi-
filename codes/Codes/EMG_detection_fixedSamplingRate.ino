const int emgPin = A0;            // EMG sensor pin
const unsigned long sampleInterval = 1;  // Sampling interval in microseconds ( 1000 Hz)
unsigned long lastSampleTime = 0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  unsigned long currentTime = millis();

  // Check if it's time to sample
  if (currentTime - lastSampleTime >= sampleInterval) {
    lastSampleTime += sampleInterval;  // Use += to reduce drift
    int emgValue = analogRead(emgPin);
    Serial.println(emgValue);
  }
}

