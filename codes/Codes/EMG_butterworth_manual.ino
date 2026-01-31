// Sampling rate (Hz)
const float fs = 1000.0;

// Band-pass Butterworth (2nd order) - designed using Python/scipy for 20-450 Hz
// Coefficients are normalized for fs = 1000 Hz
// Use tools like scipy.signal.butter() to generate new coefficients if fs changes

// Example coefficients for band-pass 20–450 Hz, fs = 1000 Hz
float a0 = 1.0;
float a1 = -0.3695;
float a2 = 0.1958;
float b0 = 0.2066;
float b1 = 0.0;
float b2 = -0.2066;

// Filter history variables
float x1 = 0, x2 = 0;  // Previous inputs
float y1 = 0, y2 = 0;  // Previous outputs

// Analog input pin connected to MyoWare output
const int emgPin = A0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  // Read raw EMG input
  int rawADC = analogRead(emgPin);

  // Convert to voltage (assuming 5V ADC reference and 10-bit ADC)
  float vin = rawADC * (5.0 / 1023.0);

  // Apply band-pass Butterworth filter (biquad)
  float y = b0 * vin + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;

  // Shift history
  x2 = x1;
  x1 = vin;
  y2 = y1;
  y1 = y;

  // Send both raw and filtered values to Serial Plotter
  Serial.print(vin, 3);    // Raw EMG
  Serial.print(",");
  Serial.println(y, 3);    // Filtered EMG

  delayMicroseconds(1000); // Sampling delay (1000 Hz)
}

