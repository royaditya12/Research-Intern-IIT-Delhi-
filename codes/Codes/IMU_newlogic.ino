#include <Wire.h>
#include <MPU9250.h>

MPU9250 mpu;

float accRoll, accPitch;
float roll = 0.0, pitch = 0.0;
float alpha = 0.98;          // Weight for gyro in complementary filter
unsigned long lastTime = 0;  // For computing dt

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  mpu.setup(0x68);

  //if(!mpu.setup(0x68)) { // change to 0x69 if your MPU9250 uses that address
   // Serial.println("MPU9250 connection failed!");
    //while (1);
  //}

  // Wait for sensor to stabilize
  delay(1000);
  lastTime = millis();
}

void loop() {
  if (mpu.update()) {
    // Time delta (in seconds)
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // --- Get accelerometer-based roll and pitch ---
    float ax = mpu.getAccX();
    float ay = mpu.getAccY();
    float az = mpu.getAccZ();

    accRoll  = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
    accPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

    // --- Get gyroscope angular velocity ---
    float gx = mpu.getGyroX();  // degrees/sec
    float gy = mpu.getGyroY();

    // --- Complementary Filter ---
    roll  = alpha * (roll + gx * dt) + (1 - alpha) * accRoll;
    pitch = alpha * (pitch + gy * dt) + (1 - alpha) * accPitch;

    // --- Output ---
    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print("  Pitch: ");
    Serial.println(pitch);
  }
}
