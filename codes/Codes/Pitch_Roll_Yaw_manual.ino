#include <MPU9250.h>

#include <math.h>

MPU9250 mpu;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  if(!mpu.setup(0x68)) {
    Serial.println("MPU not connected successfully");
    while(1);
  }
  mpu.calibrateAccelGyro();

}

void loop() {
  // put your main code here, to run repeatedly:
  if(mpu.update())  {
        float ax, ay, az;
        float pitch, roll, yaw ;
        ax = mpu.getAccX();
        ay = mpu.getAccY();
        az = mpu.getAccZ();

        Serial.println("The accelerations are: "); 
        Serial.print("Acc X: "); Serial.println(ax, 2); 
        Serial.print("Acc Y: "); Serial.println(ay,2);
        Serial.print("Acc Z: "); Serial.println(az,2);

        roll = atan2(ay, sqrt(ax*ax + az*az))*(180./3.142);
        pitch = atan2(ax,sqrt(ay*ay + az*az))*(180./3.142);
        yaw = atan2(az, sqrt(ax*ax + ay*ay))*(180./3.142);

        float roll_1  = constrain(roll, -83, 83);
        float pitch_1 = constrain(pitch, -83, 83);

        float roll_new = map (roll_1, -83, 83, 0, 180);
        float pitch_new = map(pitch_1, -83, 83, 0, 180);

        //Serial.println("\n\nThe angles are :");
        //Serial.print("Roll : "); 
        Serial.print(roll_new);
        Serial.print(",");
        //Serial.print("Pitch : "); 
        Serial.print(pitch_new);
        //Serial.print("Yaw : ");
        Serial.print(",");
        Serial.println(yaw);


    }
    
  
  delay(1000);
}

