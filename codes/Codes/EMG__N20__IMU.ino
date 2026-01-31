#include <TimerOne.h>
#include <Encoder.h>
#include <MPU9250.h>
#include <math.h>
//#include <SimpleKalmanFilter.h>

// Motor control pins (use with motor driver)
#define MOTOR_FWD 10
#define MOTOR_REV 9
#define PWM_EN    5  // PWM pin to control speed

// Encoder pins
#define ENCODER_A 2
#define ENCODER_B 3

#define EMG_PIN A0
#define SAMPLE_RATE 1000     // Hz
#define CLIP_LIMIT 0.75         // volts
#define WINDOW_SIZE 200          // For moving average
#define LPF_B 0.00156           // Low-pass filter coefficient (0-1)
#define LPF_A -1.0
#define BASELINE_DURATION 2000 // First 2s = 2000 samples

float alpha = 0.8; // tuning factor
float dt = 0.01;    // 10 ms loop

// Motor + Encoder specs
const int gearRatio = 100;
const int pulsesPerRev = 28;  // pulses per shaft revolution
const float degreesPerPulse = 360.0 / (gearRatio * pulsesPerRev);
float targetAngle = 90.0;

MPU9250 mpu;
Encoder motorEncoder(ENCODER_A, ENCODER_B);
//SimpleKalmanFilter kalman(2, 2, 0.01);

// PID variables
float Kp = 5;
float Ki = 2.5;
float Kd = 1;

float errorSum = 0;
float lastError = 0;

long lastTime = 0;
int targetAngle ;  // stores the IMU input
//float prevTargetAngle1 = 0.0 ; //stores the previous IMU input
//float prevTargetAngle2 = 0.0;
const int N = 10;
int idx = 0;
//float angleChange = 0.0;  // to compute the successive changes in IMU input

int angle_buffer[N] = {0};
//
volatile bool sampleFlag = false;  // Flag set by timer ISR

float clippedSignal = 0;
float filteredSignal = 0;
float lpfOutput = 0;

float baselineSum = 0;
float baselineSqSum = 0;
int baselineCount = 0;
float mean = 0, stddev = 0;

float window[WINDOW_SIZE];
int windowIndex = 0;
bool baselineDone = false;

float threshold = 0.0;
float threshold1 = 0.0;
static float prev = 0;

unsigned long lastMotorUpdate = 0;
const unsigned long motorInterval = 20; // ms

bool flag_detect = false;


void setup() {
  Serial.begin(250000);
  Timer1.initialize(1000000 / SAMPLE_RATE);  // 1ms = 1000Hz
  Timer1.attachInterrupt(readEMG);
  //
  pinMode(MOTOR_FWD, OUTPUT);
  pinMode(MOTOR_REV, OUTPUT);
  pinMode(PWM_EN, OUTPUT);

  digitalWrite(MOTOR_FWD, LOW);
  digitalWrite(MOTOR_REV, LOW);
  analogWrite(PWM_EN, 0);

  motorEncoder.write(0);
  lastTime = millis();

 /* Wire.begin();
  if (!mpu.setup(0x68)) {
    if (!mpu.setup(0x69)) {
      Serial.println("MPU9250 not connected at 0x68 or 0x69");
      while (1);
    } else {
      Serial.println("MPU9250 found at 0x69");
    }
  } else {
  Serial.println("MPU9250 found at 0x68");
  }*/
}

void readEMG() {
  sampleFlag = true;
}

void loop() {
  if(sampleFlag){
    sampleFlag = false ;

    unsigned long t_us = micros(); 

    int rawADC = analogRead(EMG_PIN);
    float voltage = (rawADC / 1023.0) * 5;  // Convert to volts

    Serial.print(t_us / 1000000.0, 4);  // Convert to seconds and print
    Serial.print(',');
    Serial.print(voltage,5);
    
    float clipped = clipAmplitude(voltage);
    float averaged = movingAverage(clipped);
    lpfOutput = lowPassFilter(averaged);

    Serial.print(',');
    Serial.print(clipped);
    Serial.print(',');
    Serial.print(averaged,5);
    Serial.print(',');
    Serial.print(lpfOutput,5);

    if (!baselineDone) {
      threshold1 = updateBaseline(lpfOutput);
    } else if(flag_detect==false) {
      threshold = threshold1;
      flag_detect = detectActivation(lpfOutput);
    } else if(flag_detect==true) {
        
    }
    


void loop() {
  if (millis() - lastMotorUpdate >= motorInterval) {
    lastMotorUpdate = millis();

    motor_control(directionFlag, desiredPWM);  // Efficient function from above
  }
    /*if(flag_detect == true){
          digitalWrite(MOTOR_FWD, LOW);
          digitalWrite(MOTOR_REV, HIGH);
          analogWrite(PWM_EN, 150);
         // delay(100);
    }
    else 
    {
      digitalWrite(MOTOR_FWD, HIGH);
      digitalWrite(MOTOR_REV, LOW);
      analogWrite(PWM_EN, 150);
      //delay(100);
    }*/
    digitalWrite(MOTOR_FWD, LOW);
    digitalWrite(MOTOR_REV, HIGH);
    analogWrite(PWM_EN, 150);
    
  Serial.print(',');
  Serial.println(threshold,5);
  }

}

float clipAmplitude(float val) {
  if (val > CLIP_LIMIT) return CLIP_LIMIT;
  //if (val < -CLIP_LIMIT) return -CLIP_LIMIT;
  return val;
}

float movingAverage(float input) {
  window[windowIndex] = input;
  windowIndex = (windowIndex + 1) % WINDOW_SIZE;

  float sum = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) sum += window[i];
  return sum / WINDOW_SIZE;
}

float lowPassFilter(float input) {
  float output = LPF_B * input + LPF_B * prev - LPF_A * prev;
  prev = input;
  return output;
}

float updateBaseline(float value) {
  baselineSum += value;
  baselineSqSum += value * value;
  baselineCount++;

  if (baselineCount >= BASELINE_DURATION) {
    mean = baselineSum / baselineCount;
    float variance = (baselineSqSum / baselineCount) - (mean * mean);
    stddev = sqrt(variance);
    baselineDone = true;
    /*Serial.print("Baseline complete. Mean: ");
    Serial.print(mean, 4);
    Serial.print("  Std Dev: ");
    Serial.println(stddev, 4);*/
    return mean+3*stddev ;
  }
}

bool detectActivation(float val) {
  float threshold = mean + 3*stddev;
  if (val > threshold) {
    return true;
  } else {
    return false;
  }
}

void motor_control(){

  /*if(mpu.update())  {

        long pulseCount = motorEncoder.read();
        float rawAngle = pulseCount * degreesPerPulse;
        //float shaftAngle = kalman.updateEstimate(rawAngle);
       float shaftAngle = rawAngle ;

        float ax = mpu.getAccX();
        float ay = mpu.getAccY();
        float az = mpu.getAccZ();
        
        float gyroXrate = mpu.getGyroX();  // degrees/s
        float gyroYrate = mpu.getGyroY();

        float accRoll = atan2(ay, sqrt(ax*ax + az*az))*(180./PI);
        float accPitch = atan2(ax,sqrt(ay*ay + az*az))*(180./PI);
        //float yaw_o = atan2(az, sqrt(ax*ax + ay*ay))*(180./3.142);
        
        // Gyro + Accel
        float roll  = alpha * (roll + gyroXrate * dt) + (1 - alpha) * accRoll;
        float pitch = alpha * (pitch + gyroYrate * dt) + (1 - alpha) * accPitch;

        float roll_1  = constrain(roll, -83, 83);
        float pitch_1 = constrain(pitch, -83, 83);

        float roll_new = map(roll_1, -83, 83, 0, 180);
        float pitch_new = map(pitch_1, -83, 83, 0, 180);

        angle_buffer[idx] = int(roll_new);
        targetAngle = int(computeRMS(angle_buffer));
        idx = (idx + 1) % N ;
        // --- Compute PID ---
        int error = targetAngle - shaftAngle;

        long now = millis();
        float dt = (now - lastTime) / 1000.0;  // seconds

        errorSum += ((error + lastError)/2) * dt;     //for integration term
        float dError = (error - lastError) / dt;  //

        float pidOutput = Kp * error + Ki * errorSum + Kd * dError;

        lastError = error;
        lastTime = now;

        // --- Apply motor control ---
        int pwm = constrain(abs(pidOutput), 0, 255);


        
        if (abs(error) < 0.5) {   //if rotor angle is close enough to the target
          analogWrite(PWM_EN, 0);
          digitalWrite(MOTOR_FWD, LOW);
          digitalWrite(MOTOR_REV, LOW);
        }
        else if (pidOutput > 0) {
          digitalWrite(MOTOR_FWD, HIGH);
          digitalWrite(MOTOR_REV, LOW);
          analogWrite(PWM_EN, pwm);
        } else {
          digitalWrite(MOTOR_FWD, LOW);
          digitalWrite(MOTOR_REV, HIGH);
          analogWrite(PWM_EN, pwm);
        }

        // --- Debug info ---
        //Serial.print("Angle: ");
        Serial.print("0");
        Serial.print(",");      // Y-axis minimum
        Serial.print(roll_new);
        Serial.print(",");
        Serial.print(targetAngle);
        Serial.print(",");
        Serial.print(shaftAngle);
        Serial.print(",");

        Serial.println("180");  // Y-axis maximum

        delay(10);  // Small delay for stability
  }*/
  

}
/*float computeRMS(int *buffer) {
  float sumSq = 0;
  for (int i = 0; i < N; i++) {
    sumSq += buffer[i] * buffer[i];
  }
  return sqrt(sumSq / N);
}*/

