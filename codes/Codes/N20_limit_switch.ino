#include <Encoder.h>
//#include <SimpleKalmanFilter.h>

// Motor control pins (use with motor driver)
#define MOTOR_FWD 10
#define MOTOR_REV 9
#define PWM_EN    5  // PWM pin to control speed

// Encoder pins
#define ENCODER_A 2
#define ENCODER_B 3

#define LM_SW_PIN 7
bool homed = false;
// Motor + Encoder specs
const int gearRatio = 100;
const int pulsesPerRev = 28;  // pulses per shaft revolution
const float degreesPerPulse = 360.0 / (gearRatio * pulsesPerRev);

Encoder motorEncoder(ENCODER_A, ENCODER_B);
//SimpleKalmanFilter kalman(2, 2, 0.01);

// PID variables
float Kp = 10;
float Ki = 5;
float Kd = 2;

float errorSum = 0;
float lastError = 0;
float limit_reached = 0;

float shaftAngle, rawAngle, pidOutput, error;
bool switchPinState = LOW; 

long lastTime = 0;
float targetAngle = 360;  // Change as needed

void setup() {
  Serial.begin(9600);
  pinMode(LM_SW_PIN, INPUT);
  pinMode(MOTOR_FWD, OUTPUT);
  pinMode(MOTOR_REV, OUTPUT);
  pinMode(PWM_EN, OUTPUT);

  motorEncoder.write(0);
  Serial.print(0); Serial.print(" "); Serial.print(1000);
}

void loop() {
  // --- Compute shaft position in degrees ---
  long pulseCount = motorEncoder.read();
  switchPinState = digitalRead(LM_SW_PIN);
  if(switchPinState == HIGH && homed == false) {
    homed = true;
    analogWrite(PWM_EN, 0);
    digitalWrite(MOTOR_FWD, LOW);
    digitalWrite(MOTOR_REV, LOW);
    motorEncoder.write(0);
    delay(500);
    lastTime = millis();
  }
  else if(homed == true) {
    rawAngle = pulseCount * degreesPerPulse;
    shaftAngle = rawAngle ;


    // --- Compute PID ---
    error = targetAngle - shaftAngle;

    long now = millis();
    float dt = (now - lastTime) / 1000.0;  // seconds

    errorSum += ((error + lastError)/2) * dt;
    float dError = (error - lastError) / dt;

    pidOutput = Kp * error + Ki * errorSum + Kd * dError;

    lastError = error;
    lastTime = now;

    // --- Apply motor control ---
    int pwm = constrain(abs(pidOutput), 0, 255);

    if (abs(error) < 0.5) {
      // close enough to target
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
  }
  else {
    digitalWrite(MOTOR_FWD, LOW);
    digitalWrite(MOTOR_REV, HIGH);
    analogWrite(PWM_EN, 70);
  }
    // --- Debug info ---
    //Serial.print("Angle: ");
    //Serial.print("60,");      // Y-axis minimum
    Serial.print(shaftAngle);
    Serial.print(",");
    Serial.print(error);
    Serial.print(",");
    Serial.println(pulseCount);
    //Serial.println(", 120, ");  // Y-axis maximum

    //Serial.print(" | PWM: "); Serial.print(pwm);
    //Serial.println(pwm); 
  
  delay(10);  // Small delay for stability
}
