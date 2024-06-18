// SISTEMAS DE CONTROL CONTÍNUO
// INGENIERÍA ELECTRÓNICA UDEA 20241
// Profs. Hernán Felipe García y Amado Tavera Crespo


#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <PID_v1.h>

// Motor A connected between A01 and A02
// Motor B connected between B01 and B02-
#define ENA 6  // Enable/speed motor A
#define IN1 3   // Direction A
#define IN2 4   // Direction A
#define ENB 5   // Enable/speed motor B
#define IN3 7   // Direction B
#define IN4 10   // Direction B

// MPU6050 object
Adafruit_MPU6050 mpu;

// PID variables
double Setpoint, Input, Output;
double Kp = 1.0, Ki = 0.1, Kd = 0.05;  // PID coefficients, tune these for your robot
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(115200);

  // Initialize motor control pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Initialize the MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  // Set up PID controller
  Setpoint = 0;  // Target pitch is 0 degrees (upright)
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);  // PWM limits

  Serial.println("MPU6050 and motor driver initialized");
}

void loop() {
  delay(1000);
  Output = 122;
  driveMotors(Output);
}

void driveMotors(int pwm) {
  if (pwm > 0) {
    analogWrite(ENA, pwm);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENB, pwm);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    analogWrite(ENA, -pwm);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENB, -pwm);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
}
