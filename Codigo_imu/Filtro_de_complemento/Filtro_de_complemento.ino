// SISTEMAS DE CONTROL CONTÍNUO
// INGENIERÍA ELECTRÓNICA UDEA 20241
// Angee, Imar y Oscar

// Librerias para controlar el mpu6050
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <PID_v1.h>

// MPU6050 Object
MPU6050 sensor;

// Motor A connected between A01 and A02
// Motor B connected between B01 and B02
#define ENA 10  // Enable/speed motor A
#define IN1 4   // Direction A
#define IN2 8   // Direction A
#define ENB 5   // Enable/speed motor B
#define IN3 7   // Direction B
#define IN4 6   // Direction B

  // PID variables
  double Setpoint, Input, Output;
  double Kp = 18, Ki = 0.5, Kd = 0.08;  // PID coefficients, tune these for your robot
  PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

  // Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
  int ax, ay, az;
  int gx, gy, gz;

  long tiempo_prev;
  float dt;
  float ang_x, ang_y;
  float ang_x_prev, ang_y_prev;

void setup() {
  Serial.begin(57600);    //Iniciando puerto serial
  Wire.begin();           //Iniciando I2C  
  sensor.initialize();    //Iniciando el sensor

  // Initialize motor control pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  //Try initialize sensor
  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");


  //Get sensor offsets
  ax=sensor.getXAccelOffset();
  ay=sensor.getYAccelOffset();
  az=sensor.getZAccelOffset();
  gx=sensor.getXGyroOffset();
  gy=sensor.getYGyroOffset();
  gz=sensor.getZGyroOffset();

  // Set up PID controller
  Setpoint = -6.5;  // Target pitch is 0 degrees (upright)
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);  // PWM limits
}

void loop() {
  // Leer las aceleraciones y velocidades angulares
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  dt = (millis()-tiempo_prev)/1000.0;
  tiempo_prev=millis();
  
  //Calcular los ángulos con acelerometro
  float accel_ang_x=atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14);
  float accel_ang_y=atan(-ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14);
  
  //Calcular angulo de rotación con giroscopio y filtro complemento  
  ang_x = 0.98*(ang_x_prev+(gx/131)*dt) + 0.02*accel_ang_x;
  ang_y = 0.98*(ang_y_prev+(gy/131)*dt) + 0.02*accel_ang_y;
  
  
  ang_x_prev=ang_x; 
  ang_y_prev=ang_y;

  if(ang_y < 1.5 && ang_y > -15){
    Kp = 6.5;
    Ki = 0.09;
    Kd = 0.8;
  }
  else{
    Kp = 20;
    Ki = 0.3;
    Kd = 0.08;
  }

  Input = ang_y;

  myPID.SetTunings(Kp,Ki,Kd);
  myPID.Compute();

  driveMotors(Output);

  //Mostrar los angulos separadas por un [tab]
  Serial.print("Rotacion en X:  ");
  Serial.print(ang_x); 
  Serial.print("\tRotacion en Y: ");
  Serial.println(ang_y);

  Serial.print("Out:\t");
  Serial.println(Output);

  delay(100);
}

void driveMotors(int pwm) {
  Serial.print("Pwm: \t");
  Serial.println(pwm);
  if (pwm > 0) {
    analogWrite(ENA, pwm);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENB, pwm);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    analogWrite(ENA, -pwm);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENB, -pwm);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
}