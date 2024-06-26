#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SoftwareSerial.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#define MIN_ABS_SPEED 120
MPU6050 mpu;
//*************************************** Ajustes ***************************************
double MotorVelocidadIzq = 0.6; //double MotorVelocidadIzq = 0.3;
double MotorVelocidadDer = 0.6; //double MotorVelocidadDer = 0.3;
double PuntoEquilibrio = 0;

//-----------------Control de Motores
#define ENA 5  // Enable/speed motor A
#define IN1 7   // Direction A
#define IN2 6   // Direction A
#define ENB 8   // Enable/speed motor B
#define IN3 9   // Direction B
#define IN4 4   // Direction B

//***************************************************************************************+
// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector


double originalSetpoint = PuntoEquilibrio;   //double originalSetpoint = 172.50;

double setpoint = originalSetpoint; 
double movingAngleOffset = 0.1;
double input, output;


double motorSpeedFactorLeft = MotorVelocidadIzq; //double motorSpeedFactorLeft = 0.6;
double motorSpeedFactorRight = MotorVelocidadDer; //double motorSpeedFactorRight = 0.5;


LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

float Ts = 0.01;

//A observador
double a11 = -0.033, a12 = -0.079, a13 = 0.002, a14 = 0.006;
double a21 = 0.085, a22 = 0.201, a23 = -0.010, a24 = -0.016;
double a31 = -1.522, a32 = -3.597, a33 = 0.183, a34 = 0.282;
double a41 = 2.097, a42 = 4.947, a43 = -0.257, a44 = -0.397;

//B observador
double b11 = 58.0,    b12 = -23.0;
double b21 = -27.0,   b22 = 58.0;
double b31 = 255.0,   b32 = -1033.0;
double b41 = -177.0,  b42 = 1420.0;

//Variables de Estado
double x1_hat = 0;
double x1d_hat = 0;
double x2_hat = 0;
double x2d_hat = 0;

//Ganancia del controlador
double k1 = -121.12, k2 = -170.87, k3 = 10.91, k4 = 2.003; 

//Señal de control
double u, umax, umin;

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
 mpuInterrupt = true;
}


void setup()
{
//Serial_2.begin(9600);    // inicia el puerto serial para comunicacion con el Bluetooth
Serial.begin(9600);  
 // join I2C bus (I2Cdev library doesn't do this automatically)
 #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 Wire.begin();
 TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
 #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
 Fastwire::setup(400, true);
 #endif

 mpu.initialize();

 devStatus = mpu.dmpInitialize();

 // supply your own gyro offsets here, scaled for min sensitivity
 mpu.setXGyroOffset(-27);
 mpu.setYGyroOffset(-25);
 mpu.setZGyroOffset(-67);
 mpu.setZAccelOffset(1435); // 1688 factory default for my test chip

 // make sure it worked (returns 0 if so)
 if (devStatus == 0)
 {
 // turn on the DMP, now that it's ready
 mpu.setDMPEnabled(true);

 // enable Arduino interrupt detection
 attachInterrupt(0, dmpDataReady, RISING);
 mpuIntStatus = mpu.getIntStatus();

 // set our DMP Ready flag so the main loop() function knows it's okay to use it
 dmpReady = true;

 // get expected DMP packet size for later comparison
 packetSize = mpu.dmpGetFIFOPacketSize();
 }
 else
 {
 // ERROR!
 // 1 = initial memory load failed
 // 2 = DMP configuration updates failed
 // (if it's going to break, usually the code will be 1)
 Serial.print(F("DMP Initialization failed (code "));
 Serial.print(devStatus);
 Serial.println(F(")"));
 }
}

void loop()
{
  
 // if programming failed, don't try to do anything
 if (!dmpReady) return;

 // wait for MPU interrupt or extra packet(s) available
 
    //no mpu data - performing PID calculations and output to motors 
    //Serial.println(y_r);

    
    //Reiniciar los estados
    if (x1_hat > 10000)
      {x1_hat = 0;}
    else if (x1d_hat > 10000)
      {x1d_hat = 0;}   
    else if (x2_hat > 10000)
      {x2_hat = 0;}
    else if (x2d_hat > 10000)
      {x2d_hat = 0;}  

    // States of the controller
    x1_hat = Ts*(a11*x1_hat + a12*x1d_hat + a13*x2_hat + a14*x2_hat + b11*u + b12*input);
    x1d_hat = Ts*(a21*x1_hat + a22*x1d_hat + a23*x2_hat + a24*x2_hat + b21*u + b22*input);
    x2_hat =  Ts*(a31*x1_hat + a32*x1d_hat + a33*x2_hat + a34*x2_hat + b31*u + b32*input);
    x2d_hat =  Ts*(a41*x1_hat + a42*x1d_hat + a43*x2_hat + a44*x2_hat + b41*u + b42*input);

    // control signal
    u = -(k1*x1_hat + k2*x1d_hat + k3*x2_hat + k4*x2d_hat) + setpoint;
    umax = 130;
    umin = -1;

    output = map(u,umin,umax,-255,255);
    output = -output;

    Serial.print("u:  ");
    Serial.println(u);
    Serial.println(output);
    motorController.move(output, MIN_ABS_SPEED);

 // reset interrupt flag and get INT_STATUS byte
 mpuInterrupt = false;
 mpuIntStatus = mpu.getIntStatus();

 // get current FIFO count
 fifoCount = mpu.getFIFOCount();

 // check for overflow (this should never happen unless our code is too inefficient)
 if ((mpuIntStatus & 0x10) || fifoCount == 1024)
 {
 // reset so we can continue cleanly
 mpu.resetFIFO();
 Serial.println(F("FIFO overflow!"));

 // otherwise, check for DMP data ready interrupt (this should happen frequently)
 }
 else if (mpuIntStatus & 0x02)
 {
 // wait for correct available data length, should be a VERY short wait
 while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

 // read a packet from FIFO
 mpu.getFIFOBytes(fifoBuffer, packetSize);
 
 // track FIFO count here in case there is > 1 packet available
 // (this lets us immediately read more without waiting for an interrupt)
 fifoCount -= packetSize;

 mpu.dmpGetQuaternion(&q, fifoBuffer);
 mpu.dmpGetGravity(&gravity, &q);
 mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
 input = ypr[1] * 180/M_PI;
 
 
 Serial.print("Input:   ");
 Serial.print(input);
 Serial.print(" | Output:   ");
 Serial.println(output);
 }
}