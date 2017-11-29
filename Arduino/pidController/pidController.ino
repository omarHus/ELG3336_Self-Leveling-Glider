/*** Code for Self-Leveling Glider.
 *** Method: Created two PID controllers for the elevator and the ailerons which are actuated by Servos
 *** Hardware: Arduino Nano, 6-DOF IMU: MPU6050 (accelerometer + gyro), 3 servos
 *** Author: Omar Husain
 *** In collaboration with Jeromy Denk, and Jason Killen
 *** Date: November 26, 2017
***/

#include <Servo.h>
#include <PID_v1.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <MPU6050.h>

/* PID Parameters */
double Kp_roll = 4, Kd_roll = 0.2, Ki_roll = 0.02;
double Kp_pitch = 4, Kd_pitch = 0.1, Ki_pitch = 0;  
double Kp_yaw = 4, Kd_yaw = 0.01, Ki_yaw = 0.1; 

double rollSetpoint = 0, rollInput, rollOutput;
double pitchSetpoint = -4.5, pitchInput, pitchOutput;
float roll, pitch;

PID rollPID(&rollInput, &rollOutput, &rollSetpoint, Kp_roll, Ki_roll, Kd_roll, DIRECT);
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, Kp_pitch, Ki_pitch, Kd_pitch, DIRECT);

/* Servo Parameters */
Servo left_aileronServo, right_aileronServo, elevatorServo, rudderServo;
int minRoll = 900, maxRoll = 2000, minPitch = 900, maxPitch = 2000;
double aileronOutput, elevatorOutput, opposite_aileronOutput, rudderOutput;
int left_aileronPin = 6, right_aileronPin = 9, elevatorPin = 5, rudderPin = 10, ledPin = 13;
bool invertedElevator = true;

unsigned long microsPerReading, microsPrevious, microsNow;

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

/* Sensor */
MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

void setup() {
  /* join I2C bus */
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000);
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
    
  Serial.begin(115200);

  /* Gyro Offsets */
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  /* Attach Servos */
  left_aileronServo.attach(left_aileronPin);
  right_aileronServo.attach(right_aileronPin);
  elevatorServo.attach(elevatorPin);
  
  /* Set all servos to zero position */
  left_aileronServo.write(90);
  elevatorServo.write(90);
  right_aileronServo.write(90);
      
  /* PID Setup */
  rollPID.SetMode(AUTOMATIC);
  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetOutputLimits(-180,180);
  pitchPID.SetOutputLimits(-180,180);
  
  /* initialize variables to pace updates to correct rate */
  microsPerReading = 400;
  microsPrevious = micros();

  /* LED Blink twice To indicate Start of program */
  pinMode(ledPin,OUTPUT);
  digitalWrite(ledPin, HIGH);
  delay(200);
  digitalWrite(ledPin, LOW);
  delay(200);
  digitalWrite(ledPin, HIGH);
  delay(200);
  digitalWrite(ledPin, LOW);  
}

void loop() {

  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {
  
  /* Set PID constants */
  rollPID.SetTunings(Kp_roll, Ki_roll, Kd_roll);
  pitchPID.SetTunings(Kp_pitch, Ki_pitch, Kd_pitch);
  
  /* Compute Yaw, Pitch, and Roll using Madgwick Filter */
  #ifdef OUTPUT_READABLE_YAWPITCHROLL
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  #endif
  
  /* Get Roll and Pitch values for PID input */
  roll = ypr[2]*180/M_PI;
  pitch = ypr[1]*180/M_PI;
  rollInput = roll;
  pitchInput = pitch;
  
  /* print to RPI via serial */       
  Serial.print(rollInput);
  Serial.print(",");
  Serial.print(pitchInput);
  Serial.print(",");
      
  /* RUN PID */
  pitchPID.Compute();
  rollPID.Compute();
   
  /* Invert aileronoutput */
  opposite_aileronOutput = rollOutput;
  
  /* PID output commands*/
  aileronOutput = rollOutput;
  elevatorOutput = pitchOutput;   
    
  if (invertedElevator)
  {
    elevatorOutput = -elevatorOutput;
  }
  
  /* Offset PID output to map onto Servo range */
  elevatorOutput = elevatorOutput +90;
  aileronOutput = aileronOutput +90;
  opposite_aileronOutput = aileronOutput;
  
  /*Print PID output values */
  Serial.print(aileronOutput);
  Serial.print(",");
  Serial.print(elevatorOutput);
  Serial.print(",");
   
  /* Increase resolution of servo commands */
  elevatorOutput = map(elevatorOutput, 0, 180, 450, 2450);
  aileronOutput = map(aileronOutput, 0, 180, 450, 2450);
  opposite_aileronOutput = map(opposite_aileronOutput, 0, 180, 450, 2450);
  
  /* Limit on servo inputs */
  if (elevatorOutput < minPitch)
  {
    elevatorOutput = minPitch;
  }
  if (elevatorOutput > maxPitch)
  {
    elevatorOutput = maxPitch;
  }
  if (aileronOutput < minRoll)
  {
    aileronOutput = minRoll;
  }
  if (aileronOutput > maxRoll)
  {
    aileronOutput = maxRoll;
  }

  /* Send PID output data to RaspberryPi */
  Serial.print(aileronOutput);
  Serial.print(",");
  Serial.println(elevatorOutput);
  
  /* Servo Commands */
  left_aileronServo.writeMicroseconds((int)aileronOutput);
  right_aileronServo.writeMicroseconds((int)aileronOutput);
  elevatorServo.writeMicroseconds((int)elevatorOutput);
  
  microsPrevious = microsPrevious + microsPerReading;
  }
}

