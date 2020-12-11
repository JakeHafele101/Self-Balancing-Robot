#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"
#include <NewPing.h>
#include <Stepper.h>

#define leftMotorStep   6
#define leftMotorDirection   5
#define rightMotorStep  4
#define rightMotorDirection  3
#define STEPS_PER_REV 200 //200 for Nema 17 Stepper motors

#define TRIGGER_PIN 1
#define ECHO_PIN 0
#define MAX_DISTANCE 75

#define Kp  40
#define Kd  0.05
#define Ki  40
#define sampleTime  0.005
#define targetAngle -2.5

MPU6050 mpu;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

Stepper leftMotor(STEPS_PER_REV, leftMotorStep, leftMotorDirection);
Stepper rightMotor(STEPS_PER_REV, rightMotorStep, rightMotorDirection);

int16_t accY, accZ, gyroX;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
volatile byte count=0;
int distanceCm;

void setMotors(int leftMotorSpeed, int rightMotorSpeed) {
  if(leftMotorSpeed >= 0) {
//    leftMotor.setSpeed(leftMotorSpeed);
    leftMotor.step(1);
  }
  else {
//    leftMotor.setSpeed(255 + leftMotorSpeed);
    leftMotor.step(-1);
  }
  if(rightMotorSpeed >= 0) {
//    rightMotor.setSpeed(rightMotorSpeed);
    rightMotor.step(1);
  }
  else {
//    rightMotor.setSpeed(255 + rightMotorSpeed);
    rightMotor.step(-1);
  }
}

//void setMotors(int leftMotorSpeed, int rightMotorSpeed) {
//  if(leftMotorSpeed >= 0) {
//    analogWrite(leftMotorPWMPin, leftMotorSpeed);
//    digitalWrite(leftMotorDirPin, LOW);
//  }
//  else {
//    analogWrite(leftMotorPWMPin, 255 + leftMotorSpeed);
//    digitalWrite(leftMotorDirPin, HIGH);
//  }
//  if(rightMotorSpeed >= 0) {
//    analogWrite(rightMotorPWMPin, rightMotorSpeed);
//    digitalWrite(rightMotorDirPin, LOW);
//  }
//  else {
//    analogWrite(rightMotorPWMPin, 255 + rightMotorSpeed);
//    digitalWrite(rightMotorDirPin, HIGH);
//  }
//}

void init_PID() {  
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B    
  // set compare match register to set sample time 5ms
  OCR1A = 9999;    
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}

void setup() {
  // set the motor control and PWM pins to output mode
//  pinMode(leftMotorPWMPin, OUTPUT);
//  pinMode(leftMotorDirPin, OUTPUT);
//  pinMode(rightMotorPWMPin, OUTPUT);
//  pinMode(rightMotorDirPin, OUTPUT);
  // set the status LED to output mode 
  pinMode(13, OUTPUT);
  // initialize the MPU6050 and set offset values
  mpu.initialize();
  mpu.setXAccelOffset(-2924);
  mpu.setZAccelOffset(3983);
  mpu.setYGyroOffset(-66);
  // initialize PID sampling loop
  init_PID();

  leftMotor.setSpeed(50);
  rightMotor.setSpeed(50);

  Serial.begin(115200);
}

void loop() {
  // read acceleration and gyroscope values
  accY = mpu.getAccelerationX();
  accZ = mpu.getAccelerationZ();  
  gyroX = mpu.getRotationY();
  
  // set motor power after constraining it
  motorPower = constrain(motorPower, -255, 255);
  Serial.println(accY);
  setMotors(motorPower, motorPower);
  
  // measure distance every 100 milliseconds
  if((count%20) == 0){
    distanceCm = sonar.ping_cm();
  }
  if((distanceCm < 20) && (distanceCm != 0)) {
    setMotors(-motorPower, motorPower);
  }
}
// The ISR will be called every 5 milliseconds
ISR(TIMER1_COMPA_vect)
{
  // calculate the angle of inclination
  accAngle = atan2(accY, accZ)*RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate*sampleTime;  
  currentAngle = 0.9934*(prevAngle + gyroAngle) + 0.0066*(accAngle);
  
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;  
  errorSum = constrain(errorSum, -300, 300);
  //calculate output from P, I and D values
  motorPower = Kp*(error) + Ki*(errorSum)*sampleTime - Kd*(currentAngle-prevAngle)/sampleTime;
  prevAngle = currentAngle;
  // toggle the led on pin13 every second
  count++;
  if(count == 200)  {
    count = 0;
    digitalWrite(13, !digitalRead(13));
  }
}
