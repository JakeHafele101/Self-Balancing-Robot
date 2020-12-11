#include<Stepper.h>

#define DIRECTION1 3
#define STEP1 4
#define DIRECTION2 5
#define STEP2 6
#define STEPS_PER_REV 200 //200 for Nema 17 Stepper motors
#define SPEED 100

int moveState;

Stepper motor1(STEPS_PER_REV, STEP1, DIRECTION1);
Stepper motor2(STEPS_PER_REV, STEP2, DIRECTION2);

void setup() {

  motor1.setSpeed(SPEED);
  motor2.setSpeed(SPEED);

  moveState = 0;
}

void loop() {


  switch(moveState){
    // Forward
    case(0):
      motor1.step(1);
      motor2.step(-1);
      break;
      
    //Backward
    case(1):
      motor1.step(-1);
      motor2.step(1);
      break;

    //Left
    case(2):
      motor1.step(-1);
      motor2.step(-1);
      break;

    //Right
    case(3):
      motor1.step(1);
      motor2.step(1);
      break;
  }
}
