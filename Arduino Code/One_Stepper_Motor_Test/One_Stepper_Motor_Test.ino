#define DIRECTION 3
#define STEP 4
#define STEPS_PER_REV 200 //200 for Nema 17 Stepper motors


void setup() {
  pinMode(STEP, OUTPUT);
  pinMode(DIRECTION, OUTPUT);
}

void loop() {
//  // sets motor direction to clockwise 
//  digitalWrite(DIRECTION, HIGH);

  //Sets motor direction to CCW
  digitalWrite(DIRECTION, LOW);

  for(int i = 0; i < STEPS_PER_REV; i++){
    digitalWrite(STEP, HIGH);
    delayMicroseconds(2000);
    digitalWrite(STEP, LOW);
    delayMicroseconds(2000);
  }

  delay(1000);
}
