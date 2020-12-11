//#define TRIG A1
//#define ECHO A0
//
//float distance, duration;
//
//void setup() {
//  pinMode(TRIG, OUTPUT);
//  pinMode(ECHO, INPUT);
//  Serial.begin(9600);
//}
//
//void loop() {
//  // clears Trig pin
//  digitalWrite(TRIG, LOW);
//  delayMicroseconds(2);
//
//  //Sets Trig pin on HIGH state for 10 microseconds
//  digitalWrite(TRIG, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(TRIG, LOW);
//
//  //Reads echoPin, returns sound wave travel in time microseconds
//  duration = pulseIn(ECHO, HIGH);
//
//  //Calculates distance in centimeters
//  distance = duration * 0.034/2;
//
//  //Distance is in centimeters, might want to convert to feet?
//  Serial.print("Distance: ");
//  Serial.println(distance);
//  delay(100);
//}

#include <NewPing.h>
#define TRIGGER_PIN  A1  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     A0  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
}

void loop() {
  delay(50);                      // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
  Serial.print("Ping: ");
  Serial.print(sonar.convert_cm(uS)); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
  Serial.println("cm");
}
