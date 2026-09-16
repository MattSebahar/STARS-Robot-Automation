#include <Servo.h>

Servo myServo;

const int SERVO_PIN = 9;
const int CLOSED_ANGLE = 115;
const int OPEN_ANGLE = 100;

void setup() {
  myServo.attach(SERVO_PIN);
}

void loop() {
  // Start at closed position
  myServo.write(CLOSED_ANGLE);
  delay(500);

  // Open once
  myServo.write(OPEN_ANGLE);
  delay(2000);  // Stay open for 2 seconds

  // Close once
  myServo.write(CLOSED_ANGLE);
  delay(10000);
}