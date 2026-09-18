#include <Servo.h>

Servo myServo;

const int SERVO_PIN = 9;
const int CLOSED_ANGLE = 115;
const int OPEN_ANGLE = 100;

void setup() {
  myServo.attach(SERVO_PIN);
}

//mark 1 before pickup: 12.41
//mark 2 pickup: 2.83
//mark 3 before drop: 24.65
//mark 4 drop: 1.23
//mark 5 retrieve: 11.91
//mark 6 before drop off: 2.75
//mark 7 drop off: 4.08

void loop() {
  // Alert to loop start. Hit start after the third open
  myServo.write(CLOSED_ANGLE);
  delay(1000);
  myServo.write(OPEN_ANGLE);
  delay(1000);
  myServo.write(CLOSED_ANGLE);
  delay(5000);
  myServo.write(OPEN_ANGLE);
  delay(500);
  myServo.write(CLOSED_ANGLE);

  // Mark 1 — before pickup
  delay(11000);
  myServo.write(OPEN_ANGLE);

  // Mark 2 — pickup
  delay(2750);
  myServo.write(CLOSED_ANGLE);

  // Mark 3 — before drop
  delay(24650);

  // Mark 4 — drop
  delay(1230);
  myServo.write(OPEN_ANGLE);

  // Close for testing
  delay(4000);
  myServo.write(CLOSED_ANGLE);

  // Mark 5 — retrieve
  delay(9000);
  myServo.write(OPEN_ANGLE);

  // Mark 6 — before drop-off
  delay(2500);
  myServo.write(CLOSED_ANGLE);

  // Mark 7 — drop-off
  delay(15500);
  myServo.write(OPEN_ANGLE);
  delay(4000);
}