#include <Servo.h>

Servo myServo;

const int SERVO_PIN = 9;
const int CLOSED_ANGLE = 115;
const int OPEN_ANGLE = 100;

void setup() {
  myServo.attach(SERVO_PIN);
}

void loop() {
 
  // Opening sequence to indicate that the loop is starting again
  myServo.write(OPEN_ANGLE);
  delay(500);
  myServo.write(CLOSED_ANGLE);
  delay(500);
  myServo.write(OPEN_ANGLE);
  delay(1000); 

  // Start at closed position
  // Initial State
  myServo.write(CLOSED_ANGLE);
  delay(14000);

  // Open for incoming pickup
  // Sample 3 
  myServo.write(OPEN_ANGLE);
  delay(2000);  // Stay open for 2 seconds

  // Close for movement to position
  // Safe hold -> Move to stage
  myServo.write(CLOSED_ANGLE);
  delay(25000);

  // Open for dropoff
  // Move to stage -> Stage Dip
  myServo.write(OPEN_ANGLE);
  delay(2000);

  // Close to move out
  // -> Back hold position
  myServo.write(CLOSED_ANGLE);
  delay(10000);

  //Open for final deposit
  // Deposit -> Deposit dip
  myServo.write(OPEN_ANGLE);
  delay(1000);

  // Delay for back to start position
  myServo.write(CLOSED_ANGLE);
  delay(12000);
}