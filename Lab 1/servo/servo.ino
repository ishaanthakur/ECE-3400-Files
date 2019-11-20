/* Test servo control by setting speed according to the position of a trimmer */

#include <Servo.h>

Servo servo;

const int ANALOG_INPUT_PIN = A0;
const int SERVO_PIN = 5;

// setup servo
void servoSetup() {
  // associate servo structure with corresponding PWM pin
  servo.attach(SERVO_PIN);

  // stop servo upon startup
  servo.write(90);
}

// the setup function runs once when you press reset or power the board
void setup() {
  servoSetup();
}

// the loop function runs over and over again forever
void loop() {
  unsigned int reading = analogRead(ANALOG_INPUT_PIN); // take a reading
  servo.write(reading / 6); // set servo speed. divide by 6 to map 0-1023 -> 0 - 180
}
