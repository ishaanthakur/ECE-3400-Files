/*Go forward at low speed*/

#include <Servo.h>

Servo servo_left;
Servo servo_right;

const int LEFT_SERVO_PIN = 5;
const int RIGHT_SERVO_PIN = 6;

// setup servos
void servoSetup() {
  // associate servo structures with corresponding PWM pins
  servo_left.attach(LEFT_SERVO_PIN);
  servo_right.attach(RIGHT_SERVO_PIN);

  // stop robot upon startup
  servo_left.write(90);
  servo_right.write(90);
}

// go forward with both servos at equal speed
void go() {
  // speed is 10
  servo_left.write(100); // 90 + 10
  servo_right.write(80); // 90 - 10
}

// the setup function runs once when you press reset or power the board
void setup() {
  servoSetup();
  go();
}

// the loop function runs over and over again forever
void loop() {
}
