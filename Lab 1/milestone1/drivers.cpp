#include <Arduino.h>
#include <Servo.h>

#include "drivers.hpp"

// allocate servo structures
static Servo servo_left;
static Servo servo_right;

static const int LINE_LEFT_PIN = A0; // left line sensor input pin
static const int LINE_RIGHT_PIN = A1; // right line sensor input pin
static const int LINE_REAR_PIN = A2; // rear line sensor input pin (used for intersection detection)
static const unsigned int LINE_THRESHOLD_FRONT = 425; // reflectivity threshold for line detection - front sensors
static const unsigned int LINE_THRESHOLD_REAR = 700; // reflectivity threshold for line detection - rear sensor

static const int LEFT_SERVO_PIN = 6; // left servo control pin
static const int RIGHT_SERVO_PIN = 5; // right servo control pin 

void driversTest() {
  Serial.println("Drivers Test");

  // test servos
  /*setSpeedLeft(100);
  setSpeedRight(100);*/

  while(1) {
    // test line sensors
    Serial.print("Left Line Sensor: ");
    Serial.println(analogRead(LINE_LEFT_PIN));
    Serial.print("Right Line Sensor: ");
    Serial.println(analogRead(LINE_RIGHT_PIN));
    Serial.print("Rear Line Sensor: ");
    Serial.println(analogRead(LINE_REAR_PIN));
    delay(1000);
  }
}

bool checkLineLeft() {
  return analogRead(LINE_LEFT_PIN) < LINE_THRESHOLD_FRONT; // white line detection
}

bool checkLineRight() {
  return analogRead(LINE_RIGHT_PIN) < LINE_THRESHOLD_FRONT; // white line detection
}

bool checkLineRear() {
  return analogRead(LINE_REAR_PIN) < LINE_THRESHOLD_REAR; //white line detection
}

void servoInit() {
  // associate servo structures with corresponding control pins
  servo_left.attach(LEFT_SERVO_PIN);
  servo_right.attach(RIGHT_SERVO_PIN);

  // halt the motors on startup
  setSpeedLeft(0);
  setSpeedRight(0);
}

void setSpeedLeft(int8_t spd) {
  // map [-100, 100] -> [1300, 1700]
  servo_left.writeMicroseconds(1500 + (int16_t)spd * 2);
}

void setSpeedRight(int8_t spd) {
  // map [-100, 100] -> [1300, 1700]
  servo_right.writeMicroseconds(1500 - (int16_t)spd * 2);
}
