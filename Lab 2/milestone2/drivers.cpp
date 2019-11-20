#include <Arduino.h>
#include <Servo.h>

#include "drivers.hpp"

// allocate servo structures
static Servo servo_left;
static Servo servo_right;

static const int RED_LED_PIN = 9; // red indicator pin
static const int GREEN_LED_PIN = 8; // green indicator pin
static const int BLUE_LED_PIN = 13; // blue indicator pin
static const int LINE_LEFT_PIN = 4; // left line sensor input pin
static const int LINE_RIGHT_PIN = 7; // right line sensor input pin
static const int LINE_REAR_PIN = 2; // rear line sensor input pin (used for intersection detection)
static const int DISTANCE_FRONT_PIN = A1; // front distance sensor pin
static const int DISTANCE_RIGHT_PIN = A2; // right distance sensor pin

static const int LEFT_SERVO_PIN = 6; // left servo control pin
static const int RIGHT_SERVO_PIN = 5; // right servo control pin 

void driversTest() {
  Serial.println("Drivers Test");
  Serial.println();

  // test leds
  /*redLEDOn();
  greenLEDOn();
  blueLEDOn();*/

  while(1) {
    // test line sensors
    Serial.print("Left Line Sensor: ");
    Serial.println(checkLineLeft());
    Serial.print("Right Line Sensor: ");
    Serial.println(checkLineRight());
    Serial.print("Rear Line Sensor: ");
    Serial.println(checkLineRear());
    Serial.print("Front Distance Sensor: ");
    Serial.println(getDistanceFront());
    Serial.println();
    delay(1000);
  }

  // test servos
  /*setSpeedLeft(100);
  setSpeedRight(100);*/
}

void ledInit()
{
  pinMode(RED_LED_PIN, OUTPUT); // make red indicator pin an output
  pinMode(GREEN_LED_PIN, OUTPUT); // make green indicator pin an output
  pinMode(BLUE_LED_PIN, OUTPUT); // make blue indicator pin an output
}

void redLEDOn() {
  digitalWrite(RED_LED_PIN, HIGH); // LED active high
}

void greenLEDOn() {
  digitalWrite(GREEN_LED_PIN, HIGH); // LED active high
}

void blueLEDOn() {
  digitalWrite(BLUE_LED_PIN, HIGH); // LED active high
}

void redLEDOff() {
  digitalWrite(RED_LED_PIN, LOW); // LED active high
}

void greenLEDOff() {
  digitalWrite(GREEN_LED_PIN, LOW); // LED active high
}

void blueLEDOff() {
  digitalWrite(BLUE_LED_PIN, LOW); // LED active high
}

bool checkLineLeft() {
  return !digitalRead(LINE_LEFT_PIN); // white line detection
}

bool checkLineRight() {
  return !digitalRead(LINE_RIGHT_PIN); // white line detection
}

bool checkLineRear() {
  return !digitalRead(LINE_REAR_PIN); // white line detection
}

float getDistanceFront() {
  return 2457.6f / analogRead(DISTANCE_FRONT_PIN) - 0.42f; // transfer function obtained from datasheet
}

float getDistanceRight() {
  return 2457.6f / analogRead(DISTANCE_RIGHT_PIN) - 0.42f; // transfer function obtained from datasheet
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

