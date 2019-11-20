#include "drivers.hpp"
#include "middleware.hpp"

void setup() {
  Serial.begin(9600); // initialize serial comms

  ledInit(); // initialize indicators
  servoInit(); // initialize servos

  //driversTest();
}

void loop() {
  if(checkWallRight()) { // there's a wall on the right
    if(checkWallFront()) { // ...but there's also a wall blocking the path in front
      redLEDOn(); // light red indicator
      turnLeft(); // turn left
      redLEDOff();
    }
    else { // all is fine, robot can go forward
    greenLEDOn(); // light green indicator
    advanceOne(); // move forward
    greenLEDOff();
    }
  }
  else { // no wall on the right
    blueLEDOn(); // light blue indicator
    turnRight(); // turn right
    advanceOne(); // move forward
    blueLEDOff();
  }
}
