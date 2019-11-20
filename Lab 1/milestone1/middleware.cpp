#include <Arduino.h>
#include "middleware.hpp"
#include "drivers.hpp"

static const int8_t MAX_SPEED = 100; // maximum servo speed
static const int8_t TURN_SPEED = 100; // turn in place speed
static const int8_t LINE_CORRECTION_FACTOR = 3; // correct line deviation by lowering inner servo speed this many times
static const uint32_t MIN_ADVANCE_TIME = 500; // start looking for next intersection after advancing for this much time
static const uint32_t MIN_TURN_TIME = 200; // start looking for new line after turning for this much time

static int8_t nav_mode = -1; // navigation mode (i.e. advancing/turning)
static uint32_t nav_start_time = 0; // time elapsed since robot started latest navigation task

void middlewareTest() {
  Serial.println("Middleware Test");

  // test line following
  /*while(true)
  {
    moveForward();
    delay(100);
  }*/
}

int moveForward() {
  uint32_t current_time = millis(); //update current_time

  // update nav_start_time if this is the first call
  if(nav_mode != 0)
  {
    nav_start_time = current_time;
    nav_mode = 0;
  }

  if(current_time - nav_start_time > MIN_ADVANCE_TIME) { // check for new intersection after moving away from current one
    if(checkLineRear()) {
      halt(); // halt at new intersection
      return 1;
    }
  }
  if(checkLineLeft() && checkLineRight()) { // check for perfect line alignment
    // just advance full speed
    setSpeedLeft(MAX_SPEED);
    setSpeedRight(MAX_SPEED);
    
    return 0;
  }
  if(checkLineLeft()) { // otherwise, check for deviation to the right
    // turn left while continuing to advance
    setSpeedLeft(MAX_SPEED / LINE_CORRECTION_FACTOR);
    setSpeedRight(MAX_SPEED);
  }
  else { // otherwise, check for deviation to the left
    // turn right while continuing to advance
    setSpeedLeft(MAX_SPEED);
    setSpeedRight(MAX_SPEED / LINE_CORRECTION_FACTOR);
  }

  return 0;
}

int turnLeft() { // turn left in place
  uint32_t current_time = millis(); //update current_time

  // update nav_start_time if this is the first call
  if(nav_mode != 1)
  {
    nav_start_time = current_time;
    nav_mode = 1;
  }

  if(current_time - nav_start_time > MIN_TURN_TIME) { // check for new line after turning away from current one
    if(checkLineLeft() || checkLineRight()) {
      halt(); // halt at new line
      return 1;
    }
  }

  // turn left in place
  setSpeedRight(TURN_SPEED);
  setSpeedLeft(-TURN_SPEED);
  
  return 0;
}

int turnRight()
{
  uint32_t current_time = millis(); //update current_time

  // update nav_start_time if this is the first call
  if(nav_mode != 2)
  {
    nav_start_time = current_time;
    nav_mode = 2;
  }
  
  if(current_time - nav_start_time > MIN_TURN_TIME) { // check for new line after turning away from current one
    if(checkLineLeft() || checkLineRight()) {
      halt(); // halt at new line
      return 1;
    }
  }

  // turn right in place
  setSpeedRight(-TURN_SPEED);
  setSpeedLeft(TURN_SPEED);
  
  return 0;
}

void halt()
{
  nav_mode = -1;

  // stop servos
  setSpeedLeft(0);
  setSpeedRight(0);
}
