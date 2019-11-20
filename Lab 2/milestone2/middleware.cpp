#include <Arduino.h>
#include "middleware.hpp"
#include "drivers.hpp"

static const int8_t MAX_SPEED = 100; // maximum servo speed
static const int8_t TURN_SPEED = 100; // turn in place speed
static const int8_t LINE_CORRECTION_FACTOR = 3; // correct line deviation by lowering inner servo speed this many times
static const uint32_t MIN_ADVANCE_TIME = 500; // start looking for next intersection after advancing for this much time
static const uint32_t MIN_TURN_TIME = 200; // start looking for new line after turning for this much time
static const uint32_t NAV_UPDATE_TIME = 10; // handle navigation tasks at this interval
static const float GRID_UNIT_LEN = 30.48f; // length of one grid square in cm

void middlewareTest() {
  Serial.println("Middleware Test");

  while(1) {
    Serial.print("Wall in front?: ");
    Serial.println(checkWallFront());
    Serial.println();
    delay(1000);
  }
}

void advanceOne() {
  uint32_t current_time = millis(); // update current_time
  uint32_t nav_start_time = current_time; // time when robot began moving forward
  uint32_t last_nav_update_time = 0; // last time robot handled navigation

  while(1) {
    current_time = millis(); // update current_time
    
    if(current_time - last_nav_update_time > NAV_UPDATE_TIME) { // handle navigation at predefined intervals to avoid unexpected effects from cpu loading
      last_nav_update_time = current_time;
      
      if(current_time - nav_start_time > MIN_ADVANCE_TIME) { // check for new intersection after moving away from current one
        if(checkLineRear()) { // exit if robot is at new intersection
          break;
        }
      }
      
      if(checkLineLeft() && checkLineRight()) { // check for perfect line alignment
        // just advance full speed
        setSpeedLeft(MAX_SPEED);
        setSpeedRight(MAX_SPEED);
      }
      else if(checkLineLeft()) { // otherwise, check for deviation to the right
        // turn left while continuing to advance
        setSpeedLeft(MAX_SPEED / LINE_CORRECTION_FACTOR);
        setSpeedRight(MAX_SPEED);
      }
      else if(checkLineRight()){ // otherwise, check for deviation to the left
        // turn right while continuing to advance
        setSpeedLeft(MAX_SPEED);
        setSpeedRight(MAX_SPEED / LINE_CORRECTION_FACTOR);
      }
    }
  }

  halt(); // halt at new intersection
}

void turnLeft() { // turn left in place
  uint32_t current_time = millis(); //update current_time
  uint32_t nav_start_time = current_time; // time when robot began turning
  uint32_t last_nav_update_time = 0; // last time robot handled navigation

  while(1) {
    current_time = millis();
    
    if(current_time - last_nav_update_time > NAV_UPDATE_TIME) { // handle navigation at predefined intervals to avoid unexpected effects from cpu loading
      last_nav_update_time = current_time;
      
      if(current_time - nav_start_time > MIN_TURN_TIME) { // check for new line after turning away from current one
        if(checkLineLeft() || checkLineRight()) { // exit if robot is at new line
          break;
        }
      }

      // turn left in place
      setSpeedRight(TURN_SPEED);
      setSpeedLeft(-TURN_SPEED);
    }
  }

  halt(); // halt at new line
}

void turnRight() { // turn right in place
  uint32_t current_time = millis(); //update current_time
  uint32_t nav_start_time = current_time; // time when robot began turning
  uint32_t last_nav_update_time = 0; // last time robot handled navigation

  while(1) {
    current_time = millis();
    
    if(current_time - last_nav_update_time > NAV_UPDATE_TIME) { // handle navigation at predefined intervals to avoid unexpected effects from cpu loading
      last_nav_update_time = current_time;
      
      if(current_time - nav_start_time > MIN_TURN_TIME) { // check for new line after turning away from current one
        if(checkLineLeft() || checkLineRight()) { // exit if robot is at new line
          break;
        }
      }

      // turn right in place
      setSpeedRight(-TURN_SPEED);
      setSpeedLeft(TURN_SPEED);
    }
  }

  halt(); // halt at new line
}

void halt() {
  // set both servos to 0 speed
  setSpeedRight(0);
  setSpeedLeft(0);
}

bool checkWallFront() {
  // robot is facing a wall if it sees something closer than half the grid unit length
  return getDistanceFront() < GRID_UNIT_LEN / 2.0f;
}

bool checkWallRight() {
  // robot has a wall to the right if it sees something closer than half the grid unit length
  return getDistanceRight() < GRID_UNIT_LEN / 2.0f;
}
