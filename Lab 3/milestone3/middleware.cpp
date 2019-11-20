/* Middleware code */

#include <Arduino.h>
#include "middleware.hpp"
#include "drivers.hpp"

static const float BEEP_FREQ = 950.0f; // start signal frequency
static const float BEEP_THRESHOLD = 40000.0f; // threshold for note detection
static const uint32_t NUM_TRIGGER_RESULTS = 10; // number of consecutive results above the threshold required for triggering
static const uint16_t DFT_N = 1000; // DFT length

static const int8_t MAX_SPEED = 100; // maximum servo speed
static const int8_t TURN_SPEED = 100; // turn in place speed
static const int8_t LINE_CORRECTION_FACTOR = 3; // correct line deviation by lowering inner servo speed this many times
static const uint32_t MIN_ADVANCE_TIME = 500; // start looking for next intersection after advancing for this much time
static const uint32_t MIN_TURN_TIME = 200; // start looking for new line after turning for this much time
static const uint32_t DANCE_TIME = 5000; // victory dance duration
static const uint32_t NAV_UPDATE_TIME = 10; // handle navigation tasks at this interval
static const uint32_t DANCE_UPDATE_TIME = 100; // handle victory dance tasks at this interval
static const uint16_t IR_THRESHOLD = 300; // threshold for robot detection
static const float GRID_UNIT_LEN = 30.48f; // length of one grid square in cm

static uint16_t ir_front_base = 0; // baseline IR reading for front sensor
static uint16_t ir_left_base = 0; // baseline IR reading for left sensor
static uint16_t ir_right_base = 0; // baseline IR reading for right sensor

void middlewareTest() {
  Serial.println("Middleware Test");
  Serial.println();

  // test wall detection
  /*while(1) {
    uint16_t val;
    
    Serial.print("Wall in front?: ");
    val = checkWallFront();
    delay(100);
    Serial.println(val);

    Serial.print("Wall to the left?: ");
    val = checkWallLeft();
    delay(100);
    Serial.println(val);
    
    Serial.print("Wall to the right?: ");
    val = checkWallRight();
    delay(100);
    Serial.println(val);
    
    Serial.println();
    delay(700);
  }*/

  // test robot detection
  /*while(1) {
    Serial.print("Robot in front?: ");
    Serial.println(checkRobot());
    delay(1000);
  }*/

  // test victory dance
  victoryDance();
  while(1);
}

void calibrateIR() {
  ir_front_base = getIRFront(); // get front IR baseline reading
  ir_left_base = getIRLeft(); // get left IR baseline reading
  ir_right_base = getIRRight(); // get right IR baseline reading 
}

// wall if distance smaller than half a grid unit length and no IR signal
bool checkWallFront() {
  return (getDistanceFront() < GRID_UNIT_LEN / 2.0f) && !(getIRFront() < (int16_t)ir_front_base - (int16_t)IR_THRESHOLD);
}

bool checkWallLeft() {
  return (getDistanceLeft() < GRID_UNIT_LEN / 2.0f) && !(getIRLeft() < (int16_t)ir_left_base - (int16_t)IR_THRESHOLD);
}

bool checkWallRight() {
  return (getDistanceRight() < GRID_UNIT_LEN / 2.0f) && !(getIRRight() < (int16_t)ir_right_base - (int16_t)IR_THRESHOLD);
}

bool checkRobot() {
  return getIRFront() < (int16_t)ir_front_base - (int16_t)IR_THRESHOLD; // robot in front if IR reading changed more than the threshold
}

int8_t advanceOne() {
  int8_t ret = 0; // return success by default
  uint32_t current_time = millis(); // time since startup in ms
  uint32_t nav_start_time = current_time; // time when robot began moving forward
  uint32_t last_nav_update_time = 0; // last time robot handled navigation

  while(1) {
    current_time = millis(); // update current_time
    
    if(current_time - last_nav_update_time > NAV_UPDATE_TIME) { // handle navigation at predefined intervals to avoid unexpected effects from cpu loading
      last_nav_update_time = current_time; // update last time navigation was handled

      if(checkRobot()) { // if there's a robot blocking the way...
        ret = 1; // return failure
        Serial.println("Robot detected");
        redLEDOn(); // signal robot detected
        
        break; // stop advancing
      }
      
      if(current_time - nav_start_time > MIN_ADVANCE_TIME) { // check for new intersection after moving away from current one
        if(checkLineRear()) { // exit if robot is at new intersection
          break;
        }
      }
      
      if(checkLineLeft() && checkLineRight()) { // check for perfect line alignment
        // just advance at full speed
        setSpeedLeft(MAX_SPEED);
        setSpeedRight(MAX_SPEED);
      }
      else if(checkLineLeft()) { // otherwise, check for deviation to the right
        // turn left while continuing to advance
        setSpeedLeft(MAX_SPEED / LINE_CORRECTION_FACTOR);
        setSpeedRight(MAX_SPEED);
      }
      else if(checkLineRight()) { // otherwise, check for deviation to the left
        // turn right while continuing to advance
        setSpeedLeft(MAX_SPEED);
        setSpeedRight(MAX_SPEED / LINE_CORRECTION_FACTOR);
      }
    }
  }

  halt(); // halt at new intersection

  return ret;
}

void backUp() {
  uint32_t current_time = millis(); // time since startup in ms
  uint32_t nav_start_time = current_time; // time when robot began reversing
  uint32_t last_nav_update_time = 0; // last time robot handled navigation

  while(1) {
    current_time = millis(); // update current_time
    
    if(current_time - last_nav_update_time > NAV_UPDATE_TIME) { // handle navigation at predefined intervals to avoid unexpected effects from cpu loading
      last_nav_update_time = current_time; // update last time navigation was handled
      
      if(checkLineRear()) { // exit if robot is back at the previous intersection
        redLEDOff(); // turn off the robot detected signal
        
        break;
      }
      
      if(checkLineLeft() && checkLineRight()) { // check for perfect line alignment
        // just reverse at full speed
        setSpeedLeft(-MAX_SPEED);
        setSpeedRight(-MAX_SPEED);
      }
      else if(checkLineLeft()) { // otherwise, check for deviation to the right
        // turn left while continuing to reverse
        setSpeedLeft(-MAX_SPEED / LINE_CORRECTION_FACTOR);
        setSpeedRight(-MAX_SPEED);
      }
      else if(checkLineRight()) { // otherwise, check for deviation to the left
        // turn right while continuing to reverse
        setSpeedLeft(-MAX_SPEED);
        setSpeedRight(-MAX_SPEED / LINE_CORRECTION_FACTOR);
      }
    }
  }

  halt(); // halt at old intersection
}

void turnLeft() { // turn left in place
  uint32_t current_time = millis(); // time since startup in ms
  uint32_t nav_start_time = current_time; // time when robot began turning
  uint32_t last_nav_update_time = 0; // last time robot handled navigation

  while(1) {
    current_time = millis(); // update current_time
    
    if(current_time - last_nav_update_time > NAV_UPDATE_TIME) { // handle navigation at predefined intervals to avoid unexpected effects from cpu loading
      last_nav_update_time = current_time; // update last time navigation was handled
      
      if(current_time - nav_start_time > MIN_TURN_TIME) { // check for new line after turning away from current one
        if(checkLineLeft() || checkLineRight()) { // exit if robot is at new line
          break;
        }
      }

      // turn left in place
      setSpeedLeft(-TURN_SPEED);
      setSpeedRight(TURN_SPEED);
    }
  }

  halt(); // halt at new line
}

void turnRight() { // turn right in place
  uint32_t current_time = millis(); // time since startup in ms
  uint32_t nav_start_time = current_time; // time when robot began turning
  uint32_t last_nav_update_time = 0; // last time robot handled navigation

  while(1) {
    current_time = millis(); // update current_time
    
    if(current_time - last_nav_update_time > NAV_UPDATE_TIME) { // handle navigation at predefined intervals to avoid unexpected effects from cpu loading
      last_nav_update_time = current_time; // update last time navigation was handled
      
      if(current_time - nav_start_time > MIN_TURN_TIME) { // check for new line after turning away from current one
        if(checkLineLeft() || checkLineRight()) { // exit if robot is at new line
          break;
        }
      }

      // turn right in place
      setSpeedLeft(TURN_SPEED);
      setSpeedRight(-TURN_SPEED);
    }
  }

  halt(); // halt at new line
}

void victoryDance() {
  uint32_t current_time = millis(); // time since startup in ms
  uint32_t dance_start_time = current_time; // time when robot began dancing
  uint32_t last_dance_update_time = 0; // last time robot handled dancing

  randomSeed(analogRead(0)); // get a seed for randomly lighting indicators

  while(1) {
    current_time = millis(); // update current_time
    
    if(current_time - last_dance_update_time > DANCE_UPDATE_TIME) { // handle dancing at predefined intervals to avoid unexpected effects from cpu loading
      last_dance_update_time = current_time; // update last time dancing was handled
      
      if(current_time - dance_start_time > DANCE_TIME) { // exit if robot danced enough
          break;
      }

      // turn left at full speed
      setSpeedLeft(-MAX_SPEED);
      setSpeedRight(MAX_SPEED);

      // light indicators randomly
      if(random(2)) {
        redLEDOn();
      }
      else {
        redLEDOff();
      }
      if(random(2)) {
        greenLEDOn();
      }
      else {
        greenLEDOff();
      }
      if(random(2)) {
        blueLEDOn();
      }
      else {
        blueLEDOff();
      }
    }
  }

  halt(); // stop after dancing enough

  // turn off all indicators
  redLEDOff();
  greenLEDOff();
  blueLEDOff();
}

void halt() {
  // set both servos to 0 speed
  setSpeedLeft(0);
  setSpeedRight(0);
}

void waitForStart() {
  uint16_t num_consec_high_results = 0; // number of results above the threshold

  float beep_sampling_freq = adcBeepInit(); // initialize ADC for recording sound and get sampling rate;
  float w0 = 2.0f * (float)PI * (float)(round(BEEP_FREQ / beep_sampling_freq * float(DFT_N))) / (float)DFT_N; // normalized frequency of bin closest to signal of interest.

  float real_coeff = cos(w0); // pre-compute coefficient needed for Goertzel Algorithm
  float imag_coeff = sin(w0); // pre-compute coefficient needed for Goertzel Algorithm

  cli(); // prevent interrupts from interfering with the sampling process
  while(num_consec_high_results < NUM_TRIGGER_RESULTS) { // trigger when robot gets "NUM_TRIGGER_RESULTS" bins above the threshold
    // implementation of the Goertzel Algorithm for computing a specific DFT bin
    float s1 = 0;
    float s2 = 0;

    for(uint16_t sample_num = 0; sample_num < DFT_N; sample_num++) { // feed DFT_N samples through the algorithm
      while(!adcSampleAvailable()) {
      }
      
      float sample = getADCSample();

      float s0 = sample + 2.0f * real_coeff * s1 - s2;
      s2 = s1;
      s1 = s0;
    }
    
    float real = real_coeff * s1 - s2;
    float imag = imag_coeff * s1;
    float result = sqrt(real * real + imag * imag); // take the magnitude of the DFT bin
    
    if(result > BEEP_THRESHOLD) { // check whether the bin is above the threshold 
      num_consec_high_results++; // increment counter if yes
    }
    else {
      num_consec_high_results = 0; // reset counter if no
    }
  }
  sei(); // turn interrupts back on
}
