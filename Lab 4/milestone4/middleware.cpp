/* Middleware code */

#include <Arduino.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "middleware.hpp"
#include "drivers.hpp"

static const uint8_t RF_PKT_SIZE = 26; // radio packet payload size
static const uint8_t NUM_PKTS = 4; // number of packets in a complete transmission
static const uint8_t CH_FREQ = 86; // radio module channel frequency offset (in MHz)
const uint64_t pipes[2] = {0x000000000042, 0x000000000043}; // our radio pipe addresses

static const float BEEP_FREQ = 950.0f; // start signal frequency
static const float BEEP_THRESHOLD = 20000.0f; // threshold for note detection
static const uint32_t NUM_TRIGGER_RESULTS = 3; // number of consecutive results above the threshold required for triggering
static const uint16_t DFT_N = 500; // DFT length (in samples)

static const int8_t MAX_SPEED = 100; // maximum servo speed
static const int8_t TURN_SPEED = 50; // turn in place speed
static const int8_t BACK_UP_SPEED = 50; // robot avoidance backing up speed
static const uint8_t SPEED_RAMP_TERM = 3; // acceleration when traveling between nodes
static const float LINE_CORRECTION_FACTOR = 1.5f; // correct line deviation by lowering inner servo speed this many times
static const uint32_t MIN_ADVANCE_TIME = 500; // start looking for next intersection after advancing for this much time
static const uint32_t MIN_TURN_TIME = 300; // start looking for new line after turning for this much time
static const uint32_t NAV_UPDATE_TIME = 5; // handle navigation tasks at this interval
static const uint32_t TRANSMIT_INTERVAL = 20; // transmit the packets in a frame at this interval
static const uint32_t BACK_UP_RECOVER_TIME = 200; // move forward for this much time after backing up to get back to the node
static const uint16_t IR_THRESHOLD = 400; // threshold for robot detection
static const float GRID_UNIT_LEN = 30.48f; // length of one grid square in cm

static uint16_t ir_front_base = 0; // baseline IR reading for front sensor
static uint16_t ir_left_base = 0; // baseline IR reading for left sensor
static uint16_t ir_right_base = 0; // baseline IR reading for right sensor

// remember last sensor(s) that saw the line so the robot can start in the right direction after stopping
static bool last_seen_left = 0;
static bool last_seen_right = 0;

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
}

void rfInit(RF24 *radio) {
  radio->begin(); // begin operation of the radio module 
  
  radio->setRetries(15, 15); // set delay between retries and number of retries
  radio->setAutoAck(true); // turn on the Auto Acknowledge feature
  radio->setChannel(CH_FREQ); // set the frequency to (2400 + CH_FREQ) MHz
  radio->setPALevel(RF24_PA_MAX); // set power level
  radio->setDataRate(RF24_250KBPS); // set data rate (in kb/s)
  radio->setPayloadSize(RF_PKT_SIZE); // set packet payload to RF_PKT_SIZE
  
  radio->openWritingPipe(pipes[0]); // open a writing pipe
  radio->openReadingPipe(1, pipes[1]); // open a reading pipe
}

void transmitPacket(RF24 *radio, char *data, uint8_t pkt_num) {
  char tx_buff[RF_PKT_SIZE]; // payload buffer

  radio->startListening(); // for some undetermined reason, radio only works if we start and stop listening before writing each packet
  memcpy(tx_buff, data + pkt_num * (RF_PKT_SIZE - 1), RF_PKT_SIZE - 1); // copy a chunk of the maze data into the payload buffer
  tx_buff[RF_PKT_SIZE - 1] = pkt_num; // mark the packet with its index in the frame (last byte of the packet)
  radio->stopListening();
  radio->startWrite(tx_buff, RF_PKT_SIZE); // send the packet
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

void checkLines() {
  // update which sensor(s) saw the line last
  if(checkLineLeft() && checkLineRight()) {
    last_seen_left = 1;
    last_seen_right = 1;
  }
  else if(checkLineLeft()) {
    last_seen_left = 1;
    last_seen_right = 0;
  }
  else if(checkLineRight()) {
    last_seen_left = 0;
    last_seen_right = 1;
  }  
}

int8_t advanceOne(RF24 *radio, char *data) {
  int8_t ret = 0; // return success by default
  uint32_t current_time = millis(); // time since startup in ms
  uint32_t nav_start_time = current_time; // time when robot began moving forward
  uint32_t last_nav_update_time = 0; // last time robot handled navigation
  uint32_t last_transmit_time = 0; // last time robot transmitted a radio packet
  uint8_t num_pkts_transmitted = 0; // number of radio packets transmitted
  int8_t spd = 0; // current speed

  while(1) {
    current_time = millis(); // update current_time

    if(num_pkts_transmitted < NUM_PKTS) { // if robot still needs to transmit packets...
      if(current_time - last_transmit_time > TRANSMIT_INTERVAL) { // if a certain interval has pased since it last transmitted a packet...
        last_transmit_time = current_time; // update last time a packet was transmitted
        
        transmitPacket(radio, data, num_pkts_transmitted++); // transmit a packet to the base station
      }
    }
    
    if(current_time - last_nav_update_time > NAV_UPDATE_TIME) { // handle navigation at predefined intervals to avoid unexpected effects from cpu loading
      last_nav_update_time = current_time; // update last time navigation was handled

      // need to ramp speed in order to prevent wheelie
      if(spd < MAX_SPEED) { // if robot still hasn't reached full speed
        spd += SPEED_RAMP_TERM; // accelerate
      }

      if(checkRobot()) { // if there's a robot blocking the way...
        ret = 1; // return failure
        Serial.println("Robot detected");
        
        break; // stop advancing
      }
      
      if(current_time - nav_start_time > MIN_ADVANCE_TIME) { // check for new intersection after moving away from current one
        if(checkLineRear()) { // if new node reached...
          break; // stop advancing
        }
      }
      
      checkLines(); // get line info

      // apply correction or advance straight depending on position relative to line
      setSpeedLeft(last_seen_right ? spd : (float)spd / LINE_CORRECTION_FACTOR);
      setSpeedRight(last_seen_left ? spd : (float)spd / LINE_CORRECTION_FACTOR);
    }
  }

  halt(); // halt at new intersection

  return ret;
}

void backUp() {
  uint32_t current_time = millis(); // time since startup in ms
  uint32_t recover_start_time; // time since robot starting moving forwrad again to reacquire the node
  uint32_t last_nav_update_time = 0; // last time robot handled navigation
  int8_t spd = 0; // current speed

  // move backward
  while(1) {
    current_time = millis(); // update current_time
    
    if(current_time - last_nav_update_time > NAV_UPDATE_TIME) { // handle navigation at predefined intervals to avoid unexpected effects from cpu loading
      last_nav_update_time = current_time; // update last time navigation was handled

      // need to ramp speed in order to prevent wheelie
      if(spd < BACK_UP_SPEED) { // if robot still hasn't reached full speed
        spd += SPEED_RAMP_TERM; // accelerate
      }
      
      if(checkLineRear()) { // if the rear sensor sees the line...
        break; // stop moving backward
      }
            
      checkLines(); // get line info

      // apply correction or back up straight depending on position relative to line
      setSpeedLeft(last_seen_right ? (float)(-spd) / LINE_CORRECTION_FACTOR : -spd);
      setSpeedRight(last_seen_left ? (float)(-spd) / LINE_CORRECTION_FACTOR : -spd);
    }
  }

  spd = 0; // reset the speed to zero
  recover_start_time = current_time; // remember when robot started moving forward again

  // move forward for a certain amount of time to reacquire the node
  while(1) {
    current_time = millis(); // update current_time
    
    if(current_time - last_nav_update_time > NAV_UPDATE_TIME) { // handle navigation at predefined intervals to avoid unexpected effects from cpu loading
      last_nav_update_time = current_time; // update last time navigation was handled

      // need to ramp speed in order to prevent wheelie
      if(spd < BACK_UP_SPEED) { // if robot still hasn't reached full speed
        spd += SPEED_RAMP_TERM; // accelerate
      }

      if(current_time - recover_start_time > BACK_UP_RECOVER_TIME) { // if the predefined time for moving forward has elapsed...
        break; // stop moving forward
      }
            
      checkLines(); // get line info

      // apply correction or advance straight depending on position relative to line
      setSpeedLeft(last_seen_right ? (float)(spd) / LINE_CORRECTION_FACTOR : spd);
      setSpeedRight(last_seen_left ? (float)(spd) / LINE_CORRECTION_FACTOR : spd);
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
        if(checkLineLeft() && checkLineRight()) { // if new line seen...
          break; // stop turning
        }
      }

      checkLines(); // update which sensor(s) saw the line last

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
        if(checkLineLeft() || checkLineRight()) { // if new line seen...
          break; // stop turning
        }
      }

      checkLines(); // update which sensor(s) saw the line last
    
      // turn right in place
      setSpeedLeft(TURN_SPEED);
      setSpeedRight(-TURN_SPEED);
    }
  }

  halt(); // halt at new line
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
  while(num_consec_high_results < NUM_TRIGGER_RESULTS && !checkButt()) { // trigger when robot gets "NUM_TRIGGER_RESULTS" bins above the threshold or the override button is pressed
    // implementation of the Goertzel Algorithm for computing a specific DFT bin
    float s1 = 0;
    float s2 = 0;

    for(uint16_t sample_num = 0; sample_num < DFT_N; sample_num++) { // feed DFT_N samples through the algorithm
      while(!adcSampleAvailable()); // wait for a new sample to be available
      
      float sample = getADCSample(); // get the new sample

      // compute the IIR filter result
      float s0 = sample + 2.0f * real_coeff * s1 - s2;
      s2 = s1;
      s1 = s0;
    }
    
    float real = real_coeff * s1 - s2;
    float imag = imag_coeff * s1;
    float result = sqrt(real * real + imag * imag); // take the magnitude of the DFT bin

    Serial.println(result);
    
    if(result > BEEP_THRESHOLD) { // check whether the bin is above the threshold 
      num_consec_high_results++; // increment counter if yes
    }
    else {
      num_consec_high_results = 0; // reset counter if no
    }
  }
  sei(); // turn interrupts back on
}
