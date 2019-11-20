#include "drivers.hpp"
#include "middleware.hpp"

static const uint32_t NAV_UPDATE_TIME = 10; // call navigation middleware at this interval

static uint32_t last_nav_update_time = 0; // last time navigation middleware was called
static int8_t mode = 0; // FSM state for figure 8 (i.e. advancing/turning left/turning right)
static uint8_t intersection_count = 0; // intersection count in figure 8

void setup() {
  Serial.begin(9600); // initialize serial comms
  
  servoInit(); // initialize servos
}

void loop() {
  uint32_t current_time = millis(); // update current_time
   
  if(current_time - last_nav_update_time > NAV_UPDATE_TIME) { // handle navigation 
    switch(mode) { // navigation FSM
      case 0: { // advancing along line state
        if(moveForward() == 1) { // call middleware. returns 1 when new intersection reached
          intersection_count++;
          if(intersection_count == 8) { // robot is back where it started
            intersection_count = 0;
          }
          
          if(intersection_count == 0 || intersection_count == 1 || intersection_count == 6 || intersection_count == 7) { // turn right at these intersections 
            mode = 1;
          }
          else { // turn left at the others
            mode = 2;
          }
        }

        break;
      }

      case 1: { // turning right state
        if(turnRight() == 1) { // call middleware. returns 1 when aligned to new line
          mode = 0; // start advancing
        }

        break;
      }

      case 2: { // turning left state
        if(turnLeft() == 1) { // call middleware. returns 1 when aligned to new line
          mode = 0; // start advancing
        }

        break;
      }

      default: {
        break;
      }
    }
  }
}
