void driversTest(); // test driver functionality

void ledInit(); // initialize LEDs
void redLEDOn(); // turn on red LED
void greenLEDOn(); // turn on green LED
void blueLEDOn(); // turn on blue LED
void redLEDOff(); // turn off red LED
void greenLEDOff(); // turn off green LED
void blueLEDOff(); // turn off blue LED

bool checkLineLeft(); // get state of left line sensor
bool checkLineRight(); // get state of right line sensor
bool checkLineRear(); // get state of rear line sensor

float getDistanceFront(); // get front distance sensor reading in cm
float getDistanceRight(); // get right distance sensor reading in cm

void servoInit(); // initialize servos
void setSpeedLeft(int8_t spd); // set left servo speed. range: [-100, 100]
void setSpeedRight(int8_t spd); // set right servo speed. range: [-100, 100]
