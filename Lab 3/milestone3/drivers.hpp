void driversTest(); // test driver functionality

void ledInit(); // initialize LEDs
void redLEDOn(); // turn on red LED
void greenLEDOn(); // turn on green LED
void blueLEDOn(); // turn on blue LED
void redLEDOff(); // turn off red LED
void greenLEDOff(); // turn off green LED
void blueLEDOff(); // turn off blue LED
void redLEDToggle(); // toggle red LED
void greenLEDToggle(); // toggle green LED
void blueLEDToggle(); // toggle blue LED

bool checkLineLeft(); // get state of left line sensor
bool checkLineRight(); // get state of right line sensor
bool checkLineRear(); // get state of rear line sensor

float getDistanceFront(); // get front distance sensor reading in cm
float getDistanceLeft(); // get left distance sensor reading in cm
float getDistanceRight(); // get right distance sensor reading in cm

void irInit(); // initialize IR lights
void irOn(); // turn on IR lights
void irOff(); // turn off IR lights
uint16_t getIRFront(); // get front IR sensor reading in arbitrary units in the interval [0, 1023]
uint16_t getIRLeft(); // get left IR sensor reading in arbitrary units in the interval [0, 1023]
uint16_t getIRRight(); // get right IR sensor reading in arbitrary units in the interval [0, 1023]

void servoInit(); // initialize servos
void setSpeedLeft(int8_t spd); // set left servo speed. range: [-100, 100]
void setSpeedRight(int8_t spd); // set right servo speed. range: [-100, 100]

float adcBeepInit(); // initialize ADC for recording sound. return sampling rate
void adcAnalogReadInit(); // initialize ADC for the Arduino analogRead() function
bool adcSampleAvailable(); // check whether a new ADC result is available
uint16_t getADCSample(); // get the latest ADC result;
