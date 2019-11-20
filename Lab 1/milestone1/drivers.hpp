void driversTest(); // test driver functionality

bool checkLineLeft(); // get state of left line sensor
bool checkLineRight(); // get state of right line sensor
bool checkLineRear(); // get state of rear line sensor

void servoInit(); // initialize servos
void setSpeedLeft(int8_t spd); // set left servo speed. range: [-100, 100]
void setSpeedRight(int8_t spd); // set right servo speed. range: [-100, 100]
