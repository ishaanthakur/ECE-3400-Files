void middlewareTest(); // test middleware functionality

void calibrateIR();
bool checkWallFront(); // check whether there is wall in front of the robot (must be at an intersection)
bool checkWallLeft(); // check whether there is wall to the left of the robot (must be at an intersection)
bool checkWallRight(); // check whether there is wall to the right of the robot (must be at an intersection)

int8_t advanceOne(); // advance one grid unit (blocking)
void backUp(); // reverse to the previous intersection (blocking)
void turnLeft(); // turn left at an intersection (blocking)
void turnRight(); // turn right at an intersection (blocking)
void victoryDance(); // flex on losing teams
void halt(); // stop the robot

void waitForStart(); // wait for the acoustic start signal
