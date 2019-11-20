void middlewareTest(); // test middleware functionality

void advanceOne(); // advance one grid unit (blocking)
void turnLeft(); // turn left at an intersection (blocking)
void turnRight(); // turn right at an intersection (blocking)
void halt(); // stop the robot

bool checkWallFront(); // check whether there is wall in front of the robot (must be at an intersection)
bool checkWallRight(); // check whether there is wall to the right of the robot (must be at an intersection)
