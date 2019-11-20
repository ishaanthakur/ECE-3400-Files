void middlewareTest(); // test middleware functionality

int moveForward(); // advance one grid unit along the line. Non-blocking - Call at 10 ms intervals until return result is 1. Must be aligned to grid when first called.
int turnLeft(); // nonblocking turn left in place at an intersection. Non-blocking - Call at 10 ms intervals until return result is 1. Must be at intersection when first called.
int turnRight(); // turn right in place at an intersection. Non-blocking - Call at 10 ms intervals until return result is 1. Must be at intersection when first called.
void halt(); // stop the servos
