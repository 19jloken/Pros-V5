#ifndef gyroFunctions_h
#define gyroFunctions_h

float correctGyroValue(float given);// converts given value to its corresponding positive gyro value
int gyroDirection(int first, int second);// determines if the robot should turn right or left to reach its target gyro value
float gyroDifference(float first, float second);// returns the absolute value of the shortest difference between two gyro values

#endif
