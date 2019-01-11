#ifndef gyroFunctions_cpp
#define gyroFunctions_cpp
#include "headers/gyroFunctions.h"

float correctGyroValue(float given)// converts given value to its corresponding positive gyro value
{
	if((given > 3599) || (given < -3599))// if the given values is over 3599 or under -3599
	{
		given = static_cast<int>(given)%3600;// reduce the value to wtihin those limits
	}
	if(given < 0)// if the given value is negative
	{
		given = given + 3600;// convert the given value to a positive gyro values
	}
	return given;// return the fixed gyro value
}

int gyroDirection(float first, float second)// determines if the robot should turn right or left to reach the second value from the first value
{
	second = correctGyroValue(second);// correct the given gyro value
	first = correctGyroValue(first);
	if(first < second)// if the current gyro value is less than the given value
	{
		if((second-first) < ((3600-second) + first))// if the robot should turn right
		{
			return 1;// return 1
		}
		else// if the robot should turn left
		{
			return -1;// return -1
		}
	}
	else if(first > second)// if the current gyro value is greater than the given value
	{
		if((first-second) < (( 3600 - first) + second))// if the robot should turn left
		{
			return -1;// return 1
		}
		else// if the robot should turn right
		{
			return 1;// turn left
		}
	}
	else// if the robot does not need to turn
	{
		return 0;// return 0
	}
}

float gyroDifference(float first, float second)// returns the absolute value of the shortest difference between two gyro values
{
	first = correctGyroValue(first);//correct the first gyro value
	second = correctGyroValue(second);// correct the second gyro value
	if(second < first)// if the second value is less than the first value
	{
		if((first-second) < ((3600-first) + second))// if first-second is the shortest path
		{
			return (first-second);// return first-second
		}
		else// otherwise
		{
			return ((3600-first) + second);// if 3600-first+second is the shortest path
		}
	}
	else if(second > first)// if the second value is greater than the first
	{
		if((second-first) < (( 3600 - second) + first))// if second-first is the shortest path
		{
			return (second-first);// return second-first
		}
		else// otherwise
		{
			return (( 3600 - second) + first);// if 3600-second+first is the shortest path
		}
	}
	else// if the distance is zero
	{
		return 0;// return 0
	}
}

#endif
