#ifndef motorSlew_cpp
#define motorSlew_cpp

#include "headers/motorSlew.h"
#include "headers/timers.h"
#include "pros/rtos.hpp"

#include "pros/llemu.hpp"

pros::Task slewMotorsTask(slewMotors, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "slewMotorsTask");

pros::Motor *motorArray [TOTAL_MOTORS];
driveMotor driveMotors[TOTAL_MOTORS];

pros::Motor portOne (1);
pros::Motor portTwo (2);
pros::Motor portThree (3);
pros::Motor portFour (4);
pros::Motor portFive (5);
pros::Motor portSix (6);
pros::Motor portSeven (7);
pros::Motor portEight (8);

void addMotor(int name, int slew)
{
	driveMotor *x = &(driveMotors[name]);
	x->requestedSpeed = 0;
	x->currentSpeed = 0;
	x->slewRate = slew;
	x->shouldSlew = true;
}

void setMotor(int name, int speed)
{
	driveMotor *x = &(driveMotors[name]);
	x->requestedSpeed = speed;
}

void bypassSlew(int name, int speed)
{
	driveMotor *x = &(driveMotors[name]);
	x->currentSpeed = speed;
	x->requestedSpeed = speed;
}

void disableSlew(int name)
{
	driveMotor *x = &(driveMotors[name]);
	x->shouldSlew = false;
}

void enableSlew(int name)
{
	driveMotor *x = &(driveMotors[name]);
	x->shouldSlew = true;
}

void slewMotors(void* param)
{
	motorArray[0] = &portOne;
	motorArray[1] = &portTwo;
	motorArray[2] = &portThree;
	motorArray[3] = &portFour;
	motorArray[4] = &portFive;
	motorArray[5] = &portSix;
	motorArray[6] = &portSeven;
	motorArray[7] = &portEight;
	uint32_t lastRun = pros::c::millis();
	int motorIndex = 0;
	driveMotor *x;
	while(true)
	{
		while(motorIndex < TOTAL_MOTORS)
		{
			x = &(driveMotors[motorIndex]);

			if(x->shouldSlew)
			{
				if(x->currentSpeed < x->requestedSpeed)
				{
					x->currentSpeed += x->slewRate;
					x->currentSpeed = (x->currentSpeed > x->requestedSpeed) ? x->requestedSpeed:x->currentSpeed;
				}
				if(x->currentSpeed > x->requestedSpeed)
				{
					x->currentSpeed -= x->slewRate;
					x->currentSpeed = (x->currentSpeed < x->requestedSpeed) ? x->requestedSpeed:x->currentSpeed;
				}
				x->currentSpeed = (x->currentSpeed > MAX_MOTOR_SPEED) ? MAX_MOTOR_SPEED:x->currentSpeed;
				x->currentSpeed = (x->currentSpeed < MIN_MOTOR_SPEED) ? MIN_MOTOR_SPEED:x->currentSpeed;

				motorArray[motorIndex]->move(x->currentSpeed);
			}
			else
			{
				motorArray[motorIndex]->move(x->requestedSpeed);
			}
			motorIndex++;
		}
		motorIndex = 0;
		pros::c::task_delay_until(&lastRun, SLEW_REFRESH_RATE);
		lastRun = pros::c::millis();
	}
}


#endif
