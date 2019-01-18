#ifndef pidController_cpp
#define pidController_cpp

#include "headers/pidController.h"
#include "headers/mathFunctions.h"
#include <cmath>
#include "pros/rtos.hpp"

void initializePID(PIDController* pid, float xp, float xi, float xd, float xc, float integralLimit, float lowerIntegralBand, float upperIntegralBand,  bool resetIntegralAtCross, bool changeConstantDirection)
{
	PIDController *x = pid;

	x->integralLimit = integralLimit;
	x->lowerIntegralBand = lowerIntegralBand;

	x->upperIntegralBand = upperIntegralBand;
	x->resetIntegralAtCross = resetIntegralAtCross;

	x->changeConstantDirection = changeConstantDirection;

	x->previousTime = pros::c::millis();
	x->target = 0;

	x->integralLimit = integralLimit;
	x->xp = xp;
	x->xi = xi;
	x->xd = xd;
	x->xc = xc;
	x->currentC = xc;

	x->lastError = 0;

	x->integral = 0;
	x->derivative = 0;

}

float calculatePID(PIDController *controller, float currentValue)
{
	PIDController *x = controller;

	x->deltaTime = (pros::c::millis()-x->previousTime)/1000;
	x->previousTime = pros::c::millis();

	x->error = x->target-currentValue;

	if(fabs(x->error) > x->lowerIntegralBand && fabs(x->error) < x->upperIntegralBand)
		x->integral += (x->error*x->deltaTime);

	if(fabs(x->integral) > x->integralLimit)
		x->integral = x->integralLimit*sgn(x->integral);

	if(x->resetIntegralAtCross)
	{
		if(x->error == 0 || sgn(x->error) != sgn(x->lastError))
			x->integral = 0;
	}

	x->derivative = (x->error-x->lastError)*x->deltaTime;
	x->lastError = x->error;

	if(x->changeConstantDirection)
		x->currentC = fabs(x->xc) * sgn(x->error);

	float output = x->error*x->xp + x->integral*x->xi - x->derivative*x->xd + x->currentC;
	return output;
}

void resetPID(PIDController* controller)
{
	PIDController *x = controller;

	x->lastError = 0;
	x->integral = 0;
	x->derivative = 0;
	x->previousTime = 0;
	x->deltaTime = 0;
	x->target = 0;
}

void setPIDTarget(PIDController* controller, float target)
{
	PIDController *x = controller;
	x->target = target;
}

#endif
