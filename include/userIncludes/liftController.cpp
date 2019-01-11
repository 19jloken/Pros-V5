#ifndef liftController_cpp
#define liftController_cpp

#include "headers/liftController.h"

#include "pros/rtos.hpp"
#include <cmath>

void newLiftControl(holdingController *holdingObject, float liftP, int liftEven, int liftChange, int liftErrorThreshold, int liftUpperBound, int liftLowerBound, int liftRefresh, int liftMaxSpeed, bool learnHolding)
{
	holdingController *x = holdingObject;
	x->liftHolding = 0;
	x->liftP = liftP;
	x->liftEven = liftEven;
	x->wasEven = false;
	x->learnHolding = learnHolding;
	x->liftMaxSpeed = liftMaxSpeed;
	x->liftChange = liftChange;
	x->liftRefresh = liftRefresh;
	x->liftErrorThreshold = liftErrorThreshold;
	x->liftUpperBound = liftUpperBound;
	x->liftLowerBound = liftLowerBound;
}

void setLiftTarget(holdingController *holdingObject, int target)
{
	holdingController *x = holdingObject;
	x->liftTarget = target;
	x->liftControllerLastRun = 0;
}

int getPower(holdingController *holdingObject, int currentLiftPosition)//determines and sets variable holding power for the lift
{
	holdingController *x = holdingObject;

	if((pros::c::millis() - x->liftControllerLastRun) > x->liftRefresh)// if the minimum time between cycles has passed
	{
		x->liftControllerLastRun = pros::c::millis();
		// if the lift should be going down and is going up
		if((currentLiftPosition > (x->liftTarget+x->liftErrorThreshold))&&(currentLiftPosition > (x->liftLowerBound+x->liftErrorThreshold)) && (currentLiftPosition >= x->liftPrevious) && x->liftHolding > -127)
		{
			x->liftHolding = x->liftHolding - x->liftChange;// decrease the holding strength
			if(x->wasEven)// if the lift was in the threshold last cycle
			{
				x->wasEven = false;// wasEven = false
				if(x->liftEven > -127 && x->learnHolding)// if the neutral holding power can be decreased
				{
					x->liftEven = x->liftEven - x->liftChange;// decrease the neutral holding power
				}
			}
		}
		// otherwise if the lift is going down and should be going down
		else if((currentLiftPosition > (x->liftTarget+x->liftErrorThreshold))&&(currentLiftPosition > (x->liftLowerBound+x->liftErrorThreshold)) && (currentLiftPosition < x->liftPrevious) && x->liftHolding > -127)
		{
			if(abs(currentLiftPosition - x->liftPrevious) > x->liftMaxSpeed)// if the lift is going too fast
			{
				x->liftHolding = x->liftHolding + x->liftChange;// increase the holding strength
			}
			else if(abs(currentLiftPosition - x->liftPrevious) < x->liftMaxSpeed/2)// if the lift is not going fast enough
			{
				x->liftHolding = x->liftHolding - x->liftChange;// decrease the holding strength
			}
			if(x->wasEven)// if the lift was in the threshold last cycle
			{
				x->wasEven = false;// wasEven = false
				if(x->liftEven > -127 && x->learnHolding)// if the neutral holding power can be decreased
				{
					x->liftEven = x->liftEven - x->liftChange;// decrease the neutral holding power
				}
			}
		}
		// otherwise if the lift is going down and should be going up
		else if((currentLiftPosition < (x->liftTarget-x->liftErrorThreshold))&&(currentLiftPosition < (x->liftUpperBound-x->liftErrorThreshold)) && (currentLiftPosition <= x->liftPrevious) && x->liftHolding < 127)
		{
			x->liftHolding = x->liftHolding + x->liftChange;// increase the holding strength
			if(x->wasEven)// if the lift was in the threshold last cycle
			{
				x->wasEven = false;// wasEven = false
				if(x->liftEven < 127 && x->learnHolding)// if the neutral holding power can be increased
				{
					x->liftEven = x->liftEven + x->liftChange;// increase the neutral holding power
				}
			}
		}
		// otherwise if the lift is going up and should be going up
		else if((currentLiftPosition < (x->liftTarget-x->liftErrorThreshold))&&(currentLiftPosition < (x->liftUpperBound-x->liftErrorThreshold)) && (currentLiftPosition > x->liftPrevious) && x->liftHolding < 127)
		{
			if(abs(currentLiftPosition - x->liftPrevious) > x->liftMaxSpeed)// if the lift is going too fast
			{
				x->liftHolding = x->liftHolding - x->liftChange;// decrease the holding strength
			}
			else if(abs(currentLiftPosition - x->liftPrevious) < x->liftMaxSpeed/2)// if the lift is not going fast enough
			{
				x->liftHolding = x->liftHolding + x->liftChange;// increase the holding strength
			}
			if(x->wasEven)// if the lift was in the threshold last cycle
			{
				x->wasEven = false;// wasEven = false
				if(x->liftEven < 127 && x->learnHolding)// if the neutral holding power can be increased
				{
					x->liftEven = x->liftEven + x->liftChange;// increase the neutral holding power
				}
			}
		}
		// otherwise if the lift is in the threshold
		else if((currentLiftPosition < (x->liftTarget+x->liftErrorThreshold) && (currentLiftPosition > (x->liftTarget-x->liftErrorThreshold))))
		{
			x->liftHolding = x->liftEven;// set the holding power to neutral
			x->wasEven = true;// wasEven = true;
		}
		x->liftPrevious = currentLiftPosition;// record the current lift value to be used the next time the function is run
	}
	return x->liftHolding + (x->liftTarget - currentLiftPosition)*x->liftP;
}

#endif
