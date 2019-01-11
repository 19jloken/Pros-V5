#ifndef timers_cpp
#define timers_cpp

#include "headers/timers.h"
#include "pros/rtos.hpp"


void initializeTimer(timer* timer)
{
//	timer->initialValue = nSysTime;
	timer->timerValue = 0;
	timer->paused = false;
}

int currentTime(timer* timer)
{
	if(!(timer->paused))
		return (pros::c::millis() - timer->initialValue);
	else
		return (timer->timerValue);
}

void zeroTimer(timer* timer)
{
	timer->timerValue = 0;
	timer->initialValue = pros::c::millis();
}

void stopTimer(timer* timer)
{
	if(!timer->paused)
	timer->timerValue = (pros::c::millis() - timer->initialValue);

	timer->paused = true;
}

void startTimer(timer* timer)
{
	if(timer->paused)
	timer->initialValue = pros::c::millis() - timer->timerValue;

	timer->paused = false;
}

#endif
