#ifndef lcdCode_cpp
#define lcdCode_cpp

#include "headers/lcdCode.h"
#include "headers/general.h"
#include "pros/llemu.hpp"

bool leftButtonPressed = false;
bool centerButtonPressed = false;
bool rightButtonPressed = false;

void autonomousSelection()
{
  if(autonomousSelected)
  {
  pros::lcd::print(0, "Autonomous %d selected", autonomousMode);
    pros::lcd::set_text(1, "Center Btn to deselect");
    pros::lcd::register_btn1_cb(unselectAutonomous);
  }
  else
  {
    pros::lcd::print(0, "Autonomous %d", autonomousMode);
    pros::lcd::set_text(1, "Center Btn to select");
    pros::lcd::register_btn0_cb(decreaseAutonomousMode);
    pros::lcd::register_btn1_cb(selectAutonomous);
    pros::lcd::register_btn2_cb(increaseAutonomousMode);
  }
}

void decreaseAutonomousMode()
{
  leftButtonPressed = !leftButtonPressed;
  if (leftButtonPressed)
	{
		autonomousMode--;
		if(autonomousMode < 1)
		autonomousMode = 16;
  }
}

void selectAutonomous()
{
  centerButtonPressed = !centerButtonPressed;
  if (centerButtonPressed)
	{
		autonomousSelected = true;
  }
}

void increaseAutonomousMode()
{
  rightButtonPressed = !rightButtonPressed;
  if (rightButtonPressed)
	{
		autonomousMode++;
		if(autonomousMode > 16)
		autonomousMode = 1;
  }
}

void unselectAutonomous()
{
	centerButtonPressed = !centerButtonPressed;
	if(centerButtonPressed)
	{
		autonomousSelected = false;
	}
}

#endif
