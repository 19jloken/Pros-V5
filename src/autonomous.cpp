#include "main.h"
#include "headers/autons.h"
#include "headers/robotFunction.h"
#include "headers/general.h"
#include "headers/pollSensors.h"
#include "headers/lcdCode.h"
#include "headers/opcontrol.h"
/**
* Runs the user autonomous code. This function will be started in its own task
* with the default priority and stack size whenever the robot is enabled via
* the Field Management System or the VEX Competition Switch in the autonomous
* mode. Alternatively, this function may be called in initialize or opcontrol
* for non-competition testing purposes.
*
* If the robot is disabled or communications is lost, the autonomous task
* will be stopped. Re-enabling the robot will restart the task, not re-start it
* from where it left off.
*/

void autonomous()
{
  pros::lcd::set_text(1, "Hello PROS Usera!");
  //pros::lcd::clear();
  robotFunctionTask.resume();
  resetRobotFunction();
  if(clearGyroBeforeAuton)// if the gyro should reset when the auton starts
  {
     resetGyro();// reset the gyro 
  }
  while(!autonomousSelected)
  {
    autonomousSelection();
    pros::delay(10);
  }
  switch(autonomousMode)// switch to determine which auton to run
  {
    case 1:
    autonomous1();
    break;
    case 2:
    autonomous2();
    break;
    case 3:
    autonomous3();
    break;
    case 4:
    autonomous4();
    break;
    case 5:
    autonomous5();
    break;
    case 6:
    autonomous6();
    break;
    case 7:
    autonomous7();
    break;
    case 8:
    autonomous8();
    break;
    case 9:
    autonomous9();
    break;
    case 10:
    autonomous10();
    break;
    case 11:
    autonomous11();
    break;
    case 12:
    autonomous12();
    break;
    case 13:
    autonomous13();
    break;
    case 14:
    autonomous14();
    break;
    case 15:
    autonomous15();
    break;
    case 16:
    autonomous16();
    break;
  }
}
