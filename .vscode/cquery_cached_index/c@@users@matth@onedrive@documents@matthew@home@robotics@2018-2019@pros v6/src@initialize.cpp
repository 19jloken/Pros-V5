
#include "main.h"

 #include "headers/lcdCode.h"
 #include "headers/motorSlew.h"
 #include "headers/opcontrol.h"

#include "userIncludes/autons.cpp"
#include "userIncludes/filters.cpp"
#include "userIncludes/general.cpp"
#include "userIncludes/gyroFunctions.cpp"
#include "userIncludes/lcdCode.cpp"
#include "userIncludes/liftController.cpp"
#include "userIncludes/mathFunctions.cpp"
#include "userIncludes/motorSlew.cpp"
#include "userIncludes/pidController.cpp"
#include "userIncludes/pollSensors.cpp"
#include "userIncludes/robotFunction.cpp"
#include "userIncludes/thresholds.cpp"
#include "userIncludes/timers.cpp"


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
  // userAutonomousTask.suspend();
  robotFunctionTask.suspend();
	pros::lcd::initialize();
  pros::lcd::set_text(1, "DO NOT MOVE THE ROBOT");
  pros::delay(1000);
  pros::lcd::set_text(2, "Calibrating");
  pros::c::adi_analog_calibrate(5);

  motorArray[0]->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  motorArray[1]->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  motorArray[2]->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  motorArray[3]->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  motorArray[4]->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  motorArray[5]->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  motorArray[6]->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  motorArray[7]->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	addMotor(0);
	addMotor(1);
	addMotor(2);
	addMotor(3);
	addMotor(4);
	addMotor(5);
	addMotor(6);
	addMotor(7);

	enableSlew(0);
	enableSlew(1);
	enableSlew(2);
	enableSlew(3);
	enableSlew(4);
	enableSlew(5);
	enableSlew(6);
	enableSlew(7);

  motorArray[frontLeftDrive]->set_gearing(pros::E_MOTOR_GEARSET_18);
  motorArray[frontRightDrive]->set_gearing(pros::E_MOTOR_GEARSET_18);
  motorArray[backLeftDrive]->set_gearing(pros::E_MOTOR_GEARSET_18);
  motorArray[backRightDrive]->set_gearing(pros::E_MOTOR_GEARSET_18);
  motorArray[intakeMotor]->set_gearing(pros::E_MOTOR_GEARSET_06);
  motorArray[shooter]->set_gearing(pros::E_MOTOR_GEARSET_36);
  motorArray[liftMotor]->set_gearing(pros::E_MOTOR_GEARSET_36);

  motorArray[frontRightDrive]->set_reversed(true);
  motorArray[backRightDrive]->set_reversed(true);
  motorArray[shooter]->set_reversed(true);
  motorArray[intakeMotor]->set_reversed(true);
  pros::delay(2000);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
 void disabled()
 {
   pros::lcd::set_text(1, "Hello PROS User!d");
 }

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize()
{
  pros::lcd::set_text(1, "Hello PROS User!ci");
	autonomousSelection();
}
