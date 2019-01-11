#ifndef pollSensors_cpp
#define pollSensors_cpp

#include "headers/pollSensors.h"
#include "headers/filters.h"
#include "headers/timers.h"
#include "headers/motorSlew.h"
#include "headers/general.h"
#include "pros/rtos.hpp"
#include "pros/adi.hpp"
#include "headers/gyroFunctions.h"
#include "pros/llemu.hpp"

pros::Task pollSensorsTask(pollSensors, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "pollSensorsTask");

const int SensorRefreshRate = 10;

 demaFilter gyroFilter;
 demaFilter leftDriveFilter;
 demaFilter rightDriveFilter;

 pros::ADIEncoder leftDriveSensor (1, 2, false);
 pros::ADIEncoder rightDriveSensor (3, 4, false);
 pros::ADIGyro gyroSensor (5);
 pros::ADIDigitalIn launcherSensor(6);

void pollSensors(void* param)
{

   pros::ADIEncoder leftDriveSensor (1, 2, false);
   pros::ADIEncoder rightDriveSensor (3, 4, false);
   pros::ADIGyro gyroSensor (5);

	newDemaFilter(&gyroFilter);
	newDemaFilter(&leftDriveFilter);
	newDemaFilter(&rightDriveFilter);

  uint32_t lastRun = pros::c::millis();

	while(true)
	{
		filterDema(&gyroFilter, correctGyroValue(gyroSensor.get_value()));
		filterDema(&leftDriveFilter, motorArray[frontLeftDrive]->get_position());
		filterDema(&rightDriveFilter, motorArray[frontRightDrive]->get_position());
    pros::lcd::print(5, "get left %f", motorArray[frontLeftDrive]->get_position());
    pros::lcd::print(6, "get right sensor %f", motorArray[frontRightDrive]->get_position());
    pros::lcd::print(7, "get raw gyro sensor %f", gyroSensor.get_value());

    pros::c::task_delay_until(&lastRun, SensorRefreshRate);
		lastRun = pros::c::millis();
	}
}

void resetGyro()
{
  gyroSensor.reset();
}
float getRawGyro()
{
  return gyroSensor.get_value();
}
void resetLeftDriveSensor()
{
  motorArray[frontLeftDrive]->tare_position();
}
void resetRightDriveSensor()
{
  motorArray[frontRightDrive]->tare_position();
}
void resetStrafeDriveSensor()
{
  motorArray[strafeWheel]->tare_position();
}
bool getLauncherSensor()
{
	return launcherSensor.get_value();
}
#endif
