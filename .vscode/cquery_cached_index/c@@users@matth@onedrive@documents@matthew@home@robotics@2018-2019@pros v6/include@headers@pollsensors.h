#ifndef pollSensors_h
#define pollSensors_h

#include "headers/filters.h"

void pollSensors(void* param);

// extern pros::ADIEncoder leftDriveSensor;
// pros::ADIEncoder rightDriveSensor (3, 4, false);
// pros::ADIGyro gyroSensor (5);
// pros::ADIDigitalIn launcherSensor(6);

extern demaFilter gyroFilter;
extern demaFilter leftDriveFilter;
extern demaFilter rightDriveFilter;

void resetGyro();
float getRawGyro();
void resetLeftDriveSensor();
void resetRightDriveSensor();
void resetStrafeDriveSensor();
bool getLauncherSensor();


extern const int SensorRefreshRate;


#endif
