#ifndef general_cpp
#define general_cpp

#include "pros/rtos.hpp"
#include "pros/misc.hpp"

const int frontLeftDrive = 0; // port 1
const int frontRightDrive = 1; // port 2
const int backLeftDrive = 2; // port 3
const int backRightDrive = 3; // port 4
const int strafeWheel = 4; // port 5
const int intakeMotor = 5; // port 6
const int shooter = 6; // port 7
const int liftMotor = 7; // port 8


bool clearGyroBeforeAuton = true;

int autonomousMode = 1;
bool autonomousSelected = true;

#endif
