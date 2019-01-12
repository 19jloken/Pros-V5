#ifndef general_cpp
#define general_cpp

#include "pros/rtos.hpp"
#include "pros/misc.hpp"

const int frontLeftDrive = 1; // port 2
const int frontRightDrive = 2; // port 3
const int backLeftDrive = 3; // port 4
const int backRightDrive = 4; // port 5
const int intakeMotor = 5; // port 6
const int shooter = 0; // port 1
const int liftMotor = 8; // port 9


bool clearGyroBeforeAuton = true;

int autonomousMode = 1;
bool autonomousSelected = true;

#endif
