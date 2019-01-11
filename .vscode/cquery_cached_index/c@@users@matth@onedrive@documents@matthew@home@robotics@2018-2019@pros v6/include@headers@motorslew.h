#ifndef motorSlew_h
#define motorSlew_h

#include "pros/motors.hpp"
#include "pros/rtos.hpp"

#define TOTAL_MOTORS 8
#define DEFAULT_SLEW_RATE 8
#define SLEW_REFRESH_RATE 10
#define MAX_MOTOR_SPEED 127
#define MIN_MOTOR_SPEED -127

typedef struct{
int requestedSpeed;
int currentSpeed;
int slewRate;
bool shouldSlew;
} driveMotor;

extern pros::Task slewMotorsTask;

extern pros::Motor *motorArray [TOTAL_MOTORS];

extern pros::Motor portOne;
extern pros::Motor portTwo;
extern pros::Motor portThree;
extern pros::Motor portFour;
extern pros::Motor portFive;
extern pros::Motor portSix;
extern pros::Motor portSeven;
extern pros::Motor portEight;

extern driveMotor driveMotors[TOTAL_MOTORS];

void addMotor(int name, int slew = DEFAULT_SLEW_RATE);
void setMotor(int name, int speed);
void bypassSlew(int name, int speed);
void disableSlew(int name);
void enableSlew(int name);
void slewMotors(void* param);

#endif
