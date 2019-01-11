#ifndef robotFunction_h
#define robotFunction_h

//#define leftDriveSensor 1
//#define rightDriveSensor 1
#define liftSensor 1
//#define gyroSensor 1
#define mobileSensor 1
#include "pros/rtos.hpp"
float getLeftDriveSensor();
float getRightDriveSensor();
float getStrafeMotorSensor();
float getGyroSensor();
float getLiftSensor();
float getIntakeSensor();
bool getLauncherSensor();
bool isLauncherLoaded();

enum Commands
{
  drive,
  pid,
  sweepRight,
  sweepLeft,
  forwardRight,
  forwardLeft,
  driveStraight,
  gyroDriveStraight,
  gyroTurn,

  PIDDriveCommandMaxValue,

  speed,
  driveSpeed,
  turnSpeed,
  sweepRightSpeed,
  sweepLeftSpeed,
  forwardRightSpeed,
  forwardLeftSpeed,
  driveStraightSpeed,
  gyroDriveStraightSpeed,
  gyroTurnSpeed,

  driveCommandMaxValue,

  lift,
  bar,
  intake,
  pause,
  pauseAll,
  maxTime,
  end,
  holdLift,
  setLift,
  time,

  strafe,

  launcher,
  reload,
  shoot
};

// extern const int PIDDriveCommandMaxValue;
// extern const int driveCommandMaxValue;
// extern const int drive;
// extern const int sweepRight;
// extern const int sweepLeft;
// extern const int forwardRight;
// extern const int forwardLeft;
// extern const int driveStraight;
// extern const int gyroDriveStraight;
// extern const int gyroTurn;
//
//
// extern const int driveSpeed;
// extern const int turnSpeed;
// extern const int sweepRightSpeed;
// extern const int sweepLeftSpeed;
// extern const int forwardRightSpeed;
// extern const int forwardLeftSpeed;
// extern const int driveStraightSpeed;
// extern const int gyroDriveStraightSpeed;
// extern const int gyroTurnSpeed;
//
// extern const int mobile;
//
// extern const int lift;
// extern const int bar;
// extern const int intake;
// extern const int pause;
// extern const int pauseAll;
// extern const int maxTime;
// extern const int end;
// extern const int holdLift;
// extern const int aligner;
// extern const int setLift;
// extern const int hoarder;
// extern const int time;
//
// extern const int in;
// extern const int out;

extern int instructions[300];
extern int commandReadPos;
extern int commandWritePos;
extern const int goToStart;

void moveLift(int a);
void moveIntake(int a);
void moveDrive(int left, int right);
void moveStrafe(int Speed);
void moveLauncher(int speed);
int invertPotValue(int potValue);
void addCommand(int a = -1, int b = -1, int c = -1, int dd = -1, int e = -1, int f = -1, int g = -1, int h = -1, int ii = -1, int j = -1, int k = -1, int l = -1);
void robotFunction(void* param);
void resetRobotFunction();

extern pros::Task robotFunctionTask;

#endif
