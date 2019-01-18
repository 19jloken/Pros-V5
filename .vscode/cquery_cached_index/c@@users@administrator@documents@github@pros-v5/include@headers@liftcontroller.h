#ifndef liftController_h
#define liftController_h

typedef struct{
	float liftHolding;
	float liftEven;
	float liftP;
	bool wasEven;
	bool learnHolding;
	int liftPrevious;
	int liftTarget;
	int liftMaxSpeed;
	int liftErrorThreshold;
	int liftUpperBound;
	int liftLowerBound;
	float liftChange;
	int liftRefresh;
	int liftControllerLastRun;
}holdingController;

void newLiftControl(holdingController *holdingObject, float liftP, int liftEven = 0, int liftChange =1 , int liftErrorThreshold = 200, int liftUpperBound = 4095, int liftLowerBound = 0, int liftRefresh = 50, int liftMaxSpeed =100, bool learnHolding = false);

void setLiftTarget(holdingController *holdingObject, int target);

int getPower(holdingController *holdingObject, int currentLiftPosition);

#endif
