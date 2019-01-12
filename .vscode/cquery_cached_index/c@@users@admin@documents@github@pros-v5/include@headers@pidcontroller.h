#ifndef pidController_h
#define pidController_h

typedef struct{
	float xp;
	float xi;
	float xd;
	float xc;

	float currentC;
	bool changeConstantDirection;

	float lastError;
	float error;

	float integral;
	float derivative;

	float integralLimit;
	bool resetIntegralAtCross;
	float lowerIntegralBand;
	float upperIntegralBand;

	float previousTime;
	float deltaTime;

	float target;
}PIDController;

void initializePID(PIDController *pid, float xp, float xi, float xd, float xc, float integralLimit, float lowerIntegralBand, float upperIntegralBand,  bool resetIntegralAtCross = true, bool changeConstantDirection = true);
float calculatePID(PIDController *controller, float currentValue);
void setPIDTarget(PIDController *controller, float target);
void resetPID(PIDController *controller);

#endif
