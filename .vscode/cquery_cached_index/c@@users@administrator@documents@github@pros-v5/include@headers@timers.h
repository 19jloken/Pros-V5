#ifndef timers_h
#define timers_h

typedef struct{
	int initialValue;
	int timerValue;
	bool paused;
}timer;

void initializeTimer(timer *timer);
int currentTime(timer *timer);
void zeroTimer(timer *timer);
void stopTimer(timer *timer);
void startTimer(timer *timer);

#endif
