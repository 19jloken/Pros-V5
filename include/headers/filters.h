#ifndef filters_h
#define filters_h

typedef struct{
	int period;
	float alpha;
	float currentValue;
	float previousValue;
}emaFilter;
void newEmaFilter(emaFilter *filter, int length = 10, float initialValue = 0);
float filterEma(emaFilter *filter, float input);
float getEma(emaFilter *filter);

typedef struct{
	int period1;
	float alpha1;
	float currentValue1;
	float previousValue1;
	int period2;
	float alpha2;
	float currentValue2;
	float previousValue2;
	float output;
}demaFilter;

void newDemaFilter(demaFilter *filter, int length1 = 10, int length2 = 10, float initialValue = 0);
float filterDema(demaFilter *filter, float input);
float getDema(demaFilter *filter);

typedef struct{
	int width;
	float previousValues[50];
	float currentValue;
}medianFilter;

void newMedianFilter(medianFilter *filter, int width = 5, float initialValue = 0);
float filterMedian(medianFilter *filter, float input);
void resetMedianFilter(medianFilter *filter,float initialValue = 0);
float getMedian(medianFilter *filter);

#endif
