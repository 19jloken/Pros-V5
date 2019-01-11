#ifndef filters_cpp
#define filters_cpp

#include "headers/filters.h"
#include <cmath>

void newEmaFilter(emaFilter *filter, int length, float initialValue)
{
	emaFilter *x = (filter);
	x->period = length;
	x->currentValue = initialValue;
	x->previousValue = initialValue;
	x->alpha = (2/((float)length+1));
}

float filterEma(emaFilter *filter, float input)
{
	filter->currentValue = ((input-filter->previousValue) * filter->alpha) + filter->previousValue;
	filter->previousValue = filter->currentValue;
	return filter->currentValue;
}

float getEma(emaFilter *filter)
{
	return filter->currentValue;
}

void newDemaFilter(demaFilter *filter, int length1, int length2, float initialValue)
{
	demaFilter *x = filter;
	x->period1 = length1;
	x->currentValue1 = initialValue;
	x->alpha1 = (2/((float)length1+1));
	x->period2 = length2;
	x->currentValue2 = initialValue;
	x->alpha2 = (2/((float)length2+1));
	x->output = initialValue;
}

float filterDema(demaFilter *filter, float input)
{
	filter->currentValue1 = ((input-filter->previousValue1) * filter->alpha1) + filter->previousValue1;
	filter->previousValue1 = filter->currentValue1;

	filter->currentValue2 = ((filter->currentValue1-filter->previousValue2) * filter->alpha2) + filter->previousValue2;
	filter->previousValue2 = filter->currentValue2;

	filter->output = 2*filter->currentValue1-filter->currentValue2;
	return filter->output;
}

float getDema(demaFilter *filter)
{
	return filter->output;
}

void newMedianFilter(medianFilter *filter, int width, float initialValue)
{
	medianFilter *x = (filter);
	x->width = width;
	x->currentValue = initialValue;
	int y = 0;
	while(y < 50)
	{
		x->previousValues[y] = initialValue;
		y++;
	}
}

float filterMedian(medianFilter *filter, float input)
{
	medianFilter *x = (filter);
	float tempArray[50];
	int y = x->width-1;
	while(y > 0)
	{
		tempArray[y] = x->previousValues[y-1];
		y--;
	}
	tempArray[0] = (int)input;
	for(int count = 0; count < x->width; ++count)
	{
		x->previousValues[count] = tempArray[count];
	}

	y = 1;
	int j = 0;
	int testing;
	int largest = tempArray[0];
	while(y < x->width)
	{
		testing = tempArray[y];
		j = y-1;

		while(j >= 0 && tempArray[j] > testing)
		{
			tempArray[j+1] = tempArray[j];
			j -=1;
		}
		tempArray[j+1] = testing;
		y++;
	}
	x->currentValue = tempArray[static_cast<int>(floor(x->width/2))];
	return x->currentValue;

}

void resetMedianFilter(medianFilter *filter,float initialValue)
{
	medianFilter *x = (filter);
	x->currentValue = initialValue;
	int y = 0;
	while(y < 50)
	{
		x->previousValues[y] = initialValue;
		y++;
	}
}

float getMedian(medianFilter *filter)
{
	medianFilter *x = (filter);
	return x->currentValue;
}

#endif
