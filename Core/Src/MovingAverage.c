#include "MovingAverage.h"
#include <math.h>

int N = 0;

int count = 0;         /** Number of elements currently in the buffer */
float mean = 0;        /** Mean of the current values */

float sum = 0;         /** Sum of the values currently in the buffer */

float values[38];      /** Buffer */
int oldest_idx = 0;    /** Keeps track of the index of the oldest value */

void MovAvg_reset(int max_values) {
	count = 0;
	mean = 0;
	sum = 0;
	N = max_values;
}

void MovAvg_update(uint32_t newValue) {
	if(N == 0)
	{
		return;
	}

	/* First fill the buffer if not enough elements are there */
	if(count < N)
	{
		values[count] = (float)newValue;
		sum += (float)newValue;
		count++;
		mean = sum / (float)count;
	}
	else
	{
		/* Subtract old value from total sum */
		sum -= values[oldest_idx];
		/* Add new value to total sum */
		sum += (float)newValue;
		/* Calculate mean based on the new sum */
		mean = sum / (float)N;

		//mean = mean + 1.0f/(float)N * ((float)newValue - values[oldest_idx]);

		/* Store new value in the array and replace the old one */
		values[oldest_idx] = (float)newValue;

		/* Increment the index of the oldest value */
		oldest_idx++;
		if(oldest_idx == 38)
		{
			oldest_idx = 0;
		}
	}
}

uint32_t MovAvg_getMean() {
	return round(mean);
}
