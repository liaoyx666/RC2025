#include "shoot.h"
#include <math.h>

#define SAMPLE_NUM 10//采样点数


float cubic_spline[SAMPLE_NUM - 1][4] = {
{110000.000000f, 131427.549636f, 0.000000f, -11427.549636f},

{230000.000000f, 97144.900728f, -34282.648909f, 47137.748181f},

{340000.000000f, 169992.847453f, 107130.595634f, -67123.443088f},

{550000.000000f, 182883.709459f, -94239.733629f, 61356.024171f},

{700000.000000f, 178472.314712f, 89828.338883f, 21699.346405f},

{990000.000000f, 423227.031693f, 154926.378098f, -68153.409792f},

{1500000.000000f, 528619.558515f, -49533.851276f, 20914.292761f},

{2000000.000000f, 492294.734246f, 13209.027007f, -5503.761253f},

{2500000.000000f, 502201.504501f, -3302.256752f, 1100.752251f},

};//三次样条插值法参数

float sample_distance[SAMPLE_NUM] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};//采样点距离





float GetShootSpeed(float distance)
{
	uint8_t head = 0, tail = SAMPLE_NUM - 1, mid;
	
	if ((distance < sample_distance[0]) || (distance > sample_distance[SAMPLE_NUM - 1]))
	{
		return 0;
	}

	while (head < tail)
	{
		mid = (head + tail) / 2 + 1;
		
		if (distance <= sample_distance[mid])
		{
			tail = mid - 1;
		}
		else
		{
			head = mid;
		}
	}
	
	float dx = distance - sample_distance[head];
	
	return cubic_spline[head][3] * powf(dx, 3) + 
		   cubic_spline[head][2] * powf(dx, 2) + 
		   cubic_spline[head][1] * dx + 
		   cubic_spline[head][0];
}

