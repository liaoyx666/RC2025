#include "shoot.h"
#include <math.h>


#define PI 3.14159265358979f




#define SAMPLE_NUM_1 4//采样点数

float cubic_spline_1[SAMPLE_NUM_1 - 1][4] = {
{16331.000000f, 2835.827402f, 0.000000f, 4237.224978f},

{17000.000000f, 3451.072469f, 2796.568486f, -911.857676f},

{19200.000000f, 5505.502189f, 1483.493432f, -988.995621f},

};//三次样条插值法参数

float sample_distance_1[SAMPLE_NUM_1] = {1.3, 1.52, 2.0, 2.5};//采样点距离






#define SAMPLE_NUM_2 4//采样点数

float cubic_spline_2[SAMPLE_NUM_2 - 1][4] = {
{18450.000000f, 2589.333333f, 0.000000f, 2042.666667f},

{20000.000000f, 4121.333333f, 3064.000000f, -5013.333333f},

{22200.000000f, 3425.333333f, -4456.000000f, 2970.666667f},

};//三次样条插值法参数

float sample_distance_2[SAMPLE_NUM_2] = {2.0, 2.5, 3.0, 3.5};//采样点距离








float CalcSpeed(float distance, float cubic_spline[][4], float *sample_distance, uint16_t num)
{
	uint8_t head = 0, tail = num - 1, mid;

	if ((distance < sample_distance[0]) || (distance > sample_distance[num - 1]))
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


float GetShootSpeed(float distance, bool large_pitch)
{
	if (large_pitch)
	{
		return CalcSpeed(distance, cubic_spline_1, sample_distance_1, SAMPLE_NUM_1);
	}
	else
	{
		return CalcSpeed(distance, cubic_spline_2, sample_distance_2, SAMPLE_NUM_2);
	}
}

















#define HOOP_X  3.294f; // 篮筐的X坐标（单位：米）
#define HOOP_Y  0.26f; // 篮筐的Y坐标（单位：米）
float GetHoopAngle(float robot_x, float robot_y, float *distance)
{
	float delta_x, delta_y, target_theta;
	
	delta_x = 3.294f - robot_x;
	delta_y = 0.36f - robot_y;
    target_theta = atan2f(delta_y, delta_x); // 朝向篮筐的方向（弧度）

	target_theta = target_theta * 180.f / PI;
	
	
	*distance = sqrtf(delta_x * delta_x + delta_y * delta_y);
	
	
	
	
	if (target_theta <= -90.f)
	{
		target_theta += 270.f;
	}
	else
	{
		target_theta -= 90.f;
	}
	
//	if (target_theta <= -90)
//	{
//		target_theta -= 90.f;
//	}
//	else
//	{
//		target_theta += 270.f;
//	}
	return target_theta;
}



















