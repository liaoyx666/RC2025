#include "shoot.h"
#include <math.h>

#define PI 3.14159265358979f




#define SAMPLE_NUM_1 4//采样点数

float cubic_spline_1[SAMPLE_NUM_1 - 1][4] = {
{15715.000000f, 3625.000000f, 0.000000f, 5000.000000f},

{16480.000000f, 4225.000000f, 3000.000000f, -2180.000000f},

{19070.000000f, 5590.000000f, -270.000000f, 180.000000f},

};//三次样条插值法参数

float sample_distance_1[SAMPLE_NUM_1] = {1.3, 1.5, 2.0, 2.5};//采样点距离






#define SAMPLE_NUM_2 4//采样点数

float cubic_spline_2[SAMPLE_NUM_2 - 1][4] = {
{17815.000000f, 3052.146893f, 0.000000f, 551.412429f},

{19410.000000f, 3465.706215f, 827.118644f, 4642.937853f},

{21930.000000f, 7775.028249f, 7791.525424f, -8657.250471f},

};//三次样条插值法参数

float sample_distance_2[SAMPLE_NUM_2] = {2.0, 2.5, 3.0, 3.3};//采样点距离




#define SAMPLE_NUM_3 3//采样点数

float cubic_spline_3[SAMPLE_NUM_3 - 1][4] = {
{20080.000000f, 3014.820574f, 0.000000f, 4900.717703f},

{22200.000000f, 6690.358852f, 7351.076555f, -6448.312768f},

};//三次样条插值法参数

float sample_distance_3[SAMPLE_NUM_3] = {3.0, 3.5, 3.88};//采样点距离







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


float GetShootSpeed(float distance, uint8_t pitch_level)
{
	switch(pitch_level)
	{
		case 0:
			return CalcSpeed(distance, cubic_spline_1, sample_distance_1, SAMPLE_NUM_1);
			break;
		case 1:
			return CalcSpeed(distance, cubic_spline_2, sample_distance_2, SAMPLE_NUM_2);
			break;
		case 2:
			return CalcSpeed(distance, cubic_spline_3, sample_distance_3, SAMPLE_NUM_3);
			break;
		default:
			return 0;
			break;
	}
}

















#define HOOP_X  3.294f; // 篮筐的X坐标（单位：米）
#define HOOP_Y  0.26f; // 篮筐的Y坐标（单位：米）
float GetHoopAngle(float robot_x, float robot_y, float *distance)
{
	float delta_x, delta_y, target_theta;
	
	delta_x = 3.074919f - robot_x;
	delta_y = 0.35137f - robot_y;
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
	
	return target_theta;
}



















