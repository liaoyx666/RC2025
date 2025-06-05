#include "shoot.h"
#include <math.h>

#define PI 3.14159265358979f




#define SAMPLE_NUM_1 4//采样点数

float cubic_spline_1[SAMPLE_NUM_1 - 1][4] = {
{16210.000000f, 2670.046993f, 0.000000f, 5878.067391f},

{16860.000000f, 3523.542378f, 3879.524478f, -3211.288168f},

{19090.000000f, 5028.243495f, -744.730484f, 496.486990f},

};//三次样条插值法参数

float sample_distance_1[SAMPLE_NUM_1] = {1.3, 1.52, 2.0, 2.5};//采样点距离






#define SAMPLE_NUM_2 4//采样点数

float cubic_spline_2[SAMPLE_NUM_2 - 1][4] = {
{17790.000000f, 2826.000000f, 0.000000f, 1576.000000f},

{19400.000000f, 4008.000000f, 2364.000000f, -3080.000000f},

{21610.000000f, 4062.000000f, -2256.000000f, 1504.000000f},

};//三次样条插值法参数

float sample_distance_2[SAMPLE_NUM_2] = {2.0, 2.5, 3.0, 3.5};//采样点距离




#define SAMPLE_NUM_3 4//采样点数

float cubic_spline_3[SAMPLE_NUM_3 - 1][4] = {
{19930.000000f, 2837.089552f, 0.000000f, -308.358209f},

{21310.000000f, 2605.820896f, -462.537313f, 1221.791045f},

{22650.000000f, 3059.626866f, 1370.149254f, -1141.791045f},

};//三次样条插值法参数

float sample_distance_3[SAMPLE_NUM_3] = {3.0, 3.5, 4.0, 4.5};//采样点距离







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
	
	delta_x = 3.076052f - robot_x;
	delta_y = 0.38726f - robot_y;
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



















