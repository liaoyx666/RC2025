#include "shoot.h"
#include <math.h>

#define PI 3.14159265358979f




#define SAMPLE_NUM_1 5//采样点数




float cubic_spline_1[SAMPLE_NUM_1 - 1][4] = {
{10505.000000f, 1510.178571f, 0.000000f, 879.285714f},

{11370.000000f, 2169.642857f, 1318.928571f, -276.428571f},

{12750.000000f, 3281.250000f, 904.285714f, -133.571429f},

{14600.000000f, 4085.357143f, 703.928571f, -469.285714f},

};

float sample_distance_1[SAMPLE_NUM_1] = {1.5, 2.0, 2.5, 3.0, 3.5};





//float cubic_spline_1[SAMPLE_NUM_1 - 1][4] = {
//{15595.000000f, 1476.372549f, 0.000000f, 13215.686275f},

//{15996.000000f, 3062.254902f, 7929.411765f, -6075.843137f},

//{18750.000000f, 6434.784314f, -1184.352941f, 789.568627f},

//};//三次样条插值法参数

//float sample_distance_1[SAMPLE_NUM_1] = {1.3, 1.5, 2.0, 2.5};//采样点距离






#define SAMPLE_NUM_2 4//采样点数

float cubic_spline_2[SAMPLE_NUM_2 - 1][4] = {
{17815.000000f, 3052.146893f, 0.000000f, 551.412429f},

{19410.000000f, 3465.706215f, 827.118644f, 4642.937853f},

{21930.000000f, 7775.028249f, 7791.525424f, -8657.250471f},

};//三次样条插值法参数

float sample_distance_2[SAMPLE_NUM_2] = {2.0, 2.5, 3.0, 3.3};//采样点距离




#define SAMPLE_NUM_3 4//采样点数

float cubic_spline_3[SAMPLE_NUM_3 - 1][4] = {
{20150.000000f, 5133.613445f, 0.000000f, -134.453782f},

{22700.000000f, 5032.773109f, -201.680672f, 308.123249f},

{24200.000000f, 4994.957983f, 75.630252f, -252.100840f},

};//三次样条插值法参数

float sample_distance_3[SAMPLE_NUM_3] = {3.0, 3.5, 3.8, 3.9};//采样点距离







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










#define HOOP_X -3.483f   
#define HOOP_Y 14.14f

//#define HOOP_X  10.6293278f // 篮筐的X坐标（单位：米）
//#define HOOP_Y  3.77741861f // 篮筐的Y坐标（单位：米）



//#define HOOP_X  3.156007f // 篮筐的X坐标（单位：米）
//#define HOOP_Y  0.32287669f // 篮筐的Y坐标（单位：米）



float GetHoopAngle(float robot_x, float robot_y, float *distance)
{
	float delta_x, delta_y, target_theta;
	
	delta_x = HOOP_X - robot_x;
	delta_y = HOOP_Y - robot_y;
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
	
	target_theta += 180.0f;
	
	if (target_theta != target_theta)
	{
		return 0;
	}
	
	
	while (target_theta > 180.0f)
	{
		target_theta -= 360.0f;
	}
	
	while (target_theta < -180.0f)
	{
		target_theta += 360.0f;
	}
	
	return target_theta;
}


extern float valid_num1;
extern float valid_num2;
extern float valid_num3;


float GetR1Angle(float robot_x, float robot_y, float *distance)
{
	float delta_x, delta_y, target_theta;
	
	delta_x = valid_num1 - robot_x;
	delta_y = valid_num2 - robot_y;
	
	
	if (isnan(valid_num1) || isnan(valid_num2))
	{
		*distance = 0;
		return 0;
	}
	
	
    target_theta = atan2f(delta_y, delta_x); // 朝向R1的方向（弧度）

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
	
	target_theta += 180.0f;
	
	if (target_theta != target_theta)
	{
		return 0;
	}
	
	
	while (target_theta > 180.0f)
	{
		target_theta -= 360.0f;
	}
	
	while (target_theta < -180.0f)
	{
		target_theta += 360.0f;
	}
	
	return target_theta;
}
















