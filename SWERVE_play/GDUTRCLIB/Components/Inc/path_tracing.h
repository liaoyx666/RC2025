#pragma once

#include <math.h>
#include <stdint.h>
#include "speed_plan.h"
#include "motor.h"

#ifdef __cplusplus

struct PointVector
{
	float x, y;
	PointVector(float x = 0, float y = 0) : x(x), y(y) {}
};

#define ACCEL   5.5f
#define DECEL   2.5f
#define MAX_VEL 3.2f

class Path
{
	public:
		Path(float start_x, float start_y, float end_x, float end_y)
		{
			start_point.x = start_x;
			start_point.y = start_y;
			end_point.x = end_x;
			end_point.y = end_y;
			
			direction_vector.x = end_x - start_x;
			direction_vector.y = end_y - start_y;
			
			angle = atan2f(direction_vector.y, direction_vector.x);//x轴方向为0度
			
			SIN_angle = sinf(angle);
			COS_angle = cosf(angle);
			
			max_distance = sqrtf(direction_vector.x * direction_vector.x + direction_vector.y * direction_vector.y);
			
			
			
			CalcSpeedPlan();
			
			
		}
		
		void GetErrorAndDistance(PointVector currentPoint, float *error, float *distance);
		
		bool IsArriveEnd(PointVector currentPoint);
		bool IsArriveStart(PointVector currentPoint);
		bool IsApproachEnd(PointVector currentPoint);
		bool IsApproachStart(PointVector currentPoint);
		
		float max_v, accel_range, decel_range;
		
		float SIN_angle;
		float COS_angle;
		float max_distance;
		
		void GetSpeedPlan(TrapePlanner *planner);
		
	protected:
		void CalcSpeedPlan(void);
		
	private:
		const float arrive_deadzone = 0.3f, approach_deadzone = 0.3f;
		PointVector start_point, end_point;
		PointVector direction_vector;
		float angle;
		
	
	
		
	
};

	


#define SECTION_NUM  8




	
class Path_Tracing
{
	public:
		bool Pid_Param_Init_Path(uint8_t num, float Kp, float Ki, float Kd, float Integral_Max, float OUT_Max, float DeadZone);
		bool Pid_Mode_Init_Path(uint8_t num, float LowPass_error, float LowPass_d_err, bool D_of_Current, bool Imcreatement_of_Out);
		
	
		void PathTracing(enum CONTROL_E state, PointVector currentPoint, float *speed_x, float *speed_y);

	
	protected:
		
	private:
		Path path[SECTION_NUM] = {
			Path(0, -0.66,       0, -0.33),
			Path(0, -0.33,      -0.88, 0.46),
			Path(-0.88, 0.46,   -0.86, 2.3),
			Path(-0.86, 2.3,     1.2, 2.3),
			Path(1.2, 2.3,       3.28, 2.3),
			Path(3.28, 2.3,      3.31, 0.46),
			Path(3.31, 0.46,     2.46, -0.33),
			Path(2.46, -0.33,     0, -0.33),
		};
		
		//const float accel = 1.f, decel = 1.f, max_vel = 2.5f;
		
		
		
		
		PID PID_Normal;
		PID PID_Tangential;
		TrapePlanner PathPlanner = TrapePlanner(0.10f, 0.40f, 2.f, 0.6f, 0.1f);
//		TrapePlanner PathPlanner_1 = TrapePlanner(0.10f, 0.40f, 2.f, 0.05f, 0.2f);
//		TrapePlanner PathPlanner_2 = TrapePlanner(0.10f, 0.40f, 2.f, 0.05f, 0.2f);
//		TrapePlanner PathPlanner_3 = TrapePlanner(0.10f, 0.40f, 2.f, 0.05f, 0.2f);
//		TrapePlanner PathPlanner_4 = TrapePlanner(0.10f, 0.40f, 2.f, 0.05f, 0.2f);
};
	


#endif





