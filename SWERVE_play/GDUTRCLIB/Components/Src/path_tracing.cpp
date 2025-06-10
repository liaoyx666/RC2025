#include "path_tracing.h"
#include "chassis_task.h"



void Path::GetErrorAndDistance(PointVector currentPoint, float *error, float *distance)
{
    // 计算当前点相对于路径起点的向量
    PointVector vectorFromStart = PointVector(
        currentPoint.x - start_point.x, 
        currentPoint.y - start_point.y
    );
    
    // 计算纵向距离（沿路径方向的投影）
    *distance = (
        vectorFromStart.x * direction_vector.x + 
        vectorFromStart.y * direction_vector.y
    ) / max_distance;
    
    // 计算横向误差（到路径的垂直距离）
	
	
	
    *error = sqrtf(
		fabsf(
			vectorFromStart.x * vectorFromStart.x + 
			vectorFromStart.y * vectorFromStart.y - 
			*distance * *distance
		)
    );
    
	
	
	
	
    // 判断点在路径的左侧还是右侧，为误差赋予符号
    float crossProduct = vectorFromStart.x * direction_vector.y - 
                         vectorFromStart.y * direction_vector.x;
    
    if (crossProduct < 0)
    {
        *error = -*error;
    }
}

bool Path::IsArriveEnd(PointVector currentPoint)
{
	if ((fabsf(currentPoint.x - end_point.x) <= arrive_deadzone) && (fabsf(currentPoint.y - end_point.y) <= arrive_deadzone))
	{
		return true;
	}
	else
	{
		return false;
	}
}
	
bool Path::IsArriveStart(PointVector currentPoint)
{
	if ((fabsf(currentPoint.x - start_point.x) <= arrive_deadzone) && (fabsf(currentPoint.y - start_point.y) <= arrive_deadzone))
	{
		return true;
	}
	else
	{
		return false;
	}
}
	
bool Path::IsApproachEnd(PointVector currentPoint)
{
	if ((fabsf(currentPoint.x - end_point.x) <= approach_deadzone) && (fabsf(currentPoint.y - end_point.y) <= approach_deadzone))
	{
		return true;
	}
	else
	{
		return false;
	}
}
	
bool Path::IsApproachStart(PointVector currentPoint)
{
	if ((fabsf(currentPoint.x - start_point.x) <= approach_deadzone) && (fabsf(currentPoint.y - start_point.y) <= approach_deadzone))
	{
		return true;
	}
	else
	{
		return false;
	}
}




bool Path_Tracing::Pid_Param_Init_Path(uint8_t num, float Kp, float Ki, float Kd, float Integral_Max, float OUT_Max, float DeadZone)
{
	switch(num)
	{
		case 1:
			PID_Normal.PID_Param_Init(Kp, Ki, Kd, OUT_Max, Integral_Max,DeadZone);
			break;
		
		case 2:
			PID_Tangential.PID_Param_Init(Kp, Ki, Kd, OUT_Max, Integral_Max,DeadZone);
			break;
		
		default:
			break;
	}
	
	
    return true;
}

bool Path_Tracing::Pid_Mode_Init_Path(uint8_t num, float LowPass_error, float LowPass_d_err, bool D_of_Current, bool Imcreatement_of_Out)
{
	switch(num)
	{
		case 1:
			PID_Normal.PID_Mode_Init(LowPass_error, LowPass_d_err, D_of_Current, Imcreatement_of_Out);
			break;
		
		case 2:
			PID_Tangential.PID_Mode_Init(LowPass_error, LowPass_d_err, D_of_Current, Imcreatement_of_Out);
			break;
		
		default:
			break;
	}
    return true;
}



float a, b;

float lx, ly;




void Path_Tracing::PathTracing(enum CONTROL_E state, PointVector currentPoint, float *speed_x, float *speed_y)
{
	static uint8_t current_section = 0;//当前路段
	static bool diraction = true;//方向
	static uint8_t flag = 0;//状态
	
	static float current = 0;//当前路程
	static float target = 0;//目标路程
	
	float normal_speed = 0;//法向速度
	float tangential_speed = 0;//切向速度
	float error = 0;//法向误差
	float distance = 0;//路程
	float start = 0;//开始路程
	float end = 0;//结束路程
	
	
	//////////////////////////////////////////////////////////////////////////
	if (state == PATH_FORWARD)//向前
	{
		diraction = true;
		if ((path[current_section].IsApproachEnd(currentPoint)))
		{
			flag = 0;//未到达起点，使用pid
			current_section++;
		}
	}
	else if (state == PATH_BACKWARD)//向后
	{
		diraction = false;
		if ((path[current_section].IsApproachStart(currentPoint)))
		{
			flag = 0;
			current_section--;
		}
	}
	///////////////////////////////////////////////////////////////////////////
	
	//限位
	if (current_section < 0)
	{
		current_section = 0;
	}
	else if (current_section > SECTION_NUM - 1)
	{
		current_section = SECTION_NUM - 1;
	}
	
	
	
	//获取法向误差和路程
	path[current_section].GetErrorAndDistance(currentPoint, &error, &distance);
	
	a = error;
	b = distance;
	
	////////////////////////////////////////////////////////////////////////////
	//速度规划,PID方向
	if (diraction == true)
	{
		if (flag == 0)//未到达起点
		{
			target = 0;
		}
		else
		{
			start = 0;
			end = path[current_section].max_distance;
			
			target = path[current_section].max_distance;
		}


		if (path[current_section].IsApproachStart(currentPoint))
		{
			flag = 1;//到达起点，启用速度规划
		}
		
		if (((path[current_section].IsArriveEnd(currentPoint))) || (PathPlanner.GetArrivedFlag()))
		{
			flag = 2;//接近终点，使用pid
		}
		
		
		if (path[current_section].IsArriveEnd(currentPoint))
		{
			flag = 3;//到达终点
		}
	}
	else
	{
		if (flag == 0)//未到达起点
		{
			target = path[current_section].max_distance;
		}
		else
		{
			start = path[current_section].max_distance;
			end = 0;
			
			target = 0;
		}


		if (path[current_section].IsApproachEnd(currentPoint))
		{
			flag = 1;//到达起点，启用速度规划
		}
		
		if (((path[current_section].IsArriveStart(currentPoint))) || (PathPlanner.GetArrivedFlag()))
		{
			flag = 2;//接近终点，使用pid
		}
		
		
		if (path[current_section].IsArriveStart(currentPoint))
		{
			flag = 3;//到达终点
		}
	}
	//////////////////////////////////////////////////////////////
	
	
		
	
	//法向pid计算法向速度，保持小车在直线上
	PID_Normal.current = error;
	PID_Normal.target = 0;
	normal_speed = PID_Normal.Adjust();
	
	
	
	
	
	

	//切向速度计算
	if (flag == 1)
	{
		path[current_section].GetSpeedPlan(&PathPlanner);
		tangential_speed = PathPlanner.Plan(start, end, distance);
	}
	else if ((flag == 2) || (flag == 0))
	{
		PID_Tangential.current = distance;
		PID_Tangential.target = target;
		tangential_speed = PID_Tangential.Adjust();
	}
	else if (flag == 3)
	{
		tangential_speed = 0;
		normal_speed = 0;
	}
	

	
	
	
	
	
	
	
	
	
	
	
	// 速度分解，从路径坐标系转换到全局坐标系
	*speed_y = (tangential_speed * path[current_section].COS_angle + normal_speed * path[current_section].SIN_angle);
	*speed_x = -(tangential_speed * path[current_section].SIN_angle - normal_speed * path[current_section].COS_angle);
		
	lx = tangential_speed;
	ly = normal_speed; 
	
	
}




void Path::CalcSpeedPlan(void)
{
	float vel_2 = MAX_VEL * MAX_VEL;
	
	float accel_s = vel_2 / (2.f * ACCEL);
	float decel_s = vel_2 / (2.f * DECEL);
	
	
	if ((accel_s + decel_s) > max_distance)
	{
		this->max_v = sqrtf(fabsf((2.f * max_distance * ACCEL * DECEL) / (ACCEL + DECEL)));
		this->accel_range = DECEL / (ACCEL + DECEL);
		this->decel_range = ACCEL / (ACCEL + DECEL);
		
	}
	else
	{
		this->max_v = MAX_VEL;
		this->accel_range = accel_s / max_distance;
		this->decel_range = decel_s / max_distance;
	}

}




void Path::GetSpeedPlan(TrapePlanner *planner)
{
	*planner = TrapePlanner(accel_range, decel_range, max_v, 0.8, 0.0);
}
	
