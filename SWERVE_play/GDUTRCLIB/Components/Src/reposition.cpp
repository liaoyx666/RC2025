#include "reposition.h"

#define PI 3.1415926535897932384626

bool is_nan(double x)
{
    return x != x; // NaN是唯一不满足x == x的值
}


///////////////////////////////////////////////////////////////////////

#define GET_TIME_1 800000
#define GET_TIME_MAX 1400000
/**
  * @brief 让机器人稳定
  * @note 
  * @param 
  * @retval 0 失败；1 成功；2 等待
  */
uint8_t RePosition::StabilzeRobot(CONTROL_T *ctrl)
{
	static uint8_t flag = 0;
	static uint32_t time;
	static uint32_t start_time;
	
	//获取当前时间
	uint32_t current_time = Get_SystemTimer();
	
	//使车停止运动
	ctrl->twist.linear.x = 0;
	ctrl->twist.linear.y = 0;
	
	//记录开始时间并初始化time
	if (flag == 0)
	{
		time = current_time;
		start_time = current_time;
		flag = 1;
	}
	
	//防止上次flag未置0
	if (current_time - start_time > GET_TIME_MAX + 10000)
	{
		time = current_time;
		start_time = current_time;
		flag = 1;
	}	
	
	//等待pid锁yaw至车稳定
	if (fabsf(RealPosData.world_yaw) > 0.30f)
	{
		time = current_time;
	}
	
	//
	if (current_time - time > GET_TIME_1)
	{
		//停止旋转
		ctrl->twist.angular.z = 0;
		return 1;
	}
	
	//车不稳定超过最大等待时间
	if (current_time - start_time > GET_TIME_MAX)
	{
		flag = 0;
		return 0;
	}
	
	return 2;
}





void RePosition::GetLaserData(uint32_t* x, uint32_t* y1, uint32_t* y2)
{
    if (x != NULL) *x   = LaserModuleDataGroup.LaserModule1.MeasurementData.Distance;
    if (y1 != NULL) *y1 = LaserModuleDataGroup.LaserModule2.MeasurementData.Distance;
    if (y2 != NULL) *y2 = LaserModuleDataGroup.LaserModule3.MeasurementData.Distance;
}




#define LASER_DISTANCE  695.0//mm

//通过激光获得yaw
double RePosition::GetYawFromLaser(void)
{
	uint32_t y1, y2;
	GetLaserData(NULL, &y1, &y2);
	return atan2(static_cast<double>(static_cast<int32_t>(y1) - static_cast<int32_t>(y2)), LASER_DISTANCE);
}




//激光中心到position中心的距离
#define LASER_CENTRE_ERROR_X 0.0922
#define LASER_CENTRE_ERROR_Y -0.0456


//激光安装位置到两激光交点的距离
#define LASER_INSTALL_ERROR_X -0.208
#define LASER_INSTALL_ERROR_Y 0.6165


//y轴正方向为上，x轴正方向为左
void RePosition::CaliLaserData(double in_x, double in_y, double *out_x, double *out_y, double sin_error, double cos_error)
{
	//加上激光安装位置到两激光交点的距离
	in_x += LASER_INSTALL_ERROR_X;
	in_y += LASER_INSTALL_ERROR_Y;
	
	
	in_x -= sin_error / cos_error * LASER_CENTRE_ERROR_Y;
	in_y -= sin_error / cos_error * LASER_CENTRE_ERROR_X;
	
	
	//修正激光斜射的误差
	*out_x = in_x * cos_error;
	*out_y = in_y * cos_error;
}



//校正position坐标到激光坐标系
void RePosition::CaliPositionData(double in_x, double in_y, double *out_x, double *out_y, double sin_error, double cos_error)
{
    *out_x =  in_x * cos_error + in_y * sin_error;
    *out_y = -in_x * sin_error + in_y * cos_error;
}






//通过激光获得XY
void RePosition::GetXYFromLaser(double *x, double *y)
{
	uint32_t _x, _y1, _y2;
	GetLaserData(&_x, &_y1, &_y2);
	if (x!= NULL) *x = -static_cast<double>(_x) / 1000.0;
	if (y!= NULL) *y = static_cast<double>(_y1 + _y2) / 2000.0;
}


//计算yaw偏差
double RePosition::CalcYawError(void)
{
	double laser_yaw = GetYawFromLaser();
	
	laser_yaw = laser_yaw * 180.0 / PI;
	
	if (fabs(laser_yaw) > 8.0) return PI / 2.0;
	if (fabsf(RealPosData.world_yaw) > 0.5f) return PI / 2.0;
	
	return RealPosData.world_yaw - laser_yaw;
}

//计算XY偏置
bool RePosition::CalcOffset(void)
{
	double laser_x, laser_y;
	double position_x, position_y;
	double yaw_error;
	
	GetXYFromLaser(&laser_x, &laser_y);
	position_x = static_cast<double>(RealPosData.raw_x);
	position_y = static_cast<double>(RealPosData.raw_y);
	yaw_error = RealPosData.world_yaw;
	
	
	if ((laser_x - position_x) * (laser_x - position_x) + (laser_y - position_y) * (laser_y - position_y) > 3 * 3) return false;
	if (fabsf(RealPosData.world_yaw) > 0.5) return false;
	
	
	CaliLaserData(laser_x, laser_y, &laser_x, &laser_y, sin(yaw_error), cos(yaw_error));
	CaliPositionData(position_x, position_y, &position_x, &position_y, RawPosData.sin_yaw_offset, RawPosData.cos_yaw_offset);
	
	
	if (is_nan(laser_x) || is_nan(laser_y) || is_nan(position_x) || is_nan(position_y))
	{
		return false;
	}
	
	
	this->x_offset = position_x - laser_x;
	this->y_offset = position_y - laser_y;
	
	return true;
}


bool RePosition::ApplyYawError(void)
{	

	RawPosData.yaw_offset = this->axis_error + RawPosData.yaw_offset;

	RawPosData.sin_yaw_offset = sin(RawPosData.yaw_offset * PI / 180.0);
	RawPosData.cos_yaw_offset = cos(RawPosData.yaw_offset * PI / 180.0);
	
	return true;
}



bool RePosition::ApplyOffset(void)
{
	RawPosData.x_offset = this->x_offset;
	RawPosData.y_offset = this->y_offset;
	
	return true;
}


void RePosition::LaserRePosition(CONTROL_T *ctrl)
{
	static uint8_t flag = 0;
	
	if (ctrl->yaw_ctrl == YAW_LOCK_DIRECTION)//yaw锁定正方向模式下
	{
		//等待信号
		if ((ctrl->reposition_ctrl == REPOSITION_ON) && (flag == 0))
		{
			flag = 1;
		}
	
		if (flag == 1)
		{
			double y;
			GetXYFromLaser(NULL, &y);
			
			//使车停止运动
			ctrl->twist.linear.x = 0;
			ctrl->twist.linear.y = 0;
			
			//太远yaw不准
			if (fabs(y) > 0.5) flag = 0;
			 
			else
			{
				uint8_t state = StabilzeRobot(ctrl);
				if (state == 2) 
				{}
				else if(state == 1)
				{
					//停止旋转
					ctrl->twist.angular.z = 0;
					flag = 2;
				}
				else
				{
					flag = 1;
				}
			}
		}
		
		
		static uint8_t total_num = 0;
		static uint8_t real_num = 0;
		static double sum_error = 0;
		
		
		if (flag == 2)
		{
			ctrl->twist.linear.x = 0;
			ctrl->twist.linear.y = 0;
			ctrl->twist.angular.z = 0;
			
			double error = CalcYawError();
			
			total_num++;
			
			
			if (fabs(error) < 5.0)
			{
				real_num++;
				sum_error += error;
			}
			
			
			if (total_num >= 10)
			{
				if (real_num > 0) 
				{
					this->axis_error = sum_error / static_cast<double>(real_num);
					total_num = 0;
					real_num = 0;
					sum_error = 0;
					flag = 3;
				}
				else
				{
					this->axis_error = 0;
					flag = 0;
				}
			}
	
		}
		else
		{
			total_num = 0;
			real_num = 0;
			sum_error = 0;
		}
	
		
		
		
		if (flag == 3)
		{
			if(ApplyYawError())
			{
				flag = 4;
			}
		}
		

	
		if (flag == 4)
		{
			uint8_t state = StabilzeRobot(ctrl);
			if (state == 2) 
			{}
			else if(state == 1)
			{
				//停止旋转
				ctrl->twist.angular.z = 0;
				flag = 5;
			}
			else
			{
				flag = 4;
			}
		}
		
		
		static uint8_t num = 0;
		
		if (flag == 5)
		{
			ctrl->twist.linear.x = 0;
			ctrl->twist.linear.y = 0;
			ctrl->twist.angular.z = 0;

			num++;
			
			if (CalcOffset())
			{
				flag = 6;
			}
			
			if (num >= 10)
			{
				flag = 4;
			}
		}
		else
		{
			num = 0;
		}
		
		
		if (flag == 6)
		{
			if (ApplyOffset())
			{
				flag = 0;
			}
			else
			{
				flag = 0;
			}
		}

	}
	else
	{

		flag = 0;//退出校准
	}
}
