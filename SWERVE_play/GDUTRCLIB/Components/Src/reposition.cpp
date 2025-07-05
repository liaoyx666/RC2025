#include "reposition.h"

#define PI 3.1415926535897932384626

bool is_nan(float x)
{
    return x != x; // NaN是唯一不满足x == x的值
}


float GetYawError(float target, float current)
{
	float error = target - current;
	// 将误差归一化到-180度到180度之间
	while (error > 180.0f)
	{
		error -= 360.0f;
	}
	
	while (error < -180.0f)
	{
		error += 360.0f;
	}
	return error;
}


void RePosition::GetLaserData(float* x, float* y)
{
	*x = -(LaserModuleDataGroup.LaserModule1.MeasurementData.Distance / 1000.f);
	*y =  (LaserModuleDataGroup.LaserModule2.MeasurementData.Distance / 1000.f);
}


float RePosition::CalcDistance(PointVector point1, PointVector point2)
{
	return sqrtf(powf(point1.x - point2.x, 2) + powf(point1.y - point2.y, 2));
}


void RePosition::LaserRePosition(CONTROL_T *ctrl)
{
	static uint8_t flag = 0;
	static uint32_t time;
	
	if (ctrl->yaw_ctrl == YAW_LOCK_DIRECTION)//yaw锁定正方向模式下
	{
		uint32_t current_time = Get_SystemTimer();//记录当前时间
		
		//等待第一个点的采样信号
		if ((ctrl->reposition_ctrl == REPOSITION_ON) && (flag == 0))
		{
			flag = 1;
		}
		
		//采样第一个点
		if (flag == 1)
		{
			uint8_t status;
			status = GetPoint(&this->last_point, ctrl);//获取第一个点
			if (status == 1)
			{
				//成功
				flag = 2;
			}
			else if (status == 2)
			{
				//等待
			}
			else
			{
				//失败
				flag = 0;
			}
		}
		
		//等待第二个点的采样信号
		if ((flag == 2) && (ctrl->reposition_ctrl == REPOSITION_ON))
		{
			if (CalcDistance(this->last_point.position_point, PointVector(RealPosData.raw_x, RealPosData.raw_y)) > 2.5f)//两点距离大于1.5m时采样
			{
				flag = 3;
			}
		}
		
		//采样第二个点
		if (flag == 3)
		{
			uint8_t status;
			status = GetPoint(&this->current_point, ctrl);//获取第二个点
			if (status == 1)
			{
				//成功
				flag = 4;
			}
			else if (status == 2)
			{
				//等待
			}
			else
			{
				//失败
				flag = 2;
			}
		}
		
		//计算结果
		if (flag == 4)
		{
			ctrl->twist.linear.x = 0;
			ctrl->twist.linear.y = 0;
			if (CalcCalibrationData())//计算校准数据
			{
				ApplyCaliYawData();
				flag = 5;//成功
			}
			else
			{
				flag = 2;//失败
			}
		}
		
	
		
		
		if (flag == 5)
		{
			uint8_t status;
			status = GetPoint(&this->stable_point, ctrl);//
			if (status == 1)
			{
				//成功
				//last_point
				flag = 6;
			}
			else if (status == 2)
			{
				//等待
			}
			else
			{
				//失败
				flag = 5;///////////////
			}
		}
		
		
		
		
		
		
		if (flag == 6)
		{
			if (CalcOffset() == true)
			{
				ApplyCaliOffsetData();//应用校准数据
				flag = 0;
			}
			else
			{
				flag = 7;
			}
		}
		
		
		if ((flag == 7) && (ctrl->reposition_ctrl == REPOSITION_ON))
		{
			flag = 5;//多次校准
		}


	}
	else
	{
		flag = 0;//退出校准
	}
}


#define GET_TIME_1 1000000
#define GET_TIME_MAX 1600000

/**
  * @brief 获取一个点的激光，position数据
  * @note 
  * @param *point 点数据结构体指针
  * @retval 0 失败；1 成功；2 等待
  */
uint8_t RePosition::GetPoint(SamplePoint *point, CONTROL_T *ctrl)
{
	static uint8_t flag = 0;
	static uint32_t time;
	static uint32_t start_time;
	
	//获取当前时间
	uint32_t current_time = Get_SystemTimer();
	
	//使车静止运动
	ctrl->twist.linear.x = 0;
	ctrl->twist.linear.y = 0;
	
	//记录开始时间并初始化time
	if (flag == 0)
	{
		time = current_time;
		start_time = current_time;
		flag = 1;
	}
	
	//等待pid锁yaw至车稳定
	if (fabsf(GetYawError(target_yaw, RealPosData.world_yaw)) > 0.20f)
	{
		time = current_time;
	}
	
	//等待一段时间开始采样
	if (current_time - time > GET_TIME_1)
	{
		float x, y;
		
		//停止旋转
		ctrl->twist.angular.z = 0;
		
		//获取激光数据
		GetLaserData(&x, &y);
		
		//获取锁yaw不准的误差
		float yaw_error = GetYawError(target_yaw, RealPosData.world_yaw);
		
		//postion与激光数据相差过大
		if ((pow(x - RealPosData.raw_x, 2) + pow(y - RealPosData.raw_y, 2)) > 6.f)
		{
			flag = 0;
			return 0;
		}
		else
		{	//记录激光数据
			point->laser_point = PointVector(static_cast<double>(x), static_cast<double>(y));
			
			//记录position数据
			point->position_point = PointVector(static_cast<double>(RealPosData.raw_x), static_cast<double>(RealPosData.raw_y));
			
			//记录锁yaw的误差
			point->yaw_error = static_cast<double>(yaw_error);
			
			flag = 0;
			return 1;
		}
	}
	
	//车不稳定超过最大等待时间
	if (current_time - start_time > GET_TIME_MAX)
	{
		flag = 0;
		return 0;
	}
	
	return 2;
}


#define MIN_VECTOR_LENGTH 1.45f  // 最小有效向量长度
#define MAX_AXIS_ERROR 20.f      // 最大角度误差

bool RePosition::CalcCalibrationData(void)
{
    // 计算向量差
    PointVector laser_vector    = PointVector(this->last_point.laser_point.x - this->current_point.laser_point.x,
                                              this->last_point.laser_point.y - this->current_point.laser_point.y);
                                        
    PointVector position_vector = PointVector(this->last_point.position_point.x - this->current_point.position_point.x,
                                              this->last_point.position_point.y - this->current_point.position_point.y);
    
    // 计算向量模长的平方
    double laser_length_sq    = laser_vector.x * laser_vector.x + laser_vector.y * laser_vector.y;
    double position_length_sq = position_vector.x * position_vector.x + position_vector.y * position_vector.y;
    
	
    // 检查向量长度是否足够大，避免数值不稳定
    if (laser_length_sq < MIN_VECTOR_LENGTH * MIN_VECTOR_LENGTH || position_length_sq < MIN_VECTOR_LENGTH * MIN_VECTOR_LENGTH)
	{
        return false; // 向量长度过小，数据不可靠
    }
    
    // 计算向量模长
    double laser_length    = sqrt(laser_length_sq);
    double position_length = sqrt(position_length_sq);
    
	
    // 计算点积 (用于cos)
    double dot_product   = laser_vector.x * position_vector.x + laser_vector.y * position_vector.y;
    
    // 计算叉积 (用于sin)
    double cross_product = laser_vector.x * position_vector.y - laser_vector.y * position_vector.x;
    
	
    // 计算cos和sin
    this->cos_axis_error = dot_product / (laser_length * position_length);
    this->sin_axis_error = cross_product / (laser_length * position_length);
    
    // 确保值在有效范围内 (处理浮点误差)
    this->cos_axis_error = fmin(1.0, fmax(-1.0, cos_axis_error));
    this->sin_axis_error = fmin(1.0, fmax(-1.0, sin_axis_error));
    
	
    // 计算轴误差角度 (使用atan2结合dot_product和cross_product)
    this->axis_error = atan2(cross_product, dot_product) * 180.0 / PI;
    
	
    if (is_nan(axis_error))
    {
        return false; // 计算失败
    }
    
	
    // 检查角度误差是否超出允许范围
    if (fabsf(this->axis_error) > MAX_AXIS_ERROR)
    {
        return false; // 角度误差过大
    }
    
    return true; // 校准成功
}


//激光中心到position中心的距离
#define LASER_CENTRE_ERROR_X 0.0922
#define LASER_CENTRE_ERROR_Y -0.0456


//激光安装位置到两激光交点的距离
#define LASER_INSTALL_ERROR_X -0.208
#define LASER_INSTALL_ERROR_Y 0.6165

//y轴正方向为上，x轴正方向为左
void RePosition::CaliLaserData(PointVector in_point, double *out_x, double *out_y, double sin_error, double cos_error)
{
	//加上激光安装位置到两激光交点的距离
	*out_x = in_point.x + LASER_INSTALL_ERROR_X;
	*out_y = in_point.y + LASER_INSTALL_ERROR_Y;
	
	
	*out_x -= sin_error / cos_error * LASER_CENTRE_ERROR_Y;
	*out_y -= sin_error / cos_error * LASER_CENTRE_ERROR_X;
	
	
	//修正激光斜射的误差
	*out_x *= cos_error;
	*out_y *= cos_error;
}

//变换position坐标
void RePosition::CaliPositionData(PointVector in_point, double *out_x, double *out_y, double sin_error, double cos_error)
{
    *out_x =  in_point.x * cos_error + in_point.y * sin_error;
    *out_y = -in_point.x * sin_error + in_point.y * cos_error;
}


//计算激光与position坐标系偏差
bool RePosition::CalcOffset(void)
{
	double sin_offset, cos_offset;//车角度偏差
	double sin_yaw_error, cos_yaw_error;//锁yaw的误差
	
	//校准后的激光，position值 
	double cali_laser_x, cali_laser_y;
	double cali_position_x, cali_position_y;
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	sin_yaw_error = sin(this->last_point.yaw_error * PI / 180.0);
	cos_yaw_error = cos(this->last_point.yaw_error * PI / 180.0);
	
	sin_offset = this->sin_axis_error * cos_yaw_error + this->cos_axis_error * sin_yaw_error;
	cos_offset = this->cos_axis_error * cos_yaw_error - this->sin_axis_error * sin_yaw_error;
	
	CaliLaserData(this->last_point.laser_point, &cali_laser_x, &cali_laser_y, sin_offset, cos_offset);
	CaliPositionData(this->last_point.position_point, &cali_position_x, &cali_position_y, this->sin_axis_error, this->cos_axis_error);
	
	
	if (is_nan(cali_laser_x) || is_nan(cali_laser_y) || is_nan(cali_position_x) || is_nan(cali_position_y))
	{
		return false;
	}
	
	this->last_point.offset_vector.x = cali_position_x - cali_laser_x;
	this->last_point.offset_vector.y = cali_position_y - cali_laser_y;
	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	sin_yaw_error = sin(this->current_point.yaw_error * PI / 180.0);
	cos_yaw_error = cos(this->current_point.yaw_error * PI / 180.0);

	
	sin_offset = this->sin_axis_error * cos_yaw_error + this->cos_axis_error * sin_yaw_error;
	cos_offset = this->cos_axis_error * cos_yaw_error - this->sin_axis_error * sin_yaw_error;
	
	CaliLaserData(this->current_point.laser_point, &cali_laser_x, &cali_laser_y, sin_offset, cos_offset);
	CaliPositionData(this->current_point.position_point, &cali_position_x, &cali_position_y, this->sin_axis_error, this->cos_axis_error);
	
	
		
	if (is_nan(cali_laser_x) || is_nan(cali_laser_y) || is_nan(cali_position_x) || is_nan(cali_position_y))
	{
		return false;
	}

	current_point.offset_vector.x = cali_position_x - cali_laser_x;
	current_point.offset_vector.y = cali_position_y - cali_laser_y;
	
	if (CalcDistance(current_point.offset_vector, last_point.offset_vector) > 0.15f)
	{
		return false;
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	sin_yaw_error = sin(this->current_point.yaw_error * PI / 180.0);
	cos_yaw_error = cos(this->current_point.yaw_error * PI / 180.0);
	
	
	CaliLaserData(this->stable_point.laser_point, &cali_laser_x, &cali_laser_y, sin_yaw_error, cos_yaw_error);
	CaliPositionData(this->stable_point.position_point, &cali_position_x, &cali_position_y, this->sin_axis_error, this->cos_axis_error);
	
	
	if (is_nan(cali_laser_x) || is_nan(cali_laser_y) || is_nan(cali_position_x) || is_nan(cali_position_y))
	{
		return false;
	}
	
	this->stable_point.offset_vector.x = cali_position_x - cali_laser_x;
	this->stable_point.offset_vector.y = cali_position_y - cali_laser_y;
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	return true;
}


extern RawPos RawPosData;


float ss, cc;

//应用校准数据
bool RePosition::ApplyCaliYawData(void)
{
	//double temp_sin = RawPosData.sin_yaw_offset, temp_cos = RawPosData.cos_yaw_offset;
	
	RawPosData.sin_yaw_offset = this->sin_axis_error;
	RawPosData.cos_yaw_offset = this->cos_axis_error;
	
	
	RawPosData.yaw_offset = this->axis_error;//atan2(RawPosData.sin_yaw_offset, RawPosData.cos_yaw_offset) * 180.0 / PI;
	
	ss = asinf(RawPosData.sin_yaw_offset);//////
	cc = acosf(RawPosData.cos_yaw_offset);////////
	
	return true;
}



bool RePosition::ApplyCaliOffsetData(void)
{
	RawPosData.x_offset = this->stable_point.offset_vector.x;
	RawPosData.y_offset = this->stable_point.offset_vector.y;
	
	return true;
}





