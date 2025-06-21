#include "reposition.h"

#define PI 3.141592653589793238f

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
	
	
	if (ctrl->yaw_ctrl == YAW_LOCK_DIRECTION)
	{
		uint32_t current_time = Get_SystemTimer();
		
		
		//等待第一个点的采样信号
		if ((ctrl->reposition_ctrl == REPOSITION_ON) && (flag == 0))
		{
			flag = 1;
		}
		
		
		//采样第一个点
		if (flag == 1)
		{
			uint8_t status;
			status = GetPoint(&last_point, ctrl);
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
			if (CalcDistance(last_point.position_point, PointVector(RealPosData.world_x, RealPosData.world_y)) > 1.5f)//两点距离大于1.5m时采样
			{
				flag = 3;
			}
		}
		
		
		//采样第二个点
		if (flag == 3)
		{
			uint8_t status;
			status = GetPoint(&current_point, ctrl);
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
			if (CalcCalibrationData())
			{
				flag = 5;//成功
			}
			else
			{
				flag = 2;//失败
			}
		}
		
		if (flag == 5)
		{
			if (CalaOffset() == true)
			{
				ApplyCaliData();
				flag = 2;
				last_point = current_point;//循环采集
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






#define GET_TIME_1 100000
#define GET_TIME_MAX 400000

uint8_t RePosition::GetPoint(SamplePoint *point, CONTROL_T *ctrl)
{
	static uint8_t flag = 0;
	static uint32_t time;
	static uint32_t start_time;
	uint32_t current_time = Get_SystemTimer();
	
	if (flag == 0)
	{
		time = current_time;
		start_time = current_time;
		flag = 1;
	}
	
	
	//车静止
	ctrl->twist.linear.x = 0;
	ctrl->twist.linear.y = 0;
	
	if (fabsf(GetYawError(target_yaw, RealPosData.world_yaw)) > 1)
	{
		time = current_time;//等待pid稳定yaw
	}
	
	
	if (current_time - time > GET_TIME_1)
	{
		ctrl->twist.angular.z = 0;//停止旋转
		
		float x, y;
		
		GetLaserData(&x, &y);
		
		
		
		float yaw_error = GetYawError(target_yaw, RealPosData.world_yaw);
		
	
		if ((pow(x - RealPosData.world_x, 2) + pow(y - RealPosData.world_y, 2)) > 6.f)//postion与激光数据相差过大
		{
			flag = 0;
			return 0;//失败
		}
		else
		{
			point->laser_point = PointVector(x, y);
			point->position_point = PointVector(RealPosData.world_x, RealPosData.world_y);
			point->yaw_error = yaw_error;
			flag = 0;
			return 1;//成功
		}
	}
	
	if (current_time - start_time > GET_TIME_MAX)
	{
		flag = 0;
		return 0;//失败
	}
	
	return 2;//等待获取中
}









#define MIN_VECTOR_LENGTH 1.45f      // 最小有效向量长度 (米)
#define MAX_AXIS_ERROR 20.f // 20度


bool RePosition::CalcCalibrationData(void)
{
    // 计算向量差
    PointVector laser_vector = PointVector(last_point.laser_point.x - current_point.laser_point.x,
                                        last_point.laser_point.y - current_point.laser_point.y);
                                        
    PointVector position_vector = PointVector(last_point.position_point.x - current_point.position_point.x,
                                        last_point.position_point.y - current_point.position_point.y);
    
    // 计算向量模长的平方
    float laser_length_sq = laser_vector.x * laser_vector.x + laser_vector.y * laser_vector.y;
    float position_length_sq = position_vector.x * position_vector.x + position_vector.y * position_vector.y;
    
    // 检查向量长度是否足够大，避免数值不稳定
    if (laser_length_sq < MIN_VECTOR_LENGTH * MIN_VECTOR_LENGTH ||
        position_length_sq < MIN_VECTOR_LENGTH * MIN_VECTOR_LENGTH) {
        return false; // 向量长度过小，数据不可靠
    }
    
    // 计算向量模长
    float laser_length = sqrtf(laser_length_sq);
    float position_length = sqrtf(position_length_sq);
    
    // 计算点积 (用于cos)
    float dot_product = laser_vector.x * position_vector.x + laser_vector.y * position_vector.y;
    
    // 计算叉积 (用于sin)
    float cross_product = (laser_vector.x * position_vector.y - laser_vector.y * position_vector.x);
    
    // 计算cos和sin
    cos_axis_error = dot_product / (laser_length * position_length);
    sin_axis_error = cross_product / (laser_length * position_length);
    
    // 确保值在有效范围内 (处理浮点误差)
    cos_axis_error = fminf(1.0f, fmaxf(-1.0f, cos_axis_error));
    sin_axis_error = fminf(1.0f, fmaxf(-1.0f, sin_axis_error));
    
    // 计算轴误差角度 (使用atan2结合cos和sin)
    axis_error = atan2f(sin_axis_error, cos_axis_error) * 180.f / PI;
    
    if (is_nan(axis_error))
    {
        return false; // 计算失败
    }
    
    // 检查角度误差是否超出允许范围
    if (fabsf(axis_error) > MAX_AXIS_ERROR)
    {
        return false; // 角度误差过大
    }
    
    return true; // 校准成功
}




//激光中心到position中心的距离
#define LASER_CENTRE_ERROR_X 0.f
#define LASER_CENTRE_ERROR_Y -0.04f


//position原点到护栏的距离
#define X_OFFSET -0.40f
#define Y_OFFSET 0.34f


//激光安装位置到两激光交点的距离
#define LASER_INSTALL_ERROR_X -0.208f
#define LASER_INSTALL_ERROR_Y 0.256f






//y轴正方向为上，x轴正方向为左
void RePosition::CaliLaserData(PointVector in_point, float *out_x, float *out_y, float sin_yaw_error, float cos_yaw_error)
{
	//加上激光安装位置到两激光交点的距离
	*out_x = in_point.x + LASER_INSTALL_ERROR_X;
	*out_y = in_point.y + LASER_INSTALL_ERROR_Y;


	//消除旋转时激光中心与position中心的随yaw变化的偏差
	*out_x +=  LASER_CENTRE_ERROR_X * cos_yaw_error + LASER_CENTRE_ERROR_Y * sin_yaw_error;
	*out_y += -LASER_CENTRE_ERROR_X * sin_yaw_error + LASER_CENTRE_ERROR_Y * cos_yaw_error;
	
	
	//修正激光斜射的误差
	*out_x *= cos_yaw_error;
	*out_y *= cos_yaw_error;

	
	//将激光坐标系原点平移到position原点
	//*out_x -= X_OFFSET;
	//*out_y -= Y_OFFSET;
}


void RePosition::CaliPositionData(PointVector in_point, float *out_x, float *out_y, float sin_yaw_error, float cos_yaw_error)
{
    *out_x =  in_point.x * cos_yaw_error + in_point.y * sin_yaw_error;
    *out_y = -in_point.x * sin_yaw_error + in_point.y * cos_yaw_error;
}


bool RePosition::CalaOffset(void)
{
	float sin_offset, cos_offset;
	
	float sin_yaw_error, cos_yaw_error;
	
	float cali_laser_x, cali_laser_y;
	float cali_position_x, cali_position_y;


	sin_yaw_error = sinf(last_point.yaw_error * PI / 180.f);
	cos_yaw_error = cosf(last_point.yaw_error * PI / 180.f);
	
	sin_offset = sin_axis_error * cos_yaw_error + cos_axis_error * sin_yaw_error;
	cos_offset = cos_axis_error * cos_yaw_error - sin_axis_error * sin_yaw_error;
	
	CaliLaserData(last_point.laser_point, &cali_laser_x, &cali_laser_y, sin_offset, cos_offset);
	CaliPositionData(last_point.position_point, &cali_position_x, &cali_position_y, sin_axis_error, cos_axis_error);
	
	
	if (is_nan(cali_laser_x) || is_nan(cali_laser_y) || is_nan(cali_position_x) || is_nan(cali_position_y))
	{
		return false;
	}
	
	last_point.offset_vector.x = cali_position_x - cali_laser_x;
	last_point.offset_vector.y = cali_position_y - cali_laser_y;
	
	
	
	
	
	sin_yaw_error = sinf(current_point.yaw_error * PI / 180.f);
	cos_yaw_error = cosf(current_point.yaw_error * PI / 180.f);
	
	sin_offset = sin_axis_error * cos_yaw_error + cos_axis_error * sin_yaw_error;
	cos_offset = cos_axis_error * cos_yaw_error - sin_axis_error * sin_yaw_error;
	
	CaliLaserData(current_point.laser_point, &cali_laser_x, &cali_laser_y, sin_offset, cos_offset);
	CaliPositionData(current_point.position_point, &cali_position_x, &cali_position_y, sin_axis_error, cos_axis_error);
	
	
		
	if (is_nan(cali_laser_x) || is_nan(cali_laser_y) || is_nan(cali_position_x) || is_nan(cali_position_y))
	{
		return false;
	}

	
	
	current_point.offset_vector.x = cali_position_x - cali_laser_x;
	current_point.offset_vector.y = cali_position_y - cali_laser_y;
	
	
	if (CalcDistance(current_point.offset_vector, last_point.offset_vector) > 0.02f)
	{
		return false;
	}
	

	return true;
}


extern RawPos RawPosData;


bool RePosition::ApplyCaliData(void)
{
	RawPosData.yaw_offset += axis_error;
	RawPosData.x_offset += current_point.offset_vector.x * 0.8f + last_point.offset_vector.x * 0.2f;
	RawPosData.y_offset += current_point.offset_vector.y * 0.8f + last_point.offset_vector.y * 0.2f;
	RawPosData.sin_yaw_offset = sinf(RawPosData.yaw_offset * PI / 180.f);
	RawPosData.cos_yaw_offset = cosf(RawPosData.yaw_offset * PI / 180.f);
	
	
	return true;
}






