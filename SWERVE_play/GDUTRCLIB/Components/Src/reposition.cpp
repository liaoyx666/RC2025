#include "reposition.h"

#define PI 3.1415926535897932384626

/**
 * 检查一个double值是否为NaN
 * @param x 要检查的值
 * @return 如果是NaN返回true，否则返回false
 */
bool is_nan(double x) {
    return x != x;
}

/**
 * 计算数组的平均值，自动跳过NaN值
 * @param arr 输入数组
 * @param n 数组长度
 * @return 有效数据的平均值，如果没有有效数据则返回0
 */
double calculate_mean(double *arr, uint8_t n) {
    if (n == 0) return 0.0;
    
    double sum = 0.0;
    uint8_t valid_count = 0;

    for (uint8_t i = 0; i < n; i++) {
        if (!is_nan(arr[i])) {
            sum += arr[i];
            valid_count++;
        }
    }
    
    return (valid_count > 0) ? (sum / valid_count) : 0.0;
}

/**
 * 过滤掉偏离原始平均值超过阈值的数据点并计算新的平均值
 * @param input 输入数组
 * @param n 数组长度
 * @param threshold 过滤阈值（绝对值）
 * @return 过滤后的平均值，如果没有有效数据则返回0
 */
double filter_and_calculate_mean(double *input, uint8_t n, double threshold) {
    if (n == 0) return 0.0;
    
    // 计算原始数据的有效统计量
    double origin_sum = 0.0;
    uint8_t origin_count = 0;
    
    for (uint8_t i = 0; i < n; i++) {
        if (!is_nan(input[i])) {
            origin_sum += input[i];
            origin_count++;
        }
    }
    
    if (origin_count == 0) return 0.0;  // 全部为NaN的情况
    
    double origin_mean = origin_sum / origin_count;
    threshold = fabs(threshold);  // 确保阈值为正
    
    // 过滤数据并计算新的统计量
    double filtered_sum = 0.0;
    uint8_t filtered_count = 0;
    
    for (uint8_t i = 0; i < n; i++) {
        if (is_nan(input[i])) continue;  // 跳过NaN值
        
        if (fabs(input[i] - origin_mean) <= threshold) {
            filtered_sum += input[i];
            filtered_count++;
        }
    }
    
    return (filtered_count > 0) ? (filtered_sum / filtered_count) : 0.0;
}



///////////////////////////////////////////////////////////////////////

#define GET_TIME_1 850000
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
	if (fabsf(RealPosData.world_yaw) > 0.25f)
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
bool RePosition::CalcOffset(double *offset_x, double *offset_y)
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
	
	*offset_x = position_x - laser_x;
	*offset_y = position_y - laser_y;
	
	if (*offset_x * *offset_x + *offset_y * *offset_y > 3 * 3)
	{
		return false;
	}
	
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




#define SAMPLE_TIME 10000

void RePosition::LaserRePosition(CONTROL_T *ctrl)
{
	static uint8_t flag = 0;
	static uint8_t yaw_total_num = 0;
	static uint8_t offset_total_num = 0;
	static uint32_t start_time = Get_SystemTimer();
	
	uint32_t current_time = Get_SystemTimer();
	
	
	
	if (ctrl->yaw_ctrl == YAW_LOCK_DIRECTION)//yaw锁定正方向模式下
	{
		//等待开始信号
		if ((ctrl->reposition_ctrl == REPOSITION_ON) && (flag == 0))
		{
			flag = 1;
		}
	

		//稳定机器人
		if (flag == 1)
		{
			double y;
			GetXYFromLaser(NULL, &y);
			
			//使车停止运动
			ctrl->twist.linear.x = 0;
			ctrl->twist.linear.y = 0;
			
			//太远yaw不准
			if (fabs(y) > 0.6) flag = 0;
			
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
		
		//////////////////////////////////////////////////////////////////////////////////////////////
		
		
		//计算yaw偏移
		if (flag == 2)
		{
			ctrl->twist.linear.x = 0;
			ctrl->twist.linear.y = 0;
			ctrl->twist.angular.z = 0;
			
			static double yaw_rror[10];
			double error = CalcYawError();
			
			
			if (yaw_total_num < 10)
			{
				if ((yaw_total_num == 0) || (current_time - start_time > SAMPLE_TIME))
				{
					start_time = current_time;
					if (fabs(error) < 5.0)
					{
						yaw_rror[yaw_total_num] = error;
						yaw_total_num++;
					}
				}
			}
			else
			{
				this->axis_error = filter_and_calculate_mean(yaw_rror, yaw_total_num, 0.5);
				yaw_total_num = 0;
				flag = 3;
			}
		}
		else
		{
			yaw_total_num = 0;
		}
		///////////////////////////////////////////////////////////////////////////////////////////////////
		
		
		//应用yaw偏移
		if (flag == 3)
		{
			if(ApplyYawError())
			{
				flag = 4;
			}
		}
		

		//稳定机器人
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
		////////////////////////////////////////////////////////////////////////////////////////
		
		//计算XY偏移
		if (flag == 5)
		{
			ctrl->twist.linear.x = 0;
			ctrl->twist.linear.y = 0;
			ctrl->twist.angular.z = 0;

			static double offset_x_arr[5], offset_y_arr[5];
			double offset_x = 0, offset_y = 0;
			
			if (offset_total_num < 5)
			{
				if ((offset_total_num == 0) || (current_time - start_time > SAMPLE_TIME))
				{
					if (CalcOffset(&offset_x, &offset_y))
					{
						offset_x_arr[offset_total_num] = offset_x;
						offset_y_arr[offset_total_num] = offset_y;
						offset_total_num++;
					}
				}
			}
			else
			{
				this->x_offset = filter_and_calculate_mean(offset_x_arr, offset_total_num, 0.015);
				this->y_offset = filter_and_calculate_mean(offset_y_arr, offset_total_num, 0.015);
				offset_total_num = 0;
				flag = 6;
			}
		}
		else
		{
			offset_total_num = 0;
		}
		/////////////////////////////////////////////////////////////////////////////////////////////////
		
		//应用XY偏移
		if (flag == 6)
		{			
			if (ApplyOffset())
			{
				flag = 7;
			}
			else
			{
				flag = 0;
			}
		}
		
		
		//稳定机器人
		if (flag == 7)
		{
			uint8_t state = StabilzeRobot(ctrl);
			if (state == 2) 
			{}
			else if(state == 1)
			{
				//停止旋转
				ctrl->twist.angular.z = 0;
				flag = 8;
			}
			else
			{
				flag = 7;
			}
		}
		
		//测试校准结果
		if (flag == 8)
		{
			ctrl->twist.linear.x = 0;
			ctrl->twist.linear.y = 0;
		
			double X, Y, x, y;
			GetXYFromLaser(&X, &Y);
			CaliLaserData(X, Y, &x, &y, sinf(RealPosData.world_yaw * PI / 180.f), cosf(RealPosData.world_yaw * PI / 180.f));
	
			
			if ((x - RealPosData.world_x) * (x - RealPosData.world_x) + (y - RealPosData.world_y) * (y - RealPosData.world_y) > 0.015 * 0.015)
			{				
				flag = 4;
			}
			else
			{
				flag = 0;
			}
		}
		
		
		
		//保护
		if (flag > 8)
		{
			flag = 0;
		}
	}
	else
	{
		flag = 0;//退出校准
	}
}
