#pragma once
#include <stdint.h>
#include "LaserPositioning.h"
#include "action.h"
#include <math.h>
#include "chassis_task.h"
#include "drive_tim.h"

#ifdef __cplusplus


typedef struct PointVector
{
	PointVector(float x, float y) : x(x), y(y) {};
	float x;
	float y;
}PointVector;


typedef struct SamplePoint
{
	SamplePoint() 
        : laser_point(0.f, 0.f)
        , position_point(0.f, 0.f)  // 这里给 position_point 初始值，你按实际需求改
		, offset_vector(0.f, 0.f)
        , yaw_error(0.f)  // 初始化 yaw_error
    {}
	PointVector laser_point;
	PointVector position_point;
	PointVector offset_vector;
	float yaw_error;//采样时yaw与target_yaw的误差
};






class RePosition
{
public:
	RePosition(float target_yaw) : 
        axis_error(0),
        cos_axis_error(1),
        sin_axis_error(0)
    {
        
    }
	void LaserRePosition(CONTROL_T *ctrl);
	void GetLaserData(float* x, float* y);
	void CaliLaserData(PointVector in_point, float *out_x, float *out_y, float sin_yaw_error, float cos_yaw_error);
		
protected:
		
		
		
private:
	float CalcDistance(PointVector point1, PointVector point2);
	
	uint8_t GetPoint(SamplePoint *point, CONTROL_T *ctrl);
	bool CalcCalibrationData(void);
	bool CalaOffset(void);
	void CaliPositionData(PointVector in_point, float *out_x, float *out_y, float sin_yaw_error, float cos_yaw_error);
	bool ApplyCaliData(void);




	float target_yaw;

	float axis_error;
	float cos_axis_error, sin_axis_error;

	SamplePoint last_point, current_point;
};




























extern "C" {
#endif
	
	
extern RawPos RawPosData;

	
	
	
#ifdef __cplusplus
}
#endif

