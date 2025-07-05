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
	PointVector(double x, double y) : x(x), y(y) {};
	double x;
	double y;
}PointVector;


typedef struct SamplePoint
{
	SamplePoint() 
        : laser_point(0, 0)
        , position_point(0, 0)
		, offset_vector(0, 0)
        , yaw_error(0)  // 初始化 yaw_error
    {}
	PointVector laser_point;
	PointVector position_point;
	PointVector offset_vector;
	double yaw_error;//采样时yaw与target_yaw的误差
};


class RePosition
{
public:
	RePosition(float target_yaw) : 
        axis_error(0),
        cos_axis_error(1),
        sin_axis_error(0)
    {}
	void LaserRePosition(CONTROL_T *ctrl);
	void GetLaserData(float* x, float* y);
	void CaliLaserData(PointVector in_point, double *out_x, double *out_y, double sin_error, double cos_error);
protected:
		
		
		
private:
	float CalcDistance(PointVector point1, PointVector point2);
	
	uint8_t GetPoint(SamplePoint *point, CONTROL_T *ctrl);
	bool CalcCalibrationData(void);
	bool CalcOffset(void);
	void CaliPositionData(PointVector in_point, double *out_x, double *out_y, double sin_error, double cos_error);
	bool ApplyCaliYawData(void);
	bool ApplyCaliOffsetData(void);

	float target_yaw;
	double axis_error;
	double cos_axis_error, sin_axis_error;

	SamplePoint last_point, current_point, stable_point;
};



extern "C" {
#endif
	
	
extern RawPos RawPosData;

	
	
	
#ifdef __cplusplus
}
#endif

