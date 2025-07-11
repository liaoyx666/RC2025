#pragma once
#include <stdint.h>
#include "LaserPositioning.h"
#include "action.h"
#include <math.h>
#include "chassis_task.h"
#include "drive_tim.h"

#ifdef __cplusplus




class RePosition
{
public:
	RePosition() : x_offset(0), y_offset(0), axis_error(0) {}
		
		
	void GetLaserData(uint32_t* x, uint32_t* y1, uint32_t* y2);
	void LaserRePosition(CONTROL_T *ctrl);
	double GetYawFromLaser(void);

	void GetXYFromLaser(double *x, double *y);
	void CaliLaserData(double in_x, double in_y, double *out_x, double *out_y, double sin_error, double cos_error);
	void CaliPositionData(double in_x, double in_y, double *out_x, double *out_y, double sin_error, double cos_error);

protected:
	
	
	
private:
	double CalcYawError(void);
	bool CalcOffset(double *offset_x, double *offset_y);
	bool ApplyYawError(void);
	bool ApplyOffset(void);
	uint8_t StabilzeRobot(CONTROL_T *ctrl);




	double axis_error;
	double x_offset, y_offset;
};



extern "C" {
#endif
	
	
extern RawPos RawPosData;

	
	
	
#ifdef __cplusplus
}
#endif

