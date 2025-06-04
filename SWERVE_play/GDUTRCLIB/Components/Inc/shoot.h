#pragma once

#include <stdint.h>


#ifdef __cplusplus

#endif
extern "C" {
	
	
float GetShootSpeed(float distance, uint8_t pitch_level);
float GetHoopAngle(float robot_x, float robot_y, float *distance);
float CalcSpeed(float distance, float cubic_spline[][4], float *sample_distance, uint16_t num);
	
	
	
#ifdef __cplusplus
}
#endif
