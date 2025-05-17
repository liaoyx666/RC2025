#ifndef __DRIBBLE_H
#define __DRIBBLE_H

#include "main.h"
#include "cmsis_os.h"
#include "drive_tim.h"
#include "pid.h"
#include "chassis_task.h"
//#include "chassis_task.h"

#ifdef __cplusplus
extern "C" {
#endif 


enum CylinderState//气缸状态
{
	CYLINDER_STRETCH = GPIO_PIN_RESET,//伸长
	CYLINDER_SHRINK = GPIO_PIN_SET//收缩
};


void Dribble_Ball(CONTROL_T *ctrl);




void Hiting_Cylinder_State(enum CylinderState state);
void Holding_Cylinder_State(enum CylinderState state);
void Pushing_Cylinder_State(enum CylinderState state);
#ifdef __cplusplus
}
#endif

#endif





