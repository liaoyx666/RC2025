#include "location_task.h"
#include "LaserPositioning.h"

#include "reposition.h"


float X, Y;
double XX, YY;
double ag;
double er;

void Location_Task(void *pvParameters)
{
	static RePosition position(0);
	for (;;)
	{
		
		LaserPositioning(NULL, NULL);
		position.GetLaserData(&X, &Y);
		position.CaliLaserData(PointVector(X, Y), &XX, &YY, sinf(RealPosData.world_yaw * 3.14159265f / 180.f), cosf(RealPosData.world_yaw * 3.14159265f / 180.f));
		ag = atan2(static_cast<double>(-X - Y), 0.695) * 180 / 3.14159265358979;
		er = ag - RealPosData.world_yaw;
		
		
		
		
		osDelay(1);
	}
}



