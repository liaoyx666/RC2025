#include "location_task.h"
#include "LaserPositioning.h"

#include "reposition.h"


float X, Y;
double XX, YY;


void Location_Task(void *pvParameters)
{
	static RePosition position(0);
	for (;;)
	{
		
		LaserPositioning(NULL, NULL);
		position.GetLaserData(&X, &Y);
		position.CaliLaserData(PointVector(X, Y), &XX, &YY, sinf(RealPosData.world_yaw * 3.14159265f / 180.f), cosf(RealPosData.world_yaw * 3.14159265f / 180.f));
	
		
		
		
		
		
		osDelay(1);
	}
}



