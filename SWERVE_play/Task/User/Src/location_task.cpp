#include "location_task.h"
#include "LaserPositioning.h"

WorldXYCoordinatesTypedef WorldXYCoordinates;


void Location_Task(void *pvParameters)
{
	for (;;)
	{
		
		
		LaserPositioning(3.14159265f / 2.f * 3.f, &WorldXYCoordinates);
		
		
		
		
		
		
		
		
		osDelay(1);
	}
}



