#include "location_task.h"
#include "LaserPositioning.h"

void Location_Task(void *pvParameters)
{
	for (;;)
	{
		WorldXYCoordinatesTypedef WorldXYCoordinates;
		
		LaserPositioning(3.14159265f / 2.f * 3.f, &WorldXYCoordinates);
		
		
		
		
		
		
		
		
		osDelay(1);
	}
}



