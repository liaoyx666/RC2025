#include "location_task.h"
#include "LaserPositioning.h"

WorldXYCoordinatesTypedef WorldXYCoordinates;

float X, Y;

void Location_Task(void *pvParameters)
{
	for (;;)
	{
		
		
		LaserPositioning(3.14159265f / 2.f * 3.f, &WorldXYCoordinates);
		X = LaserModuleDataGroup.LaserModule1.MeasurementData.Distance / 1000.f;
		Y = LaserModuleDataGroup.LaserModule2.MeasurementData.Distance / 1000.f;
		
		
		
		
		
		
		
		osDelay(1);
	}
}



