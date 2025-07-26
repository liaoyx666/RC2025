#include "location_task.h"
#include "LaserPositioning.h"
#include "drive_atk_mw1278d_uart.h"
#include "reposition.h"


uint32_t X, Y1, Y2;
double XX, YY, x, y;
double ag;
double er;

void Location_Task(void *pvParameters)
{
	static RePosition position;
	for (;;)
	{
		
		LaserPositioning(NULL, NULL);
		position.GetXYFromLaser(&XX, &YY);
		position.CaliLaserData(XX, YY, &x, &y, sinf(RealPosData.world_yaw * 3.14159265358979f / 180.f), cosf(RealPosData.world_yaw * 3.14159265358979f / 180.f));
		
		position.GetLaserData(&X, &Y1, &Y2);
		//position.CaliLaserData(PointVector(X, Y), &XX, &YY, sinf(RealPosData.world_yaw * 3.14159265f / 180.f), cosf(RealPosData.world_yaw * 3.14159265f / 180.f));
		ag = position.GetYawFromLaser() * 180 / PI;
		er = ag - RealPosData.world_yaw;
		
		
		
		
		
		atk_mw1278d_uart_printf("%f,%f,%d",31.5445f,23.56565f,556);
		
		
		
		
		
		
		osDelay(50);
	}
}



