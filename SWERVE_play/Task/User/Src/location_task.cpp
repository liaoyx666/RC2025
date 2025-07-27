#include "location_task.h"
#include "LaserPositioning.h"
#include "drive_atk_mw1278d_uart.h"
#include "reposition.h"
#include "drive_tim.h"

uint32_t X, Y1, Y2;
uint32_t X_last, Y1_last, Y2_last;

float world_x_last, world_y_last;


double XX, YY, x, y;
double ag;
double er;


void Location_Task(void *pvParameters)
{
	static RePosition position;
	static uint32_t start_time = 0;
	
	for (;;)
	{
		uint32_t current_time = Get_SystemTimer();
		
		
		LaserPositioning(NULL, NULL);
		position.GetXYFromLaser(&XX, &YY);
		position.CaliLaserData(XX, YY, &x, &y, sinf(RealPosData.world_yaw * 3.14159265358979f / 180.f), cosf(RealPosData.world_yaw * 3.14159265358979f / 180.f));
		
		position.GetLaserData(&X, &Y1, &Y2);
		
		
		if ((X == 0 || Y1 == 0 || Y2 == 0))
		{
			LaserModuleGroup_Init();
		}
		
		
		ag = position.GetYawFromLaser() * 180 / PI;
		er = ag - RealPosData.world_yaw;
		
		
		if (current_time - start_time > 1000000)
		{
			if (world_x_last != RealPosData.world_x || world_y_last != RealPosData.world_y)
			{
				if (X_last == X || Y1_last == Y1 || Y2_last == Y2)
				{
					//LaserModuleGroup_Init();
				}
			}
			
			X_last = X;
			Y1_last = Y1;
			Y2_last = Y2;
			
			world_x_last = RealPosData.world_x;
			world_y_last = RealPosData.world_y;
			
			start_time = current_time;
		}
		
		
		
		atk_mw1278d_uart_printf("%f,%f,%d",31.5445f,23.56565f,556);
		
		
		
		
		

		osDelay(100);
	}
}



