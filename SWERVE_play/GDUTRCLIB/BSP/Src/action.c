#include "action.h"
#include <math.h>

ACTION_GL_POS ACTION_GL_POS_DATA = {0};


ROBOT_REAL_POS ROBOT_REAL_POS_DATA = {0};




uint32_t Action_UART4_RxCallback(uint8_t *buf, uint16_t len)
{
	
	uint8_t count = 0;
	uint8_t i = 0;
	
	
	union 
	{
		uint8_t data[24];
		float ActVal[6];
	} posture;
	
	uint8_t break_flag = 1;
	
	
	while(i < len && break_flag == 1)
	{
		switch (count)
		{
			case 0:
			{
				if (buf[i] == 0x0d)
				{
					count++;
				}
				else
				{
					count = 0;
				}
				i++;
				break;
			}
			
			case 1:
			{
				if (buf[i] == 0x0a)
				{
					count++;
				}
				else
				{
					count = 0;
				}
				i++;
				break;
			}
			
			case 2:
			{
				uint8_t j;
				
				
				if (i > len - 26)
				{
					break_flag = 0;
				}
				
				for(j = 0; j < 24; j++)
				{
					posture.data[j] = buf[i];
					i++;
				}
				count++;
				break;
			}
			
			case 3:
			{
				if (buf[i] == 0x0a)
				{
					count++;
				}
				else
				{
					count = 0;
				}
				i++;
				break;
			}
			
			case 4:
			{
				if (buf[i] == 0x0d)
				{	
					Update_Action_gl_position(posture.ActVal);
				}
				count = 0;
				
				break_flag = 0;
				
				break;
			}
			
			default:
			{
				count = 0;
				break;
			}
		}
		
	}
	
	
	return 0;
}




void Update_Action_gl_position(float value[6])
{
	
	ACTION_GL_POS_DATA.LAST_POS_X = ACTION_GL_POS_DATA.POS_X;
	ACTION_GL_POS_DATA.LAST_POS_Y = ACTION_GL_POS_DATA.POS_Y;

	
	ACTION_GL_POS_DATA.ANGLE_Z = value[0];
	ACTION_GL_POS_DATA.ANGLE_X = value[1];
	ACTION_GL_POS_DATA.ANGLE_Y = value[2];
	ACTION_GL_POS_DATA.POS_X = value[3]; 
	ACTION_GL_POS_DATA.POS_Y = value[4]; 
	ACTION_GL_POS_DATA.W_Z = value[5];

	
	ACTION_GL_POS_DATA.DELTA_POS_X = ACTION_GL_POS_DATA.POS_X - ACTION_GL_POS_DATA.LAST_POS_X;
	ACTION_GL_POS_DATA.DELTA_POS_Y = ACTION_GL_POS_DATA.POS_Y - ACTION_GL_POS_DATA.LAST_POS_Y;

	
	ROBOT_REAL_POS_DATA.world_w = ACTION_GL_POS_DATA.ANGLE_Z;
	//ROBOT_CHASSI.world_w = ROBOT_REAL_POS_DATA.world_w;


	ACTION_GL_POS_DATA.REAL_X += (ACTION_GL_POS_DATA.DELTA_POS_X);
	ACTION_GL_POS_DATA.REAL_Y += (ACTION_GL_POS_DATA.DELTA_POS_Y);
	

	ROBOT_REAL_POS_DATA.world_x = ACTION_GL_POS_DATA.REAL_X + INSTALL_ERROR_X * sinf(ROBOT_REAL_POS_DATA.world_w * PI / 180.f);
	//ROBOT_CHASSI.world_x = ROBOT_REAL_POS_DATA.world_x;
	ROBOT_REAL_POS_DATA.world_y = ACTION_GL_POS_DATA.REAL_Y + INSTALL_ERROR_Y * cosf(ROBOT_REAL_POS_DATA.world_w * PI / 180.f);
	//ROBOT_CHASSI.world_y = ROBOT_REAL_POS_DATA.world_y;
	
	
}






