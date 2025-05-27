#ifndef __ACTION_H
#define __ACTION_H
#include "drive_uart.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif 

	
#define PI 3.14159265358979f


#define INSTALL_ERROR_X		0.0
#define INSTALL_ERROR_Y		0.0


typedef struct ROBOT_REAL_POS
{
  float world_x;
  float world_y;     
  float world_w;
}ROBOT_REAL_POS;

typedef struct ACTION_GL_POS
{
	float ANGLE_Z;
	float POS_X;
	float POS_Y;
	float Speed_X;
	float Speed_Y;
	

	float LAST_POS_X;
	float LAST_POS_Y;

	float DELTA_POS_X;
	float DELTA_POS_Y;
	
	float REAL_X;
	float REAL_Y;
}ACTION_GL_POS;




extern ROBOT_REAL_POS ROBOT_REAL_POS_DATA;







void POS_Change(float X, float Y);



uint32_t Action_UART3_RxCallback(uint8_t *buf, uint16_t len);

void Update_Action_gl_position(float value[5]);
	

#ifdef __cplusplus
}
#endif
	

#endif
