#ifndef __ACTION_H
#define __ACTION_H
#include "drive_uart.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif 

	
#define PI 3.14159265358979f


#define INSTALL_ERROR_X		0.0f
#define INSTALL_ERROR_Y		-0.33f


typedef struct RealPos
{
	float world_x;
	float world_y;     
	float world_yaw;
	float speed_x;
	float speed_y;
}RealPos;

typedef struct RawPos
{
	float angle_Z;
	float Pos_X;
	float Pos_Y;
	float Speed_X;
	float Speed_Y;
	

	float LAST_Pos_X;
	float LAST_Pos_Y;

	float DELTA_Pos_X;
	float DELTA_Pos_Y;
	
	float REAL_X;
	float REAL_Y;
}RawPos;




extern RealPos RealPosData;







void POS_Change(float X, float Y);



uint32_t Position_UART3_RxCallback(uint8_t *buf, uint16_t len);

void Update_RawPosition(float value[5]);
	

#ifdef __cplusplus
}
#endif
	

#endif
