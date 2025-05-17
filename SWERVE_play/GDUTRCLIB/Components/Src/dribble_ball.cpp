#include "dribble_ball.h"

#define DRIBBLE_TIME_1 30000
#define DRIBBLE_TIME_2 100000
#define DRIBBLE_TIME_3 500000



#define DRIBBLE_TIME_4 100000





void Dribble_Ball(CONTROL_T *ctrl)
{
	static uint32_t start_time;
	static uint8_t flag = 0;
	
	if (ctrl->cylinder_ctrl == CYLINDER_DRIBBLE)
	{
		if (flag == 0)
		{
			Hiting_Cylinder_State(CYLINDER_STRETCH);//击球气缸击球
			start_time = Get_SystemTimer();//获取开始运球时间戳
			flag = 1;
		}
	}
	
	
	if ((flag == 1) && (Get_SystemTimer() - start_time >= DRIBBLE_TIME_1))
	{
		Holding_Cylinder_State(CYLINDER_STRETCH);//夹爪开启
		flag = 2;
	}
	
	if ((flag == 2) && (Get_SystemTimer() - start_time >= DRIBBLE_TIME_1 + DRIBBLE_TIME_2))
	{
		Hiting_Cylinder_State(CYLINDER_SHRINK);//击球气缸收缩
		flag = 3;
	}
	
	if ((flag == 3) && (Get_SystemTimer() - start_time >= DRIBBLE_TIME_1 + DRIBBLE_TIME_2 + DRIBBLE_TIME_3))
	{
		Holding_Cylinder_State(CYLINDER_SHRINK);//夹爪关闭
		flag = 4;
	}
	
	if ((flag == 4) && (Get_SystemTimer() - start_time >= DRIBBLE_TIME_1 + DRIBBLE_TIME_2 + DRIBBLE_TIME_3 + DRIBBLE_TIME_4))//延时，防止运球间隔过快
	{
		flag = 0;
	}
}













void Hiting_Cylinder_State(enum CylinderState state)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, (GPIO_PinState)state);
}

void Holding_Cylinder_State(enum CylinderState state)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, (GPIO_PinState)state);
}


void Pushing_Cylinder_State(enum CylinderState state)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, (GPIO_PinState)state);
}





