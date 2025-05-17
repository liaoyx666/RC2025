#include "dribble_ball.h"






void Dribble_Ball(CONTROL_T *ctrl)
{
	static uint32_t start_time;
	static uint32_t now_time;
	
	static uint8_t flag = 0;
	
	if (ctrl->cylinder_ctrl == CYLINDER_DRIBBLE)
	{
		Holding_Cylinder_State(CYLINDER_STRETCH);
		Hiting_Cylinder_State(CYLINDER_STRETCH);
		
		if (flag == 0)
		{
			start_time = Get_SystemTimer();
			flag = 1;
		}
	}
	
	now_time = Get_SystemTimer();
	
	if ((now_time - start_time) > 676000)
	{
		Holding_Cylinder_State(CYLINDER_SHRINK);
		Hiting_Cylinder_State(CYLINDER_SHRINK);
		
		flag = 0;
//		ctrl->cylinder_ctrl == CYLINDER_KEEP;
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





