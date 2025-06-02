#include "dribble_ball.h"


//时间段
#define DRIBBLE_TIME_1 20000//击球气缸伸长（开始）到夹球气缸伸长
#define DRIBBLE_TIME_2 100000//夹球气缸伸长到击球气缸收缩
#define DRIBBLE_TIME_3 400000//击球气缸收缩到夹球气缸收缩

#define DRIBBLE_TIME_4 150000//夹球气缸收缩到结束（防止两次运球时间间隔过短）





void Dribble_Ball(enum CONTROL_E state)
{
	static uint32_t start_time;//开始击球1时间
	static uint8_t flag = 0;
	
	
	if ((flag == 0) && (state == CYLINDER_KEEP))
	{
		//默认状态下击球和夹球气缸收缩
		Hiting_Cylinder_State(CYLINDER_SHRINK);
		Holding_Cylinder_State(CYLINDER_SHRINK);
	}
	
	if ((flag == 0) && (state == CYLINDER_RELEASE))
	{
		Hiting_Cylinder_State(CYLINDER_SHRINK);
		Holding_Cylinder_State(CYLINDER_STRETCH);
	}
	
	
	
	if ((flag == 0) && (state == CYLINDER_DRIBBLE))
	{
		Hiting_Cylinder_State(CYLINDER_STRETCH);//击球气缸击球
		start_time = Get_SystemTimer();//获取开始运球时间戳
		flag = 1;
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





//#define SHOOT_TIME_1 400000
//#define SHOOT_TIME_2 100000

//void Shoot_Ball(enum CONTROL_E state)
//{
//	
//	static uint32_t start_time;//开始推球时间
//	static uint8_t flag = 0;
//	
//	
//	if ((flag == 0) && (state == SHOOT_OFF))
//	{
//		Pushing_Cylinder_State(CYLINDER_SHRINK);
//	}
//	
//	
//	
//	if (state == SHOOT_ON)
//	{
//		if (flag == 0)
//		{
//			Pushing_Cylinder_State(CYLINDER_STRETCH);
//			start_time = Get_SystemTimer();//获取开始运球时间戳
//			flag = 1;
//		}
//	}
//	
//	if ((flag == 1) && (Get_SystemTimer() - start_time >= SHOOT_TIME_1))
//	{
//		Pushing_Cylinder_State(CYLINDER_SHRINK);
//		flag = 2;
//	}
//	
//	if ((flag == 2) && (Get_SystemTimer() - start_time >= SHOOT_TIME_1 + SHOOT_TIME_2))
//	{
//		flag = 0;
//	}
//	
//	
//	
//	
//}


















////推球气缸控制
//void Push_Ball(enum CylinderState state)
//{
//	Pushing_Cylinder_State(state);
//}










void Hiting_Cylinder_State(enum CylinderState state)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, (GPIO_PinState)state);
}

void Holding_Cylinder_State(enum CylinderState state)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, (GPIO_PinState)state);
}


//void Pushing_Cylinder_State(enum CylinderState state)
//{
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, (GPIO_PinState)state);
//}





