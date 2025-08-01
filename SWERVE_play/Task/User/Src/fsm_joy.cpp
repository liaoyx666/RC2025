/**
 * @file fsm_joy.cpp
 * @author Yang JianYi
 * @brief 舵轮底盘应用文件，包括上位机控制接口的调用以及stm32手柄的调试，开关是通过宏定义来控制的(USE_ROS_CONTROL)。
 * @version 0.1
 * @date 2024-05-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "fsm_joy.h"
#include "drive_tim.h"
#include "chassis_task.h"
#include "drive_uart.h"

void Air_Joy_Task(void *pvParameters)
{
	static uint16_t last_SWD = 1000;//上一次SWD按键值
	static uint16_t last_SWA = 1000;//上一次SWA按键值
	
    static CONTROL_T ctrl;
    for(;;)
    {
        //遥杆消抖
        if(air_joy.LEFT_X>1450&&air_joy.LEFT_X<1550)
            air_joy.LEFT_X = 1500;
        if(air_joy.LEFT_Y>1450&&air_joy.LEFT_Y<1550)
            air_joy.LEFT_Y = 1500;
        if(air_joy.RIGHT_X>1450&&air_joy.RIGHT_X<1550)
            air_joy.RIGHT_X = 1500;
        if(air_joy.RIGHT_Y>1450&&air_joy.RIGHT_Y<1550)  
            air_joy.RIGHT_Y = 1500;

        //遥控器启动判断
        if(air_joy.LEFT_X!=0||air_joy.LEFT_Y!=0||air_joy.RIGHT_X!=0||air_joy.RIGHT_Y!=0)
        {
            //底盘控制命令
            if(_tool_Abs(air_joy.SWB - 1000) > 450)
            {
                ctrl.twist.linear.x = -(air_joy.RIGHT_Y - 1500)/500.0 * 3;
                ctrl.twist.linear.y = -(air_joy.LEFT_X - 1500)/500.0 * 3;
                ctrl.twist.angular.z = (air_joy.RIGHT_X - 1500)/500.0 * 2;
                //ctrl.twist.angular.x = air_joy.RIGHT_Y;
				/*******************************************************************************/
				if (_tool_Abs(air_joy.SWB - 1500) < 50)//运球、装球模式
				{
					ctrl.yaw_ctrl = YAW_HAND;
					ctrl.chassis_ctrl = CHASSIS_ON;
					ctrl.friction_ctrl = FRICTION_OFF;
					/////////////////////////////////////////////////////////////////////////////
					if (_tool_Abs(air_joy.SWC - 1500) < 50)//运球
					{
						
						ctrl.mode_ctrl = MODE_DRIBBLE;
						
//						float length = sqrtf(ctrl.twist.linear.x * ctrl.twist.linear.x + ctrl.twist.linear.y * ctrl.twist.linear.y);
//						if(length > 0.5f)
//						{
//							float limit = 0.5f / length;
//							ctrl.twist.linear.x *= limit;
//							ctrl.twist.linear.y *= limit;
//						}//限制底盘速度

						
						ctrl.spin_ctrl = SPIN_OUTSIDE;//旋转到外侧
						
						if (_tool_Abs(last_SWD - air_joy.SWD) > 800)//SWD按键值改变，运球一次
						{
							last_SWD = air_joy.SWD;//更新按键值
							ctrl.cylinder_ctrl = CYLINDER_DRIBBLE;
						}
					}
					/////////////////////////////////////////////////////////////////////////////
					else if (_tool_Abs(air_joy.SWC - 1000) < 50)//人工装球
					{
						ctrl.chassis_ctrl = CHASSIS_ON;//
						
						//ctrl.mode_ctrl = MODE_DEFEND;
						ctrl.spin_ctrl = SPIN_OUTSIDE;//旋转到外侧
						ctrl.yaw_ctrl = YAW_LOCK_DIRECTION;//锁正方向
						ctrl.mode_ctrl = MODE_REPOSITION;//重定位
						
						
						if (_tool_Abs(last_SWD - air_joy.SWD) > 800)
						{
							last_SWD = air_joy.SWD;
							static uint8_t flag = 0;
							if (flag == 0)
							{
								ctrl.cylinder_ctrl = CYLINDER_RELEASE;//张开夹爪（人工装球）
								flag = 1;
							}
							else
							{
								ctrl.cylinder_ctrl = CYLINDER_KEEP;//关闭夹爪（保持球）
								flag = 0;
							}
						}
						
						if (_tool_Abs(last_SWA - air_joy.SWA) > 800)
						{
							last_SWA = air_joy.SWA;//更新按键值
							ctrl.reposition_ctrl = REPOSITION_ON;
						}
						
					}
					////////////////////////////////////////////////////////////////////////////
					else//放球到发射机构
					{
						ctrl.mode_ctrl = MODE_LOAD;
						
						if (_tool_Abs(last_SWD - air_joy.SWD) > 800)//
						{
							last_SWD = air_joy.SWD;//更新按键值
							ctrl.load_ctrl = LOAD_ON;
						}
						
					}
					/////////////////////////////////////////////////////////////////////////////
				}
				/********************************************************************************/
				else if (_tool_Abs(air_joy.SWB - 2000) < 50)//射球模式
				{
					ctrl.chassis_ctrl = CHASSIS_ON;
					
					if (_tool_Abs(air_joy.SWA - 2000) < 50)
					{
						ctrl.pass_ctrl = PASS_ON;
					}
					else
					{
						ctrl.pass_ctrl = PASS_OFF;
					}
					
					
					if (_tool_Abs(air_joy.SWC - 1500) < 50)
					{
						ctrl.yaw_ctrl = YAW_HAND;
						ctrl.mode_ctrl = MODE_SHOOT;//使世界坐标为0
					}
					else if (_tool_Abs(air_joy.SWC - 1000) < 50)
					{
						ctrl.yaw_ctrl = YAW_HAND;
						//ctrl.mode_ctrl = MODE_REPOSITION;
						ctrl.mode_ctrl = MODE_SHOOT;
					}
					else if (_tool_Abs(air_joy.SWC - 2000) < 50)
					{
						ctrl.yaw_ctrl = YAW_LOCK_BASKET;
						ctrl.mode_ctrl = MODE_SHOOT;//使世界坐标为0
					}
					else
					{
						ctrl.yaw_ctrl = YAW_HAND;
						ctrl.mode_ctrl = MODE_SHOOT;//使世界坐标为0
					}
					
					
//					if (ctrl.yaw_ctrl == YAW_LOCK_DIRECTION)
//					{
//						
//						if (_tool_Abs(last_SWD - air_joy.SWD) > 800)
//						{
//							last_SWD = air_joy.SWD;//更新按键值
//							ctrl.reposition_ctrl = REPOSITION_ON;
//						}	
//						
//					}
					
					if ((ctrl.yaw_ctrl == YAW_HAND) || (ctrl.yaw_ctrl == YAW_LOCK_BASKET))
					{
						if (_tool_Abs(last_SWD - air_joy.SWD) > 800)
						{
							last_SWD = air_joy.SWD;//更新按键值
							ctrl.shoot_ctrl = SHOOT_ON;
						}	
					}
					
					
				}
				/*********************************************************************************/
            }
			else//锁定
			{
				ctrl.twist.linear.x = 0;
				ctrl.twist.linear.y = 0;
				ctrl.twist.angular.z = 0;
				ctrl.twist.angular.x = 0;
				
                ctrl.chassis_ctrl = CHASSIS_OFF;//底盘关闭
                ctrl.friction_ctrl = FRICTION_OFF;//摩擦带关闭
                ctrl.pitch_ctrl = PITCH_RESET;
                ctrl.shoot_ctrl = SHOOT_OFF;
				ctrl.cylinder_ctrl = CYLINDER_KEEP;
				
				last_SWD = air_joy.SWD;//更新按键值
				last_SWA = air_joy.SWA;//更新按键值
			}
			
            xQueueSend(Chassia_Port, &ctrl, 0);
			
			ctrl.shoot_ctrl = SHOOT_OFF;
			ctrl.load_ctrl = LOAD_OFF;
			ctrl.reposition_ctrl = REPOSITION_OFF;
			
			if (_tool_Abs(air_joy.SWC - 1500) < 50)
			{
				ctrl.cylinder_ctrl = CYLINDER_KEEP;//防止人工装球干扰运球模式
			}
        }
        else
        {
			ctrl.twist.linear.x = 0;
			ctrl.twist.linear.y = 0;
			ctrl.twist.angular.z = 0;
			ctrl.twist.angular.x = 0;
			
			ctrl.chassis_ctrl = CHASSIS_OFF;//底盘关闭
			ctrl.friction_ctrl = FRICTION_OFF;//摩擦带关闭
			ctrl.pitch_ctrl = PITCH_RESET;
			ctrl.shoot_ctrl = SHOOT_OFF;
			ctrl.cylinder_ctrl = CYLINDER_KEEP;
			ctrl.yaw_ctrl = YAW_HAND;
            //ctrl.twist = {0};
        }
        osDelay(1);
    }
}

