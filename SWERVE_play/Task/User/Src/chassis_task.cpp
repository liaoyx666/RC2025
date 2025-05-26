/**
 * @file chassis_task.cpp
 * @author Yang JianYi
 * @brief 舵轮底盘应用文件，包括底盘配置的初始化以及控制接口的调用
 * @version 0.1
 * @date 2024-05-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "chassis_task.h"
#include "speed_plan.h"
#include "dribble_ball.h"
#include "shoot.h"

Omni_Chassis chassis(0.152/2.f, 0.442f/2.f, 4, 1.f); //底盘直径0.442m，轮子半径0.152m，底盘加速度0.5m/s^2
Launcher launch(400.f, 460.f, 570.f);
bool shoot_ready = false;
CONTROL_T ctrl;
volatile float target_angle = 0;
bool spin_state = false;
float shoot_speed = 0;
float distance = 0;

uint8_t c0, c1, c2;


void Chassis_Task(void *pvParameters)
{
    // static CONTROL_T ctrl;
    for(;;)
    {
		c0 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);
		c1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);
		c2 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);		
		
        if(xQueueReceive(Chassia_Port, &ctrl, 1) == pdPASS)
        {
			Dribble_Ball(ctrl.cylinder_ctrl);//运球
			//Shoot_Ball(ctrl.shoot_ctrl);//推球
			launch.PushBall(ctrl.shoot_ctrl);//推球
			
			///////////////////////////////////////////////////	
            //底盘控制、电机控制    
            if(ctrl.chassis_ctrl == CHASSIS_ON)
            {
                chassis.Control(ctrl.twist);
            }
            else
            {
                Robot_Twist_t twist = {0};
                chassis.Control(twist);
            }
			////////////////////////////////////////////////////
			//俯仰
			if (ctrl.spin_ctrl != SPIN_INSIDE)//不是装球状态
			{
//				if(ctrl.pitch_ctrl == PITCH_HAND)
//				{
//					if(ctrl.twist.linear.x>0.5f)
//					{
//						target_angle += 0.15f;
//					}
//					else if(ctrl.twist.linear.x<-0.5f)
//					{
//						target_angle -= 0.15f;
//					}
//					else 
//					{
//					}
//				}
//				else if(ctrl.pitch_ctrl == PITCH_AUTO)//自动俯仰
//				{
//					target_angle = 0;
//				}
//				else
//				{
//					target_angle = 0;
//				}
				target_angle = 0;
			}
			
			
			//限位
			if(target_angle < 0)
			{
				target_angle = 0;
			}
			else if(target_angle > 400)
			{
				target_angle = 400;
			}
			else
			{}
			/////////////////////////////////////////////////////
			//摩擦带
            if(ctrl.friction_ctrl == FRICTION_ON)
            {
				shoot_speed = GetShootSpeed(distance);//获得速度
				
                launch.FrictionControl(true,21000);
            }
            else
            {
				shoot_speed = 0;
				
                launch.FrictionControl(false,0);
            }
			/////////////////////////////////////////////////////
			//旋转
			if (ctrl.spin_ctrl == SPIN_INSIDE)
			{
				if (launch.LauncherMotor[0].get_angle() >= 340)//防止机构干涉
				{
					spin_state = true;
				}
				else
				{
					spin_state = false;
				}
				target_angle = 350;
			}
			else
			{
				spin_state = false;
			}

			if (launch.LauncherMotor[1].get_angle() > 400)//防止机构干涉
			{
				target_angle = 350;
			}
				
			
			launch.PushControl(1);
			
			
			
			launch.PitchControl(target_angle);//控制俯仰
			launch.SpinControl(spin_state);//控制旋转
			//////////////////////////////////////////////////////
			//CAN发送
			chassis.Motor_Control();
            launch.LaunchMotorCtrl();
        }
        osDelay(1);
    }
}


void PidParamInit(void)
{   
    chassis.Pid_Param_Init(0, 12.0f, 0.015f, 0.0f, 16384.0f, 16384.0f, 10); 
    chassis.Pid_Param_Init(1, 12.0f, 0.015f, 0.0f, 16384.0f, 16384.0f, 10); 
    chassis.Pid_Param_Init(2, 12.0f, 0.015f, 0.0f, 16384.0f, 16384.0f, 10); 
	chassis.Pid_Param_Init(3, 12.0f, 0.015f, 0.0f, 16384.0f, 16384.0f, 10); 
	
    chassis.Pid_Mode_Init(0, 0.1f, 0.0f, false, true);
    chassis.Pid_Mode_Init(1, 0.1f, 0.0f, false, true);
    chassis.Pid_Mode_Init(2, 0.1f, 0.0f, false, true);
	chassis.Pid_Mode_Init(3, 0.1f, 0.0f, false, true);
	 
    launch.Pid_Param_Init(0,12.0f, 0.015f, 0.0f, 16384.0f, /*16384.0f*/6000.f, 0);
    launch.Pid_Mode_Init(0,0.1f, 0.0f, false, true);

    launch.Pid_Param_Init(1,12.0f, 0.015f, 0.0f, 16384.0f, /*16384.0f*/6000.f, 0);
    launch.Pid_Mode_Init(1,0.1f, 0.0f, false, true);
	
	
	launch.Pid_Param_Init(2,12.0f, 0.015f, 0.0f, 16384.0f, /*16384.0f*/6000.f, 0);
    launch.Pid_Mode_Init(2,0.1f, 0.0f, false, true);
}
