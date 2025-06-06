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

Omni_Chassis chassis(0.152/2.f, 0.442f/2.f, 4, 2.f); //底盘直径0.442m，轮子半径0.152m，底盘加速度0.5m/s^2
Launcher launch(450.f, 455.f, 2045.f);
bool shoot_ready = false;
CONTROL_T ctrl;
uint8_t pitch_level = 0;//

//volatile float pitch_angle = 0;
float pitch_angle = 0;
bool spin_state = false;
float shoot_speed = 0;
float distance = 0;
float yaw_angle = 0;

//main

uint8_t a5, a3;

void Chassis_Task(void *pvParameters)
{
    // static CONTROL_T ctrl;
    for(;;)
    {
		a3 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
		a5 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
	
        if(xQueueReceive(Chassia_Port, &ctrl, 1) == pdPASS)
        {	
			yaw_angle = GetHoopAngle(RealPosData.world_x, RealPosData.world_y, &distance);
			
			if (ctrl.mode_ctrl == MODE_DRIBBLE)
			{
				Dribble_Ball(ctrl.cylinder_ctrl);//运球
			}
			
			launch.PushBall(ctrl.shoot_ctrl);//推球
			
			///////////////////////////////////////////////////	
            //底盘控制、电机控制    
            if(ctrl.chassis_ctrl == CHASSIS_ON)
            {
				/////////////////////////////////////////
				if (ctrl.yaw_ctrl == YAW_LOCK_DIRECTION)
				{
					chassis.Yaw_Control(yaw_angle, &ctrl.twist);
					//printf_DMA("%f\r\n", ctrl.twist.angular.z);
				}
				else
				{
					
				}
				////////////////////////////////////////		
            }
            else
            {
                //Robot_Twist_t twist = {0};
                //chassis.Control(twist);
				ctrl.twist.angular.z = 0;
				ctrl.twist.angular.x = 0;
				ctrl.twist.linear.x = 0;
				ctrl.twist.linear.y = 0;
            }
			
			
			
			
			
			chassis.World_Coordinate(0, &ctrl.twist);
			////////////////////////////////////////////////////
			//俯仰
			if ((pitch_level == 0) && (distance > 2.5f) && (distance <= 3.5f))
			{
				pitch_level = 1;
			}
			
			if ((pitch_level == 1) && (distance <= 2.0f))
			{
				pitch_level = 0;
			}
			
			if ((pitch_level == 1) && (distance > 3.5f))
			{
				pitch_level = 2;
			}
			
			if ((pitch_level == 2) && (distance <= 3.0f) && (distance > 2.0f))
			{
				pitch_level = 1;
			}
			
			
			
			
			
			
			if (pitch_level == 0)
			{
				pitch_angle = 0;
			}
			
			if (pitch_level == 1)
			{
				pitch_angle = 60;
			}
			
			if (pitch_level == 2)
			{
				pitch_angle = 90;
			}
			
			/////////////////////////////////////////////////////
			//摩擦带
            if(ctrl.friction_ctrl == FRICTION_ON)
            {
				shoot_speed = GetShootSpeed(distance, pitch_level);//获得速度
				
                launch.FrictionControl(true,shoot_speed);
            }
            else
            {
				//shoot_speed = 0;
				
                launch.FrictionControl(false,0);
            }
			/////////////////////////////////////////////////////
			//旋转
//			if (ctrl.spin_ctrl == SPIN_INSIDE)
//			{
//				if (launch.LauncherMotor[0].get_angle() >= 180)//防止机构干涉
//				{
//					spin_state = true;
//				}
//				else
//				{
//					spin_state = false;
//				}
//				pitch_angle = 410;
//			}
//			else
//			{
//				spin_state = false;
//			}

			
			
			

			
			///////////////////////////////////////////////

			
			//pitch_angle = 60;

//			
//			static int16_t i = 0, flag = 0;
//			
//			
//			if (i == 1000)
//			{
//				flag = 1;
//			}
//			
//			if (i == -1000)
//			{
//				flag = 0;
//			}
//			
//			
//			if (flag == 0)
//			{
//				launch.PushControl(1);
//				i++;
//			}
//			
//			
//			if (flag == 1)
//			{
//				launch.PushControl(0);
//				i--;
//			}
			//launch.LauncherMotor[2].Out = 1000;
			
			
			if (ctrl.mode_ctrl == MODE_LOAD)
			{
				launch.LoadBall(ctrl.load_ctrl, &pitch_angle, &spin_state);
			}
			
			
			
			if (launch.LauncherMotor[1].get_angle() > 330)//防止机构干涉
			{
				if (pitch_angle < 390)
				{
					pitch_angle = 390;
				}
			}
			
			
			if (launch.LauncherMotor[0].get_angle() > 180)//防止机构干涉
			{
				//spin_state = true;
			}
			else
			{
				spin_state = false;
			}

			
			
			
			
			
			
			
			
			
			
			//限位
			if(pitch_angle < 0)
			{
				pitch_angle = 0;
			}
			else if(pitch_angle > 450)
			{
				pitch_angle = 450;
			}
			else
			{}
				
				
			chassis.Control(ctrl.twist);	
			launch.PitchControl(pitch_angle);//控制俯仰
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
	
	
	launch.Pid_Param_Init(2,2.4f, 0.015f, 0.0f, 16384.0f, /*16384.0f*/5000.f, 0);
    launch.Pid_Mode_Init(2,0.1f, 0.0f, false, true);
	
	
	
	chassis.Pid_Param_Init_Yaw(0.13f, 0.0f, 0.0f, 2.0f, 2.5f, 0.0f);
	chassis.Pid_Mode_Init_Yaw(0, 0, false, true);
}
