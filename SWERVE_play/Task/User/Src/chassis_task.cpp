/**
 * @file chassis_task.cpp
 * @author Yang JianYi
 * @brief 舵轮底盘应用文件，包括底盘配置的初始化以及控制接口的调用
 * @version 0.1
 * 
 * @date 2024-05-16
 * @copyright Copyright (c) 2024
 * 
 */
#include "chassis_task.h"
#include "speed_plan.h"
#include "dribble_ball.h"
#include "shoot.h"
#include "reposition.h"
#include "ws2812.h"
#include "BusServo.h"


Omni_Chassis chassis(0.152/2.f, 0.442f/2.f, 4, 3.f); //底盘直径0.442m，轮子半径0.152m，底盘加速度0.5m/s^2
Launcher launch(1450.f, 455.f, 2045.f);
bool shoot_ready = false;
CONTROL_T ctrl;
uint8_t pitch_level = 0;
BusServo servo1(&huart1,GPIOA, GPIO_PIN_9,1), servo2(&huart1,GPIOA, GPIO_PIN_9,2);

RePosition reposition;

float pitch_angle = 0;
bool spin_state = false;
float shoot_speed = 0;
float distance = 0;
float yaw_angle = 0;
bool friction_ready = false;

Ws2812b_SIGNAL_T Ws2812b_signal = SIGNAL_NORMAL;


float sp = 0;

//main

//uint8_t a5, a3;

void Chassis_Task(void *pvParameters)
{
	servo1.load();
	servo2.load();
	
	servo1.moveToAngle(0,100);
	servo2.moveToAngle(0,100);
    for(;;)
    {
		//a3 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
		//a5 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
	
        if(xQueueReceive(Chassia_Port, &ctrl, 1) == pdPASS)
        {	
			
			if (ctrl.mode_ctrl == MODE_DRIBBLE || ctrl.mode_ctrl == MODE_DEFEND || ctrl.mode_ctrl == MODE_REPOSITION)
			{
				spin_state = false;
				if (ctrl.mode_ctrl == MODE_DRIBBLE)
				{
					pitch_angle = 600;
				}
				else
				{
					pitch_angle = 0;
				}
				
				Dribble_Ball(ctrl.cylinder_ctrl);//运球
			}
			
			launch.PushBall(&ctrl);//推球
			
			///////////////////////////////////////////////////	
            //底盘控制、电机控制    
            if(ctrl.chassis_ctrl == CHASSIS_ON)
            {
				//yaw_angle = GetHoopAngle(RealPosData.world_x, RealPosData.world_y, &distance);
				if (ctrl.pass_ctrl == PASS_ON)
				{
					yaw_angle = GetR1Angle(RealPosData.world_x, RealPosData.world_y, &distance);
				}
				else
				{
					yaw_angle = GetHoopAngle(RealPosData.world_x, RealPosData.world_y, &distance);
				}
				
				
				/////////////////////////////////////////
				if (ctrl.yaw_ctrl == YAW_LOCK_DIRECTION)//锁正方向
				{
					chassis.Yaw_Control(0, &ctrl.twist);
				}
				else if (ctrl.yaw_ctrl == YAW_LOCK_BASKET)//锁框
				{

					if (distance >= 1.f)
					{
						chassis.Yaw_Control(yaw_angle, &ctrl.twist);
					}
					
				}
				else//手动yaw
				{
					
				}
				////////////////////////////////////////		
            }
            else
            {
				ctrl.twist.angular.z = 0;
				ctrl.twist.angular.x = 0;
				ctrl.twist.linear.x = 0;
				ctrl.twist.linear.y = 0;
            }
			
			

			
			//重定位
			reposition.LaserRePosition(&ctrl, &Ws2812b_signal);
			
			
			
			
			if (ctrl.mode_ctrl == MODE_DEFEND)
			{
				chassis.World_Coordinate(180, &ctrl.twist);//世界坐标
			}
			else
			{
				chassis.World_Coordinate(0, &ctrl.twist);//世界坐标
			}
			
			
			//chassis.World_Coordinate(-90, &ctrl.twist);//世界坐标
			////////////////////////////////////////////////////
			//俯仰
//			if ((pitch_level == 0) && (distance > 2.5f) && (distance <= 3.3f))
//			{
//				pitch_level = 1;
//			}
//			
//			if ((pitch_level == 1) && (distance <= 2.0f))
//			{
//				pitch_level = 0;
//			}
//			
//			if ((pitch_level == 1) && (distance > 3.3f))
//			{
//				pitch_level = 2;
//			}
//			
//			if ((pitch_level == 2) && (distance <= 3.0f) && (distance > 2.0f))
//			{
//				pitch_level = 1;
//			}
//			
//			
//			if (distance < 2.0)
//			{
//				pitch_level = 0;
//			}
//			
//			if ((distance > 2.5) && (distance < 3.0))
//			{
//				pitch_level = 1;
//			}
//			
//			if (distance > 3.3)
//			{
//				pitch_level = 2;
//			}
//			
//			
//			
//			
//			
//			if (pitch_level == 0)
//			{
//				pitch_angle = 0;
//			}
//			
//			if (pitch_level == 1)
//			{
//				pitch_angle = 60;
//			}
//			
//			if (pitch_level == 2)
//			{
//				pitch_angle = 90;
//			}
			
			
			
			if (ctrl.mode_ctrl == MODE_SHOOT)
			{
				pitch_angle = 1450;
				spin_state = true;
			}

			
			if (ctrl.mode_ctrl == MODE_REPOSITION)
			{
				pitch_angle = 0;
				spin_state = false;
			}
			
			
			/////////////////////////////////////////////////////
			//摩擦带
            if(ctrl.friction_ctrl == FRICTION_ON && ctrl.mode_ctrl == MODE_SHOOT)
            {
				if (-launch.LauncherMotor[0].get_angle() > 1400)
				{
					shoot_speed = GetShootSpeed(distance, 0);//获得速度
					friction_ready = true;
				}
            }
            else
            {
				friction_ready = false;
				shoot_speed = 0;
            }
			/////////////////////////////////////////////////////
			
			
			
			if (ctrl.mode_ctrl == MODE_LOAD)//装球
			{
				if (pitch_angle > 600)
				{
					pitch_angle = 600;
				}
				
				spin_state = true;
				
				friction_ready = true;
				
				launch.LoadBall(ctrl.load_ctrl, &pitch_angle, &spin_state, &shoot_speed);
			}
			
			
			/////////////////////////////////////////////////////////////////////////////
			if (launch.LauncherMotor[1].get_angle() < 330)//防止机构干涉
			{
				if (pitch_angle > 600)
				{
					pitch_angle = 600;
				}
			}
			
			
			if (-launch.LauncherMotor[0].get_angle() > 620)//防止机构干涉
			{
				spin_state = true;
			}

			
			//限位
			if(pitch_angle < 0)
			{
				pitch_angle = 0;
			}
			else if(pitch_angle > 1450)
			{
				pitch_angle = 1450;
			}
			else
			{}
			
			//
			//pitch_angle = 0;
			//
			
			//pitch_angle = pt;
			//pitch_angle = 0;
			//pitch_angle = 1450;
			//spin_state = true;
				
			//shoot_speed = sp;///////////////////////
			
			chassis.Control(ctrl.twist);//控制底盘
			launch.FrictionControl(friction_ready,shoot_speed);//控制摩擦带
			launch.PitchControl(pitch_angle);//控制俯仰
			launch.SpinControl(spin_state);//控制旋转
			//////////////////////////////////////////////////////
			//CAN发送
			chassis.Motor_Control();
            launch.LaunchMotorCtrl();
				
			//发送灯带信号
			xQueueSend(Send_WS2812_Port, &Ws2812b_signal, 0);
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
	 
    launch.Pid_Param_Init(0,12.0f, 0.015f, 0.0f, 16384.0f, 7000.f, 0);
    launch.Pid_Mode_Init(0,0.1f, 0.0f, false, true);

    launch.Pid_Param_Init(1,12.0f, 0.015f, 0.0f, 16384.0f, 6000.f, 0);
    launch.Pid_Mode_Init(1,0.1f, 0.0f, false, true);
	
	
	launch.Pid_Param_Init(2,2.4f, 0.015f, 0.0f, 16384.0f, 6000.f, 0);
    launch.Pid_Mode_Init(2,0.1f, 0.0f, false, true);
	
	
	
	chassis.Pid_Param_Init_Yaw(0.13f, 0.0f, 0.0f, 3.f, 3.f, 0.0f);
	chassis.Pid_Mode_Init_Yaw(0, 0, false, true);
}
