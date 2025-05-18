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


Omni_Chassis chassis(0.152/2.f, 0.442f/2.f, 4, 1.f); //底盘直径0.442m，轮子半径0.152m，底盘加速度0.5m/s^2
Launcher launch(400.f,-460.f);
//float pos_set = 0;
bool shoot_ready = false;
CONTROL_T ctrl;
volatile float target_angle = 0;


void Chassis_Task(void *pvParameters)
{
    // static CONTROL_T ctrl;
	
    for(;;)
    {
        if(xQueueReceive(Chassia_Port, &ctrl, pdTRUE) == pdPASS)
        {
			Dribble_Ball(&ctrl);//运球
			
//////////////////////////////////////////////////////////////////				
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
/////////////////////////////////////////////////////////////
            if(ctrl.pitch_ctrl == PITCH_HAND)
            {
                // float target_angle = 0;
                if(ctrl.twist.linear.x>0.5f)
				{
                    target_angle += 0.05f;
				}
                else if(ctrl.twist.linear.x<-0.5f)
				{
                    target_angle -= 0.05f;
				}
                else 
				{
				}
		
                if(target_angle < 0)
				{
                    target_angle = 0;
				}
                else if(target_angle > 400)
				{
                    target_angle = 400;
				}
                else
				{
				/*target_angle=0*/;
				}
				
                launch.PitchControl(target_angle);
            }
            else if(ctrl.pitch_ctrl == PITCH_AUTO)//自动俯仰
            {
				
			}
            else
            {
                launch.PitchControl(0);
            }
///////////////////////////////////////////////////////////////////		
            if(ctrl.friction_ctrl == FRICTION_ON)
            {
                //if(ctrl.shoot_ctrl == SHOOT_OFF)
                    launch.ShootControl(false,true,25000);
               // else
                   //launch.ShootControl(true,true,10000);
            }
            else
            {
                launch.ShootControl(false,false,0);
            }
////////////////////////////////////////////////////////////////////		
			 if(ctrl.shoot_ctrl == SHOOT_ON)
			 {
				 Push_Ball(CYLINDER_SHRINK);
				 
				 //launch.PitchControl(pos_set);
				 //Motor_SendMsgs(&hcan1,launch.LauncherMotor[0]);
		
		
				 //launch.ShootControl(shoot_ready,false,0);
				 //Motor_SendMsgs(&hcan1,launch.LauncherMotor);
			 }
			 else
			 {
				Push_Ball(CYLINDER_STRETCH);
			 }
//////////////////////////////////////////////////////////////////////
			chassis.Motor_Control();
            launch.LaunchMotorCtrl();
             //launch.FrictionMotor[2].Mode = SET_eRPM;
             //launch.FrictionMotor[2].Out=5000;
             //Motor_SendMsgs(&hcan2,launch.FrictionMotor[2]);
        }	
		

		
		
		
		
		
//		 launch.PitchControl(400);
//		launch.LauncherMotor[0].Out = 500;
//        launch.LauncherMotor[1].Out = 1000;

//			Motor_SendMsgs(&hcan1, launch.LauncherMotor[0]);
//			if (i < 2500)
//			{
//				launch.ShootControl(true,0,0 );
//			
//			
//			
//				Motor_SendMsgs(&hcan1, launch.LauncherMotor[1]);
//			}
////		osDelay(pdMS_TO_TICKS(5000));
//		
//			i++;
//		if (i > 2500)
//		{
//			
//		launch.ShootControl(false,0,0 );
//		
//		
//		 Motor_SendMsgs(&hcan1, launch.LauncherMotor[1]);
//		}
//		if (i == 5000)
//		{
//			i = 0;
//		}
		
		 
//		 
//		 osDelay(pdMS_TO_TICKS(5000));
		//launch.FrictionMotor[2].Mode = SET_eRPM;
        //launch.FrictionMotor[2].Out=5000;
//		launch.ShootControl(false,true,5000);
//        Motor_SendMsgs(&hcan2,launch.FrictionMotor[2]);

//		ctrl.twist.linear.x = 0;
//		ctrl.twist.linear.y = 0.2;
//		ctrl.twist.angular.z = 0;
//		
//		chassis.Control(ctrl.twist);
//		
//		
//		chassis.Motor_Control();
		
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
	 
    launch.Pid_Param_Init(0,12.0f, 0.015f, 0.0f, 16384.0f, 16384.0f, 0);
    launch.Pid_Mode_Init(0,0.1f, 0.0f, false, true);

    launch.Pid_Param_Init(1,12.0f, 0.015f, 0.0f, 16384.0f, 16384.0f, 0);
    launch.Pid_Mode_Init(1,0.1f, 0.0f, false, true);
}
