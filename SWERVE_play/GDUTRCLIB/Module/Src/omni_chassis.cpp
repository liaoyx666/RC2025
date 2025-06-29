/**
 * @file omni_chassis.cpp
 * @author Yang JianYi (2807643517@qq.com)
 * @brief 全向轮底盘驱动文件，使用该文件，需要创建一个全向轮底盘类(由于这个工程是舵轮底盘工程，所以这个文件没有使用)。如果要使用这个类，需要将舵轮底盘的
 *        调用文件替换为全向轮底盘的调用文件。(chassis_task.cpp),同时把通信文件中的can接收函数进行更改。
 * @version 0.1
 * @date 2024-05-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "chassis_omni.h"
#include <math.h>

extern RealPos RealPosData;


void Omni_Chassis::Control(Robot_Twist_t cmd_vel)
{
    Velocity_Calculate(cmd_vel);
    for(int i=0; i<wheel_num; i++)
    {
        PID_Wheel[i].current = WheelMotor[i].get_speed();
        PID_Wheel[i].target = wheel[i].wheel_vel;
        WheelMotor[i].Out = PID_Wheel[i].Adjust();
    }
}

void Omni_Chassis::Motor_Control(void)
{
    Motor_SendMsgs(&hcan1, WheelMotor);
}

void Omni_Chassis::Velocity_Calculate(Robot_Twist_t cmd_vel)
{
    update_timeStamp();

    //使用加速度控制底盘速度
    if(cmd_vel.linear.x > 0 && cmd_vel.linear.x >= cmd_vel_last.linear.x)
        cmd_vel.linear.x = cmd_vel_last.linear.x + accel_vel*dt;
    else if(cmd_vel.linear.x < 0 && cmd_vel.linear.x <= cmd_vel_last.linear.x)
        cmd_vel.linear.x = cmd_vel_last.linear.x - accel_vel*dt;
    else
    {}

    if(cmd_vel.linear.y > 0 && cmd_vel.linear.y >= cmd_vel_last.linear.y)
        cmd_vel.linear.y = cmd_vel_last.linear.y + accel_vel*dt;
    else if(cmd_vel.linear.y < 0 && cmd_vel.linear.y <= cmd_vel_last.linear.y)
        cmd_vel.linear.y = cmd_vel_last.linear.y - accel_vel*dt;
    else
    {}
        cmd_vel_last = cmd_vel;

    if(wheel_num==4)
    {
        wheel[0].wheel_vel = (-cmd_vel.linear.y*COS45 + cmd_vel.linear.x*COS45 + cmd_vel.angular.z*Chassis_Radius) * ChassisVel_Trans_MotorRPM(Wheel_Radius, 19);
        wheel[1].wheel_vel = (-cmd_vel.linear.y*COS45 - cmd_vel.linear.x*COS45 + cmd_vel.angular.z*Chassis_Radius) * ChassisVel_Trans_MotorRPM(Wheel_Radius, 19);
        wheel[2].wheel_vel = ( cmd_vel.linear.y*COS45 - cmd_vel.linear.x*COS45 + cmd_vel.angular.z*Chassis_Radius) * ChassisVel_Trans_MotorRPM(Wheel_Radius, 19);
        wheel[3].wheel_vel = ( cmd_vel.linear.y*COS45 + cmd_vel.linear.x*COS45 + cmd_vel.angular.z*Chassis_Radius) * ChassisVel_Trans_MotorRPM(Wheel_Radius, 19);
    }
    else
    {
        wheel[0].wheel_vel = ( cmd_vel.linear.x + cmd_vel.angular.z*Chassis_Radius) * ChassisVel_Trans_MotorRPM(Wheel_Radius, 19);
        wheel[1].wheel_vel = (-cmd_vel.linear.y*SIN60 - cmd_vel.linear.x*COS60 + cmd_vel.angular.z*Chassis_Radius) * ChassisVel_Trans_MotorRPM(Wheel_Radius, 19);
        wheel[2].wheel_vel = ( cmd_vel.linear.y*COS30 - cmd_vel.linear.x*SIN30  + cmd_vel.angular.z*Chassis_Radius) * ChassisVel_Trans_MotorRPM(Wheel_Radius, 19);
    }
}



bool Omni_Chassis::Pid_Param_Init(int num, float Kp, float Ki, float Kd, float Integral_Max, float OUT_Max, float DeadZone)
{
    PID_Wheel[num].PID_Param_Init(Kp, Ki, Kd, OUT_Max, Integral_Max,DeadZone);
    return true;
}

bool Omni_Chassis::Pid_Mode_Init(int num, float LowPass_error, float LowPass_d_err, bool D_of_Current, bool Imcreatement_of_Out)
{
    PID_Wheel[num].PID_Mode_Init(LowPass_error, LowPass_d_err, D_of_Current, Imcreatement_of_Out);
    return true;
}


bool Omni_Chassis::Pid_Param_Init_Yaw(float Kp, float Ki, float Kd, float Integral_Max, float OUT_Max, float DeadZone)
{
	PID_Yaw.PID_Param_Init(Kp, Ki, Kd, OUT_Max, Integral_Max,DeadZone);
    return true;
}

bool Omni_Chassis::Pid_Mode_Init_Yaw(float LowPass_error, float LowPass_d_err, bool D_of_Current, bool Imcreatement_of_Out)
{
	PID_Yaw.PID_Mode_Init(LowPass_error, LowPass_d_err, D_of_Current, Imcreatement_of_Out);
    return true;
}

void Omni_Chassis::Yaw_Control(float target_yaw, Robot_Twist_t *twist)
{
//	if (target_yaw > -180.f && target_yaw <= 180.f)
//	{
	float current_yaw = RealPosData.world_yaw;
	
	// 计算角度误差
	float error = target_yaw - current_yaw;
	
	// 将误差归一化到-180度到180度之间
	while (error > 180.0f)
	{
		error -= 360.0f;
	}
	
	while (error < -180.0f)
	{
		error += 360.0f;
	}
	
	PID_Yaw.current = current_yaw;
	PID_Yaw.target = current_yaw + error;
	twist->angular.z = PID_Yaw.Adjust();
	//printf_DMA("%f,%f,%f\r\n", RawPosData.angle_Z, target_yaw, twist->angular.z);
//	}
}

void Omni_Chassis::World_Coordinate(float direction_yaw, Robot_Twist_t *twist)
{
	if (direction_yaw > -180.f && direction_yaw <= 180.f)
	{
		float current_yaw = RealPosData.world_yaw;
		
		float error = current_yaw - direction_yaw;
		
		// 将误差归一化到-180度到180度之间
        if (error > 180.0f)
		{
            error -= 360.0f;
		}
		
		if (error < -180.0f)
		{
            error += 360.0f;
        }
		
		error = error * PI / 180.f;//转换弧度制
		
		float COS_error = cosf(error);
		float SIN_error = sinf(error);
		
		// 保存原始值，避免计算干扰
		float vx = twist->linear.x;
		float vy = twist->linear.y;
		
		twist->linear.x  =  (vx * COS_error + vy * SIN_error);
        twist->linear.y  = -(vx * SIN_error - vy * COS_error);
	}
}



