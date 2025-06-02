#include "launcher.h"
#include "chassis_task.h"

#define ABS(x)  ((x) >= 0 ? (x) : -(x))


bool Launcher::Reset()
{
    static int start_time=get_systemTick()/1000;
    if(get_systemTick()/1000 - start_time > 1000)
    {
        LauncherMotor[0].encoder_offset = LauncherMotor[0].get_encoder();
        LauncherMotor[1].encoder_offset = LauncherMotor[1].get_encoder();
		LauncherMotor[2].encoder_offset = LauncherMotor[2].get_encoder();
        machine_init_ = true;
    }
    else
    {
        LauncherMotor[0].Out = -30;
        LauncherMotor[1].Out = 20;
		LauncherMotor[2].Out = 0;
        machine_init_ = false;
    }
}

void Launcher::LaunchMotorCtrl()
{
    Motor_SendMsgs(&hcan1,LauncherMotor);
    static int send_flag=0;
    if(send_flag<1)
    {
        Motor_SendMsgs(&hcan2, FrictionMotor[0]);
    }
    else if (send_flag>=1&&send_flag<2)
    {
        Motor_SendMsgs(&hcan2, FrictionMotor[2]);
    }
    else if (send_flag>=2&&send_flag<3)
    {
        Motor_SendMsgs(&hcan2, FrictionMotor[1]);
    }
    else
    {
        send_flag = -1;
    }

    send_flag++;
}

void Launcher::PitchControl(float pitch_angle)
{
    if(!machine_init_)
    {
        Reset();
        PidPitchPos.PID_Mode_Init(0.1,0.1,true,false);
        PidPitchPos.PID_Param_Init(15, 0, 0.2, 100, /*300*/900, 0.2);
    }
    else
    {
        //判断俯仰角度是否在范围内
        if(pitch_angle > pitch_angle_max_)
            pitch_angle = pitch_angle_max_;
        else if(pitch_angle < 0)
            pitch_angle = 0;
        else{;}

        PidPitchPos.target = pitch_angle;
        PidPitchPos.current = LauncherMotor[0].get_angle();
        PidPitchSpd.target = PidPitchPos.Adjust();
        PidPitchSpd.current = LauncherMotor[0].get_speed();
        LauncherMotor[0].Out = PidPitchSpd.Adjust();
    }
}

void Launcher::FrictionControl(bool friction_ready, float shoot_speed)
{
  if(machine_init_)
	{
        if(friction_ready)
        {
            FrictionMotor[0].Out = shoot_speed;
            FrictionMotor[1].Out = -shoot_speed;
            FrictionMotor[2].Out = -shoot_speed;
        }
        else
        {
            FrictionMotor[0].Out = 0;
            FrictionMotor[1].Out = 0;
            FrictionMotor[2].Out = 0;
        }
    }
}

void Launcher::SpinControl(bool spin_state)
{
	if(!machine_init_)
    {
        Reset();
        PidSpinPos.PID_Mode_Init(0.1,0.1,true,false);
        PidSpinPos.PID_Param_Init(15, 0, 0.2, 100, /*300*/900, 0.1);
    }
    else
    {
		if(spin_state)
		{
			if (LauncherMotor[1].get_angle() < (spin_angle_max_ - 30))
			{
				PidSpinSpd.target = SpinPlanner.Plan(0,spin_angle_max_,LauncherMotor[1].get_angle());
				PidSpinSpd.current = LauncherMotor[1].get_speed();
				LauncherMotor[1].Out = PidSpinSpd.Adjust();
			}
			else
			{
				PidSpinPos.target = spin_angle_max_;
				PidSpinPos.current = LauncherMotor[1].get_angle();
				PidSpinSpd.target = PidSpinPos.Adjust();
				PidSpinSpd.current = LauncherMotor[1].get_speed();
				LauncherMotor[1].Out = PidSpinSpd.Adjust();
			}
		}
		else
		{
			if (LauncherMotor[1].get_angle() > 30)
			{
				PidSpinSpd.target = SpinPlanner.Plan(spin_angle_max_,0,LauncherMotor[1].get_angle());
				PidSpinSpd.current = LauncherMotor[1].get_speed();
				LauncherMotor[1].Out = PidSpinSpd.Adjust();
			}
			else
			{
				PidSpinPos.target = 0;
				PidSpinPos.current = LauncherMotor[1].get_angle();
				PidSpinSpd.target = PidSpinPos.Adjust();
				PidSpinSpd.current = LauncherMotor[1].get_speed();
				LauncherMotor[1].Out = PidSpinSpd.Adjust();
			}
		}
	}
}

void Launcher::PushControl(bool push_state)
{
	if(push_state)
	{
		PidPushSpd.target = 6000;
		PidPushSpd.current = LauncherMotor[2].get_speed();
		LauncherMotor[2].Out = PidPushSpd.Adjust();
	}
	else
	{
		PidPushSpd.target = -5000;
		PidPushSpd.current = LauncherMotor[2].get_speed();
		LauncherMotor[2].Out = PidPushSpd.Adjust();
	}
}

#define PUSH_TIME_1 70000

void Launcher::PushBall(enum CONTROL_E state)
{
	static uint32_t start_time;
	static uint8_t flag = 0;
	static uint8_t time_flag = 0;
	
	if ((flag == 0) && (state == SHOOT_OFF))
	{
		LauncherMotor[2].Out = -10;
	}
	if (state == SHOOT_ON)
	{
		flag = 1;
	}
	if (flag == 1)
	{
		PushControl(true);
		if ((ABS(LauncherMotor[2].Out) >= 3000) && (time_flag == 0))
		{
			start_time = Get_SystemTimer();
			time_flag = 1;
		}
		if (((Get_SystemTimer() - start_time) >=  PUSH_TIME_1) && (ABS(LauncherMotor[2].Out) >= 2700) && (time_flag == 1))
		{
			flag = 2;
			time_flag = 0;
		}
	}
	if (flag == 2)
	{
		PushControl(false);
		if ((ABS(LauncherMotor[2].Out) >= 2300) && (time_flag == 0))
		{
			start_time = Get_SystemTimer();
			time_flag = 1;
		}
		if (((Get_SystemTimer() - start_time) >=  PUSH_TIME_1) && (ABS(LauncherMotor[2].Out) >= 2700) && (time_flag == 1))
		{
			flag = 0;
			time_flag = 0;
		}
	}
}
















