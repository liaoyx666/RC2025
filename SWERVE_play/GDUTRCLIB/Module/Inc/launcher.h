#ifndef __LAUNCHER_H
#define __LAUNCHER_H


#include "motor.h"
#include "pid.h"
#include "speed_plan.h"
//#include "chassis_task.h"
//#include "main.h"
#include "drive_uart.h"







#ifdef __cplusplus
extern "C" {
#endif 

#ifdef __cplusplus
}



class Launcher : public PidTimer
{
public:
    Launcher(float pitch_angle_max, float spin_angle_max, float push_angle_max)   //最大俯仰角度和推杆最大行程
    {
        pitch_angle_max_ = pitch_angle_max;
        spin_angle_max_ = spin_angle_max;
		push_angle_max_ = push_angle_max;//
		
		

        FrictionMotor[0].Mode = SET_eRPM;
        FrictionMotor[1].Mode = SET_eRPM;
        FrictionMotor[2].Mode = SET_eRPM;
        FrictionMotor[0].Out = 0;
		FrictionMotor[1].Out = 0;
		FrictionMotor[2].Out = 0;
    }

    Motor_C620 LauncherMotor[3] = {Motor_C620(5), Motor_C620(6), Motor_C620(7)};
    
    VESC FrictionMotor[3] = {VESC(101), VESC(102), VESC(103)};

    void PitchControl(float pitch_angle);
    void FrictionControl(bool friction_ready, float shoot_speed);
	//
	void SpinControl(bool spin_state);
	
	void PushControl(bool push_state);
	void PushBall(struct CONTROL_T *ctrl);
	
	void LoadBall(enum CONTROL_E state, float *pitch_angle, bool *spin_state, float *shoot_speed);
	
	//
    bool Pid_Param_Init(int num, float Kp, float Ki, float Kd, float Integral_Max, float OUT_Max, float DeadZone)
    {
        switch (num)
        {
            case 0:
                PidPitchSpd.PID_Param_Init(Kp,Ki,Kd,Integral_Max,OUT_Max,DeadZone);
                break;
            
            case 1:
                PidSpinSpd.PID_Param_Init(Kp,Ki,Kd,Integral_Max,OUT_Max,DeadZone);
				break;
			
			case 2:
                PidPushSpd.PID_Param_Init(Kp,Ki,Kd,Integral_Max,OUT_Max,DeadZone);
				break;
			
            default:
                break;
        }
    }

    bool Pid_Mode_Init(int num, float LowPass_error, float LowPass_d_err, bool D_of_Current, bool Imcreatement_of_Out)
    {
        switch (num)
        {
            case 0:
                PidPitchSpd.PID_Mode_Init(LowPass_error,LowPass_d_err,D_of_Current,Imcreatement_of_Out);
                break;
            
            case 1:
                PidSpinSpd.PID_Mode_Init(LowPass_error,LowPass_d_err,D_of_Current,Imcreatement_of_Out);
				break;
			
			case 2:
                PidPushSpd.PID_Mode_Init(LowPass_error,LowPass_d_err,D_of_Current,Imcreatement_of_Out);
				break;
			
            default:
                break;
        }
    }

    void LaunchMotorCtrl();
private:
    float pitch_angle_max_ = 0.0f, spin_angle_max_ = 0.0f, push_angle_max_ = 0.0f;
    PID PidPitchSpd, PidPitchPos, PidSpinSpd, PidSpinPos, PidPushSpd;
    TrapePlanner SpinPlanner = TrapePlanner(0.15,0.15,1500,100,1);
	//TrapePlanner PushPlanner = TrapePlanner(0.15,0.15,3000,100,1);
    bool machine_init_ = false;
    bool Reset();
    float pitch_angle_last_=0;
    bool target_change=false;
};




#endif

#endif 
// LAUNCHER_H
