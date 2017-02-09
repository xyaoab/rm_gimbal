#include "gimbal.h"
#include "stm32f4xx.h"
#include "Dbus.h"
float lpf_a = 0.85;

float gimbal_yaw_target;


void gimbal_setyaw( float setpoint)
{
	
	gimbal_yaw_target=setpoint/40;
	
}
void gimbal_pid(gimbal_PID_Controller *pid, float target,float current)
{
	float error = target-current;
	
	float Kout = error * pid->Kp;
	
	pid->gimbal_temp_integral += error;
	
	float Iout = pid->gimbal_temp_integral * pid->Ki;
	
	pid->gimbal_temp_derivative = error - pid->gimbal_pre_error;
	
	float Dout = pid->gimbal_temp_derivative * pid->Kd;
	
	pid->output= Iout + Dout + Kout;
	
	if(pid->output>pid->MAX_Integral)
	{
		pid->output=pid->MAX_Integral;
	}
	if(pid->output<-pid->MAX_Integral)
	{
		pid->output=-pid->MAX_Integral;
	}
}

float lpf_current=0;

void low_pass_filter(float current)
{
  lpf_current = current * lpf_a +  lpf_current * (1 - lpf_a);
}

