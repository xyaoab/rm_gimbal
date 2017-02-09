#include "stm32f4xx.h"

extern float gimbal_yaw_target;

extern float lpf_current;

typedef struct{
	float Kp, Ki, Kd;
	float MAX_Integral;
	float gimbal_temp_integral;
	float gimbal_temp_derivative;
	float gimbal_pre_error;
	int16_t output;
}gimbal_PID_Controller;

void gimbal_setyaw( float setpoint);
void gimbal_pid(gimbal_PID_Controller *pid, float target,float current);
void low_pass_filter(float current);