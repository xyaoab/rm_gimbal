#include "main.h"
#include "function_list.h"

static u32 ticks_msimg = (u32)-1;
//////
CanRxMsg * msg;
s32 target_angle=0;
int32_t max=0;
float w=60;
ControlMotor motor[4];
///////

void init(){
	SysTick_Init();  
	Dbus_init();//usart1
	//buzzer_init();	 //initialization of buzzer
	tft_init(2,WHITE,BLACK,BLACK);
	LED_master_init();
	gyro_init();
	ADC1_init();
	judging_system_init(); //usart3
	gyro_init();
	pneumatic_init();
	CAN1_Configuration();
	CAN2_Configuration();
	gyro_init();
	gyro_cal();
	Quad_Encoder_Configuration();
	Encoder_Start1();
	Encoder_Start2();
	Friction_wheel_init();
	TIM5_Int_Init(24,13124);// 256hz //3.9xx ms for gyro usage
	
}

int main(void)
{	
	init();
	//buzzer_play_song(START_UP, 125, 0);
	s32 target_gimbal_yaw=GMYawEncoder.ecd_angle;
	static gimbal_PID_Controller yaw_position={3,0.01,0,40,0,0,0,0};
	static gimbal_PID_Controller yaw_speed={140,0.5,0,30000,0,0,0,0};
	
	
	while (1)  {	

		if(ticks_msimg != get_ms_ticks()) 
		{
			ticks_msimg = get_ms_ticks();  //maximum 1000000	
			//buzzer_check();	
		//gimbal_setyaw(600);
			gimbal_pid( &yaw_position, target_gimbal_yaw,GMYawEncoder.ecd_angle);
	
			low_pass_filter(GMYawEncoder.filter_rate);
			
			gimbal_pid( &yaw_speed,yaw_position.output,lpf_current);
			
			if(DBUS_ReceiveData.rc.switch_left==2)
			{
				Set_Gimbal_Current(CAN2,-yaw_speed.output,0,0);
			}
			else if(DBUS_ReceiveData.rc.switch_left==1)
			{
				Set_Gimbal_Current(CAN2,0,0,0);
			}
			
			if(ticks_msimg%3==0){
			//yaw_position.output=DBUS_ReceiveData.rc.ch2/30;
				target_gimbal_yaw+=(DBUS_ReceiveData.rc.ch2/60); //position target 
			}
			if(ticks_msimg%50==0)
			{	
		
				
				
				tft_clear();
				
				tft_prints(1,2,"gimbal%d",GMYawEncoder.filter_rate);
				tft_prints(1,3,"setpos%f",target_gimbal_yaw);
				tft_prints(1,4,"pid%d",yaw_speed.output);
				tft_prints(1,5,"angle%f",GMYawEncoder.ecd_angle);
				tft_prints(1,6,"setspeed%d",yaw_position.output);
			//		tft_prints(1,7,"Kp%f", yaw_position.Kp);
				
				
				tft_update();
				LED_blink(LED1);
			}			
			
		}
	}	
}	

	



