#include "Gimbal_task.h"
#include "DM4310multi.h"
#include "bsp_can.h"
#include "cmsis_os.h"
#include "DM4310.h"
#include "vision.h"
#include "tim.h"
#include "HWT606.h"
#include "GimbalFun.h"

extern Direc_Struct direc;
extern Motor_struct motor;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern RC_ctrl_t rc_ctrl;
float angle1 = 0,speed1 = 10,k_pitch = 0.000001;
Gimbal_t Gimbal;
void Gimbal_task(void const *argument)
{
	portTickType currentTime;
	motor.Gimbal_Pitch.Apid.goal = motor.Gimbal_Pitch.Apid.now;
	Gimbal.error = motor.Gimbal_Pitch.Apid.now;
		
	for(;;)
	{
		while(DM4310.dm_err == 0)
			Motor_DM4310_Enable(&hcan1,0x101);
		currentTime = xTaskGetTickCount(); //当前系统时间
		GimbalSelate(&Gimbal);
		if(Gimbal.State == GIMB_Keyboard)
		{
			Gimbal_Mode_Choose(&Gimbal);
			KeyboardControlGimbal(&Gimbal);
		}
		else if(rc_ctrl.rc.s[0]==3)
		{
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1500);
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 2100);
			direc.pitch = k_pitch*rc_ctrl.rc.ch[1];
			motor.Gimbal_Pitch.Apid.goal += direc.pitch;
		}
		else if(rc_ctrl.rc.s[0]==2)//自瞄
		{
			motor.Gimbal_Pitch.Apid.goal = -VisionValue.pitch_value.value;
		}
		position_speed_control(&hcan1, 0x101, motor.Gimbal_Pitch.Apid.goal, speed1);
		vTaskDelayUntil(&currentTime, 1); //绝对延时
	}
}
