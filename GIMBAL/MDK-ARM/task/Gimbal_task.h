#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "stm32f4xx.h"

void Gimbal_task(void const * argument);

typedef enum
{
	GIMB_Init     = 0x00,
	GIMB_Stop     = 0x01,
	GIMB_Remote   = 0x02,
	GIMB_Keyboard = 0x03
}GIMBALState;

typedef enum  
{
	
   GIMBAL_NORMAL = 0x00,  //����
	 GIMBAL_AUTO   = 0x01,  //����
}Gimbal_action_t;

typedef struct 
{
	float ol[3];     //�ֱ���Yaw��Pitch��Roll
	float accel[3];
	float quat[4];
	float gyro[3];
	float ol_accel[3];
	float temp;
	float pitch_target;
	uint8_t pitch_ok;     //yaw�Ƿ񵽴�����ָ��λ��(�����ж�)
}IMU_t;
//��̨��Ϣ
typedef struct GimbalDataType
{
	GIMBALState       State;
//	Gimbal_flag_t	    flag;
	IMU_t							IMUData;
//	Hanging_Mode_t		Hanging_Mode;
	Gimbal_action_t   Action;
	uint8_t           Robot_color;
	float             yaw_angle;
	int               pwm[2];
	float             error;
}Gimbal_t;

extern Gimbal_t Gimbal;

#endif
