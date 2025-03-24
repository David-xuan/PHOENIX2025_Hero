#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "stm32f4xx.h"
#include "main.h"

typedef enum
{
	GIMB_Init     = 0x00,
	GIMB_Stop     = 0x01,
	GIMB_Remote   = 0x02,
	GIMB_Keyboard = 0x03
}GIMBALState;

typedef enum  
{
   GIMBAL_NORMAL 	= 0x00,  //����ģʽ
	 GIMBAL_AUTO    = 0x01,  //����
}Gimbal_action_t;

typedef struct Gimbal_flag_t
{
    bool     Move_flag;   		//�Ƿ��ƶ�������������Ʈ�ж�
	bool     Fist_encoder; 		//��һ�ν��������ģʽ
	bool     ControlMode;      //�ж��Ǳ�����ģʽor������ģʽ 0
	int      imu_flag;
	uint32_t boardlose_time;//���ͨ��ʧ���ж�
}Gimbal_flag_t;

typedef struct IMUType
{
	float    IMU_Yaw_angle;
	float    yaw_target;
	float    Vision_yaw_target; //����Ŀ��ֵ
	char    find_bool; //�����Ƿ��ҵ�Ŀ��
	union{
		uint8_t buff[4];
		float  distance;
	}distance_t;//װ�װ���룻
	uint8_t	   pitch_ok;//���鵽λ
	uint8_t		 yaw_ok;
	uint32_t Rx_add;
	uint32_t Rx_add_2;	
	uint16_t Rx_Frequency;
	uint16_t Rx_Frequency_2;	
}IMU_t;

//��̨��Ϣ
typedef struct GimbalDataType
{
	GIMBALState       State;
	Gimbal_flag_t	    flag;
	IMU_t							IMUData;
//	Hanging_Mode_t		Hanging_Mode;
	Gimbal_action_t   Action;
}Gimbal_t;

extern Gimbal_t Gimbal;

void Gimbal_task(void const * argument);


#endif
