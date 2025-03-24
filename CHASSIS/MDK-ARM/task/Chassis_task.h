#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "stm32f4xx.h"
#include "remote_control.h"
#include "bsp_can.h"
#include "pid.h"
#include "motion.h"
#include "cmsis_os.h"
#include "can.h"

typedef enum
{
  GIMBAL_HEAD = 0x00,
	GIMBAL_TAIL = 0x01
} Gimbal_Follow;

typedef enum
{
	CHASS_Init     = 0x00,
	CHASS_Stop     = 0x01,
	CHASS_Remote   = 0x02,
	CHASS_Keyboard = 0x03
}ChassState;

typedef enum
{
    CHASS_NORMAL        = 0x01,  //���̲�������̨����	
    CHASS_FOLLOW 		= 0x02,	//���̸�����������
    CHASS_GYROSCOPE     = 0x03  //С����ģʽ
}Chassis_Action_t;

typedef struct
{
	bool 						remotelose_flag; //ң����ʧ����־�� missingtime�ڶ�ʱ���ж��ۼӣ��ڽ���ң��������ʱ��0�����ʧ������ʱ�䣬��Chassis�����а�ʧ����־��1or��0
	Gimbal_Follow		gimbal_follow;   //�жϵ�ǰ���淽����ͷ����ƨ��
	
}CHASS_flag_t;

typedef struct
{
	ChassState        State;
	Chassis_Action_t  Action;
	CHASS_flag_t      flag;
	struct
	{
		float MotorCurr[4];
	}CtrlMsg;
	
	
}chass_t;

extern chass_t Chassis;

void Chassis_task(void const * argument);


#endif
