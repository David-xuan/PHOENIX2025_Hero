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
    CHASS_NORMAL        = 0x01,  //底盘不跟随云台行走	
    CHASS_FOLLOW 		= 0x02,	//底盘跟随云盘行走
    CHASS_GYROSCOPE     = 0x03  //小陀螺模式
}Chassis_Action_t;

typedef struct
{
	bool 						remotelose_flag; //遥控器失联标志。 missingtime在定时器中断累加，在接收遥控器数据时清0，如果失联超过时间，在Chassis任务中把失联标志置1or置0
	Gimbal_Follow		gimbal_follow;   //判断当前跟随方向是头还是屁股
	
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
