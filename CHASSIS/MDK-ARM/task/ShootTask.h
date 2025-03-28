#ifndef __SHOOTTASK_H__
#define __SHOOTTASK_H__

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

typedef enum
{
		SHOOT_Init = 0x00,
		SHOOT_Run  = 0x01,
		SHOOT_STOP = 0x02
}Shoot_State_t;

typedef enum
{
		NORMAL    = 0x00,
		SINGLE 		= 0x01,
		BERSERK	  = 0x02,  //连发
		STOP      = 0x03   //卡弹or裁判系统下电
}Shoot_Action_t;

typedef struct
{
	bool      wheel_shoot_flag; 		//是否执行，确保单发
	bool      left_shoot_flag;      //同上
	bool      shoot_finish_flag;   //打弹是否完成
	bool      shoot_lock_flag;     //是否卡弹
	bool	    judge_on_flag;			//裁判系统上下电
	uint16_t  shoot_finish_time;   //判断是否完成打弹，若超时则视为卡弹
    uint16_t  shoot_stop_time;     //放松时间
	uint16_t  R_press_time;
	portTickType ATUO_time;
	portTickType ATUO_time_last;

}Shoot_flag_t;

typedef struct
{
	Shoot_State_t 	State;	
	Shoot_Action_t  Action;
	Shoot_flag_t 		flag;
}Shoot_t;


extern Shoot_t Shoot;

#endif


