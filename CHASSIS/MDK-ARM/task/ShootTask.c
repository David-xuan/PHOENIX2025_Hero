#include "cmsis_os.h"
#include "ShootTask.h"
#include "ShootFun.h"
#include "DM4310.h"
#include "can.h"
#include "pid.h"

Shoot_t Shoot;
static int enable_flag = 0;
static int target_flag = 0;
//extern Motor_struct motor;

 void Shoot_task(void const * argument)
{
//  while(Motor_4310_Rammer.tempure == 0)
//  {
//	  Motor_DM4310_Enable(&hcan1,DM4310_RAMMER_ID);
//	  osDelay(1);
//  }
  Shoot_flag_Init(&Shoot);
  Rammer_PID_Init(&motor.Rammer);
  for(;;)
  {
	  	while(Motor_4310_Rammer.dm_err == 0)
		{
			Motor_DM4310_Enable(&hcan1,DM4310_RAMMER_ID);
			motor.Rammer.Vpid.i = 0;
			motor.Rammer.Vpid.p = 0;
			motor.Rammer.Vpid.d = 0;
			motor.Rammer.Apid.d = 0;
			motor.Rammer.Apid.i = 0;
			motor.Rammer.Apid.p = 0;
			target_flag = 1;
			
		}
		if((Motor_4310_Rammer.dm_err!=0)&&target_flag)
		{
			motor.Rammer.Apid.goal = motor.Rammer.Apid.now;
			target_flag = 0;
		}
		ShootSelate(&Shoot);//状态控制(停止/运行)
		switch(Shoot.State)
		{
			case SHOOT_Init:
			{
				Shoot_Statemachine_2_Stop(&Shoot);
				break;			
			}
			case SHOOT_Run:
			{
				Shoot_Mode_Choose(&Shoot);//发射操控模式选择
				Shoot_clc();//根据不同发射模式计算target_torque
				break;
			}
		  case SHOOT_STOP:
			{
//				Clean_Shoot_CtrlMsg(&Shoot);
				break;
			}
		}
		Motor_DM4310_send(&hcan1,DM4310_RAMMER_ID, 0, 0, 0, 0, motor.Rammer.Vpid.out);//motor.Rammer.Vpid.out
//		Send_cap_msg();
		osDelay(2);
  }
}


