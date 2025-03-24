#include "ShootFun.h"
#include "ShootTask.h"
#include "Chassis_Task.h"
#include "pid.h"
#include "DM4310.h"
#include "math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Gimbal_task.h"

/*声明变量or函数*/
void Shoot_Statemachine_2_Init(Shoot_t* shoot);
void Shoot_Statemachine_2_Run(Shoot_t* shoot);
void Shoot_Statemachine_2_Stop(Shoot_t* shoot);

extern RC_ctrl_t rc_ctrl;
extern Motor_struct motor;


void Shoot_flag_Init(Shoot_t* shoot)
{
	shoot->flag.shoot_finish_flag 		= 1;	
	shoot->flag.left_shoot_flag 			= 0;
	shoot->flag.wheel_shoot_flag 			= 0;	
	shoot->flag.shoot_lock_flag  			= 0;
	
	shoot->State 	= SHOOT_Init;
	shoot->Action = NORMAL;
}

/****************************************************************拨弹盘模式选择****************************************************************/
void Shoot_Mode_Choose(Shoot_t* shoot)
{
	if((rc_ctrl.rc.s[0] == 3 && rc_ctrl.rc.s[1] == 3)||(rc_ctrl.rc.s[0] == 2&&rc_ctrl.rc.s[1] == 3)) //仅在遥控模式下开启摩擦轮后可用
		Shoot_Mode_choose_remote(shoot);
	else if(rc_ctrl.rc.s[1] == 2)
		Shoot_Mode_choose_keyboard(shoot);
}

/*键鼠模式选择*/
void Shoot_Mode_choose_keyboard(Shoot_t* shoot)
{
	
	if((!IF_KEY_PRESSED_R)&&shoot->Action == BERSERK)  //狂暴模式取消后及时修改目标角度
		motor.Rammer.Apid.goal = motor.Rammer.Apid.now;	    
	
//	if(!Revolver_Heat_Limit())  //若卡弹or热量限制则直接跳过选择		
//	{
		if(Gimbal.Action == GIMBAL_NORMAL)				//手瞄
		{	//左键&&确保单发&&已完成上次打弹&&(非自瞄or自瞄且到位)
			if(IF_MOUSE_PRESSED_LEFT&&(!shoot->flag.left_shoot_flag)&&shoot->flag.shoot_finish_flag)
			{
				shoot->Action = SINGLE;	
			}
			else if(!IF_MOUSE_PRESSED_LEFT)
			{
				shoot->flag.left_shoot_flag = 0;
				shoot->Action = NORMAL;		
			}
		}
		else if(Gimbal.Action == GIMBAL_AUTO)			//自瞄
		{				
			shoot->flag.ATUO_time = xTaskGetTickCount();
		
			if((Gimbal.IMUData.find_bool == '1')&&IF_MOUSE_PRESSED_LEFT&&(!shoot->flag.left_shoot_flag)&&shoot->flag.shoot_finish_flag)
				shoot->Action = SINGLE;	
			else if(!IF_MOUSE_PRESSED_LEFT)
			{
				shoot->flag.left_shoot_flag = 0;
				shoot->Action = NORMAL;	
			}
		}
//	}
//	else
//	{
//		if(Revolver_Heat_Limit()) //热量限制
//			shoot->Action = NORMAL;
//		else											//卡弹保护
//			shoot->Action = STOP; //放松
//			shoot->Action = NORMAL;

//	}

	
//	if(IF_KEY_PRESSED_R)     //长按R开启狂暴模式(与前面的if分开,开启R则覆盖前面的状态)
//		shoot->flag.R_press_time++;
//	else
//		shoot->flag.R_press_time=0;
//	if(shoot->flag.R_press_time>250)
//		shoot->Action = BERSERK;		
}

/*遥控器模式选择*/
void Shoot_Mode_choose_remote(Shoot_t* shoot)
{
				if(rc_ctrl.rc.ch[4]>500&&shoot->flag.wheel_shoot_flag==0)
				{
					shoot->Action = SINGLE;
				}
				else if(rc_ctrl.rc.ch[4]<100&&rc_ctrl.rc.ch[4]>-100)
				{
					shoot->flag.wheel_shoot_flag = 0;
					shoot->Action = NORMAL;
				}
}


/****************************************************************拨弹盘计算****************************************************************/
void Shoot_clc(void)
{
	switch(Shoot.Action)
	{
		case NORMAL:
		{
			Shoor_Ctl_NORMAL(&Shoot);		//计算并保持当前位置				
			break;
		}
		case SINGLE:
		{
			Shoor_Ctl_SINGLE(&Shoot);		//单发
			break;
		}
		case BERSERK:
		{			
			Shoor_Ctl_BERSERK(&Shoot);	//一件超频
			break;
		}
		case STOP:
		{			
			Shoor_Ctl_STOP(&Shoot);			//本用于防卡弹，但疑似会因此出现双发（并不确定，感觉可能性不大但是保险起见暂时不做卡弹处理）
			break;
		}    
	}

}
void Shoor_Ctl_NORMAL(Shoot_t* shoot)
{
	while(motor.Rammer.Apid.goal < -PI)
	motor.Rammer.Apid.goal += 2*PI;
	while(motor.Rammer.Apid.goal > PI)
	motor.Rammer.Apid.goal -= 2*PI;
	DM_position_control(&motor.Rammer);
/********************************卡弹判断*************************/
	if(fabsf(motor.Rammer.Apid.goal - motor.Rammer.Apid.now)<0.035f) //1°以内
	{
		shoot->flag.shoot_finish_flag = 1;
		shoot->flag.shoot_finish_time = 0;
	}
	if(!shoot->flag.shoot_finish_flag)
	{
		shoot->flag.shoot_finish_time++;
		if(shoot->flag.shoot_finish_time>2500)   
		{	
			shoot->flag.shoot_lock_flag = 1;
			shoot->flag.shoot_finish_flag = 1;
			motor.Rammer.Apid.goal -= 2*1.04719f;
			while(motor.Rammer.Apid.goal < -PI)
			motor.Rammer.Apid.goal += 2*PI;
			while(motor.Rammer.Apid.goal > PI)
			motor.Rammer.Apid.goal -= 2*PI;
		}
}
/***************裁判系统发射机构断电后保持现位，防止上电发射************/
//	if(!Game_Robot_State.power_management_shooter_output)
//		shoot->flag.judge_on_flag = 0;
//	else if(Game_Robot_State.power_management_shooter_output == 1&&shoot->flag.judge_on_flag == 0)
//	{
//		shoot->flag.judge_on_flag = 1;
//		Motor_4310_Rammer.target_angle = Motor_4310_Rammer.angle;
//	}
}




void Shoor_Ctl_SINGLE(Shoot_t* shoot)
{
	if((rc_ctrl.rc.s[0] == 3&&rc_ctrl.rc.s[1] == 3)||(Gimbal.Action == GIMBAL_NORMAL&&(!IF_MOUSE_PRESSED_RIGH)))
	{
		if((!shoot->flag.left_shoot_flag)||(!shoot->flag.wheel_shoot_flag))  //未执行过
		{
			motor.Rammer.Apid.goal += 1.04719f;		
			shoot->flag.shoot_finish_flag = 0;		
			shoot->flag.left_shoot_flag  	= 1;
			shoot->flag.wheel_shoot_flag 	= 1;		
			shoot->Action = NORMAL;	
		}
	}
	else if((rc_ctrl.rc.s[0] == 2&&rc_ctrl.rc.s[1] == 3)||Gimbal.Action == GIMBAL_AUTO)                 //自瞄
	{
		if(((!shoot->flag.left_shoot_flag)||(!shoot->flag.wheel_shoot_flag))&&Gimbal.IMUData.find_bool == '1')  //未执行过
		{
			motor.Rammer.Apid.goal += 1.04719f;		
			shoot->flag.shoot_finish_flag = 0;		
			shoot->flag.left_shoot_flag  	= 1;
			shoot->flag.wheel_shoot_flag 	= 1;		
			shoot->Action = NORMAL;	
		}
	}
}

void Shoor_Ctl_BERSERK(Shoot_t* shoot)
{
	motor.Rammer.Vpid.goal = 4;
	DM_speed_control(&motor.Rammer);
}

void Shoor_Ctl_STOP(Shoot_t* shoot)
{
	motor.Rammer.Vpid.out = 0;
	shoot->flag.shoot_stop_time++;
	if(shoot->flag.shoot_stop_time>500) 		//放松1s
	{
		shoot->flag.shoot_stop_time = 0;	
		motor.Rammer.Apid.goal = motor.Rammer.Apid.now;
		shoot->flag.shoot_finish_time = 0;
		shoot->flag.shoot_lock_flag = 0;
		shoot->Action = NORMAL;
	}
}

/****************************************************************拨弹盘保护****************************************************************/

/*卡弹保护在Normal模式里*/


/**
  * @brief  枪管热量限制
  * @param  void
  * @retval 热量是否超限
  * @attention  超限要重置一下拨盘,根据剩余可发弹量来做闭环
  *             
  */
//bool Revolver_Heat_Limit(void)
//{
//	static bool Heat_Limit = 0; //是否触发热量限制
//	int16_t Q_max,Q_now,Q_left;
//	Q_max = Game_Robot_State.shooter_barrel_heat_limit;
//	Q_now = Power_Heat_Data.shooter_id1_42mm_cooling_heat;
//	Q_left = Q_max-Q_now;
//    if(Q_left>100)
//		{ 
//        Heat_Limit = 0;
//    }
//		else
//		{
//        Heat_Limit = 1;
//    }

//    if(IF_KEY_PRESSED_R)//无视热量 一键超频
//	{
//		Heat_Limit = 0;
//	}
//	
//	return Heat_Limit;	

//}

/****************************************************************状态机控制****************************************************************/

/* 状态机控制器 */
void ShootSelate(Shoot_t* shoot)
{
		if((rc_ctrl.rc.s[1] == 3&&rc_ctrl.rc.s[0] != 1)||rc_ctrl.rc.s[1] == 2)
			Shoot_Statemachine_2_Run(shoot);
		else if(rc_ctrl.rc.s[0] == 1 && rc_ctrl.rc.s[1] == 3)
			Shoot_Statemachine_2_Stop(shoot);
//		if(Chassis.flag.remotelose_flag)//遥控器失联保护
//			Shoot_Statemachine_2_Stop(shoot);

}


/* 把拨弹状态机切入初始化模式 */
void Shoot_Statemachine_2_Init(Shoot_t* shoot)
{
	if(shoot->State != SHOOT_Init)
	{
		//处理一下
//		Clean_Shoot_CtrlMsg(shoot);
		
		//切过去
		shoot->State = SHOOT_Init;
	}
}


/* 把拨弹盘状态机切入Run模式 */
void Shoot_Statemachine_2_Run(Shoot_t* shoot)
{
	if(shoot->State != SHOOT_Run)
	{
		//处理一下，比如清空pid
//		Clean_Shoot_CtrlMsg(shoot);
		motor.Rammer.Apid.goal = motor.Rammer.Apid.now;
		//切过去
		shoot->State = SHOOT_Run;
	}
}

/* 把拨弹盘状态机切入Stop模式 */
void Shoot_Statemachine_2_Stop(Shoot_t* shoot)
{
	if(shoot->State != SHOOT_STOP)
	{
		//处理一下，比如清空pid
//		Clean_Shoot_CtrlMsg(shoot);
		
		//切过去
		shoot->State = SHOOT_STOP;
	}
}

