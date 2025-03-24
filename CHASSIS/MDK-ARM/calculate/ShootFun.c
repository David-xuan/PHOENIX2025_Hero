#include "ShootFun.h"
#include "ShootTask.h"
#include "Chassis_Task.h"
#include "pid.h"
#include "DM4310.h"
#include "math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Gimbal_task.h"

/*��������or����*/
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

/****************************************************************������ģʽѡ��****************************************************************/
void Shoot_Mode_Choose(Shoot_t* shoot)
{
	if((rc_ctrl.rc.s[0] == 3 && rc_ctrl.rc.s[1] == 3)||(rc_ctrl.rc.s[0] == 2&&rc_ctrl.rc.s[1] == 3)) //����ң��ģʽ�¿���Ħ���ֺ����
		Shoot_Mode_choose_remote(shoot);
	else if(rc_ctrl.rc.s[1] == 2)
		Shoot_Mode_choose_keyboard(shoot);
}

/*����ģʽѡ��*/
void Shoot_Mode_choose_keyboard(Shoot_t* shoot)
{
	
	if((!IF_KEY_PRESSED_R)&&shoot->Action == BERSERK)  //��ģʽȡ����ʱ�޸�Ŀ��Ƕ�
		motor.Rammer.Apid.goal = motor.Rammer.Apid.now;	    
	
//	if(!Revolver_Heat_Limit())  //������or����������ֱ������ѡ��		
//	{
		if(Gimbal.Action == GIMBAL_NORMAL)				//����
		{	//���&&ȷ������&&������ϴδ�&&(������or�����ҵ�λ)
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
		else if(Gimbal.Action == GIMBAL_AUTO)			//����
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
//		if(Revolver_Heat_Limit()) //��������
//			shoot->Action = NORMAL;
//		else											//��������
//			shoot->Action = STOP; //����
//			shoot->Action = NORMAL;

//	}

	
//	if(IF_KEY_PRESSED_R)     //����R������ģʽ(��ǰ���if�ֿ�,����R�򸲸�ǰ���״̬)
//		shoot->flag.R_press_time++;
//	else
//		shoot->flag.R_press_time=0;
//	if(shoot->flag.R_press_time>250)
//		shoot->Action = BERSERK;		
}

/*ң����ģʽѡ��*/
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


/****************************************************************�����̼���****************************************************************/
void Shoot_clc(void)
{
	switch(Shoot.Action)
	{
		case NORMAL:
		{
			Shoor_Ctl_NORMAL(&Shoot);		//���㲢���ֵ�ǰλ��				
			break;
		}
		case SINGLE:
		{
			Shoor_Ctl_SINGLE(&Shoot);		//����
			break;
		}
		case BERSERK:
		{			
			Shoor_Ctl_BERSERK(&Shoot);	//һ����Ƶ
			break;
		}
		case STOP:
		{			
			Shoor_Ctl_STOP(&Shoot);			//�����ڷ������������ƻ���˳���˫��������ȷ�����о������Բ����Ǳ��������ʱ������������
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
/********************************�����ж�*************************/
	if(fabsf(motor.Rammer.Apid.goal - motor.Rammer.Apid.now)<0.035f) //1������
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
/***************����ϵͳ��������ϵ�󱣳���λ����ֹ�ϵ緢��************/
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
		if((!shoot->flag.left_shoot_flag)||(!shoot->flag.wheel_shoot_flag))  //δִ�й�
		{
			motor.Rammer.Apid.goal += 1.04719f;		
			shoot->flag.shoot_finish_flag = 0;		
			shoot->flag.left_shoot_flag  	= 1;
			shoot->flag.wheel_shoot_flag 	= 1;		
			shoot->Action = NORMAL;	
		}
	}
	else if((rc_ctrl.rc.s[0] == 2&&rc_ctrl.rc.s[1] == 3)||Gimbal.Action == GIMBAL_AUTO)                 //����
	{
		if(((!shoot->flag.left_shoot_flag)||(!shoot->flag.wheel_shoot_flag))&&Gimbal.IMUData.find_bool == '1')  //δִ�й�
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
	if(shoot->flag.shoot_stop_time>500) 		//����1s
	{
		shoot->flag.shoot_stop_time = 0;	
		motor.Rammer.Apid.goal = motor.Rammer.Apid.now;
		shoot->flag.shoot_finish_time = 0;
		shoot->flag.shoot_lock_flag = 0;
		shoot->Action = NORMAL;
	}
}

/****************************************************************�����̱���****************************************************************/

/*����������Normalģʽ��*/


/**
  * @brief  ǹ����������
  * @param  void
  * @retval �����Ƿ���
  * @attention  ����Ҫ����һ�²���,����ʣ��ɷ����������ջ�
  *             
  */
//bool Revolver_Heat_Limit(void)
//{
//	static bool Heat_Limit = 0; //�Ƿ񴥷���������
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

//    if(IF_KEY_PRESSED_R)//�������� һ����Ƶ
//	{
//		Heat_Limit = 0;
//	}
//	
//	return Heat_Limit;	

//}

/****************************************************************״̬������****************************************************************/

/* ״̬�������� */
void ShootSelate(Shoot_t* shoot)
{
		if((rc_ctrl.rc.s[1] == 3&&rc_ctrl.rc.s[0] != 1)||rc_ctrl.rc.s[1] == 2)
			Shoot_Statemachine_2_Run(shoot);
		else if(rc_ctrl.rc.s[0] == 1 && rc_ctrl.rc.s[1] == 3)
			Shoot_Statemachine_2_Stop(shoot);
//		if(Chassis.flag.remotelose_flag)//ң����ʧ������
//			Shoot_Statemachine_2_Stop(shoot);

}


/* �Ѳ���״̬�������ʼ��ģʽ */
void Shoot_Statemachine_2_Init(Shoot_t* shoot)
{
	if(shoot->State != SHOOT_Init)
	{
		//����һ��
//		Clean_Shoot_CtrlMsg(shoot);
		
		//�й�ȥ
		shoot->State = SHOOT_Init;
	}
}


/* �Ѳ�����״̬������Runģʽ */
void Shoot_Statemachine_2_Run(Shoot_t* shoot)
{
	if(shoot->State != SHOOT_Run)
	{
		//����һ�£��������pid
//		Clean_Shoot_CtrlMsg(shoot);
		motor.Rammer.Apid.goal = motor.Rammer.Apid.now;
		//�й�ȥ
		shoot->State = SHOOT_Run;
	}
}

/* �Ѳ�����״̬������Stopģʽ */
void Shoot_Statemachine_2_Stop(Shoot_t* shoot)
{
	if(shoot->State != SHOOT_STOP)
	{
		//����һ�£��������pid
//		Clean_Shoot_CtrlMsg(shoot);
		
		//�й�ȥ
		shoot->State = SHOOT_STOP;
	}
}

