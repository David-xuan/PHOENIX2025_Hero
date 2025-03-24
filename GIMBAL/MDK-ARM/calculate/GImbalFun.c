#include "GimbalFun.h"
#include "pid.h"
#include "can.h"
#include "DM4310.h"
#include "INS_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "vision.h"
#include "bsp_can.h"
#include "math.h"
#include "tim.h"
extern Motor_struct motor;

/**
  * @brief  б�º���,ʹĿ�����ֵ������������ֵ
  * @param  �����������,��ǰ���,�仯�ٶ�(Խ��Խ��)
  * @retval ��ǰ���
  * @attention
  */
float RAMP_float( float final, float now, float ramp )
{
    float buffer = 0;


    buffer = final - now;

    if (buffer > 0)
    {
        if (buffer > ramp)
        {
            now += ramp;
        }
        else
        {
            now += buffer;
        }
    }
    else
    {
        if (buffer < -ramp)
        {
            now += -ramp;
        }
        else
        {
            now += buffer;
        }
    }

    return now;
}

/**
  * @brief  б�º���,ʹĿ�����ֵ��������ָ������ֵ
  * @param  Ҫ�ڵ�ǰ��������ۼӵ�ֵ,Ŀ�������,��������
  * @retval Ŀ�������
  * @attention
  *
*/
float RampInc_float( float *buffer, float now, float ramp )
{

    if (*buffer > 0)
    {
        if (*buffer > ramp)
        {
            now     += ramp;
            *buffer -= ramp;
        }
        else
        {
            now     += *buffer;
            *buffer  = 0;
        }
    }
    else
    {
        if (*buffer < -ramp)
        {
            now     += -ramp;
            *buffer -= -ramp;
        }
        else
        {
            now     += *buffer;
            *buffer  = 0;
        }
    }

    return now;
}
int test_pwm = 545;
int test_pwm_2 = 2100;
int test_pwm_3 = 2360;
int test_pwm_4 = 1500;
static void servo_ctrl(Gimbal_t* gimbal)
{
	if(!IF_KEY_PRESSED_R)
		rc_ctrl.flag.R = 0;
	if(IF_KEY_PRESSED_R && rc_ctrl.flag.R == 0) 
	{
		rc_ctrl.flag.R = 1;
		if(gimbal->pwm[0] == test_pwm_4)
			gimbal->pwm[0] = test_pwm;
		else
			gimbal->pwm[0] = test_pwm_4;
	}
	if(!IF_KEY_PRESSED_C)
		rc_ctrl.flag.C = 0;
	if(IF_KEY_PRESSED_C && rc_ctrl.flag.C == 0) 
	{
		rc_ctrl.flag.C = 1;
		if(gimbal->pwm[1] == test_pwm_2)
			gimbal->pwm[1] = test_pwm_3;
		else
			gimbal->pwm[1] = test_pwm_2;
	}

	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, gimbal->pwm[0]);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, gimbal->pwm[1]);
}

//void Gimbal_flag_Init(Gimbal_t* gimbal)
//{

//	gimbal->flag.remotelose_flag = 1;

//	gimbal->State = GIMB_Init;
//	gimbal->Action = GIMBAL_NORMAL;
//	gimbal->Hanging_Mode = level ; //VB����ģʽ ˮƽ	
//}

/*����ѡ����̨ģʽ*/
void Gimbal_Mode_Choose(Gimbal_t* gimbal)
{
		//�Ҽ���ס����
    if(IF_MOUSE_PRESSED_RIGH)
			gimbal->Action  = GIMBAL_AUTO;
		else
			gimbal->Action = GIMBAL_NORMAL;

}

/* Keyboard������̨���� */
void KeyboardControlGimbal(Gimbal_t* gimbal)
{
    static uint32_t Mouse_Pitch_Stop  = 0;//��겻����������?
    static float Mouse_Gyro_Pitch;  //Mouse_Gyro_Yaw, ����������ģʽ�����ͳ��Pitchƫ����,��ֵ���Լ�������С,��ֹ˦ͷ����
	servo_ctrl(&Gimbal);
	if(gimbal->Action  == GIMBAL_NORMAL)
	{
		if(MOUSE_Y_MOVE_SPEED != 0)
        {
           Mouse_Gyro_Pitch -= MOUSE_Y_MOVE_SPEED * 0.000005f;//pitch�Ծ�ʹ�û�еģʽ
        }
        else if(MOUSE_Y_MOVE_SPEED == 0)
        {
            Mouse_Pitch_Stop ++ ;
            if(Mouse_Pitch_Stop > 5) //��곤ʱ��ͣ����ֹͣ�ƶ�
            {
                Mouse_Gyro_Pitch = RAMP_float(0, Mouse_Gyro_Pitch, 50);
            }
        }
        else
        {
            Mouse_Gyro_Pitch = RAMP_float(0, Mouse_Gyro_Pitch, 50);
        }
        motor.Gimbal_Pitch.Apid.goal = RampInc_float( &Mouse_Gyro_Pitch, motor.Gimbal_Pitch.Apid.goal, 10);
	}
	else if(gimbal->Action == GIMBAL_AUTO)
	{
		if(VisionValue.find_bool =='1')
		{
			motor.Gimbal_Pitch.Apid.goal = -VisionValue.pitch_value.value ;			
		}
		else
		{
			motor.Gimbal_Pitch.Apid.goal = motor.Gimbal_Pitch.Apid.now;
//			gimbal->IMUData.pitch_ok = 0;
		}
	}
}

/************************״̬������*****************/

/* �ѵ���״̬������Stopģʽ */
void Gimbal_Statemachine_2_Stop(Gimbal_t* gimbal)
{
	if(gimbal->State != GIMB_Stop)
	{
		//����һ�£��������pid
//		Clean_Gimbal_CtrlMsg(gimbal);
		
		//�й�ȥ
		gimbal->State = GIMB_Stop;
	}
}

/* �ѵ���״̬������remoteģʽ */
void Gimbal_Statemachine_2_remote(Gimbal_t* gimbal)
{
	if(gimbal->State != GIMB_Remote)
	{
//		Clean_Gimbal_CtrlMsg(gimbal);
		
		gimbal->State = GIMB_Remote;
	}
}


/* �ѵ���״̬���������ģʽ */
void Gimbal_Statemachine_2_keyboard(Gimbal_t* gimbal)
{
	if(gimbal->State != GIMB_Keyboard)
	{
//		Clean_Gimbal_CtrlMsg(gimbal);
		
		gimbal->State = GIMB_Keyboard;
	}
}

/* �ѵ���״̬�������ʼ��ģʽ */
void Gimbal_Statemachine_2_Init(Gimbal_t* gimbal)
{
	if(gimbal->State != GIMB_Init)
	{
//		Clean_Gimbal_CtrlMsg(gimbal);
		
		gimbal->State = GIMB_Init;
	}
}

float angle_test;
		
/* ״̬�������� */
void GimbalSelate(Gimbal_t* gimbal)
{
	if(rc_ctrl.rc.s[1] == 1||rc_ctrl.rc.s[1] == 3)
	{
		Gimbal_Statemachine_2_remote(gimbal);
	}
	else if(rc_ctrl.rc.s[1] == 2)
	{	
		Gimbal_Statemachine_2_keyboard(gimbal);
	}
//	else if(rc_ctrl.rc.s[0] == 2 && rc_ctrl.rc.s[1] == 2)
//	{
//		Gimbal_Statemachine_2_Stop(gimbal);
//	}

//		if(gimbal->flag.remotelose_flag) 
//		Gimbal_Statemachine_2_Stop(gimbal);
//	if((gimbal->flag.remotelose_flag) ||(rc_ctrl.Rx_Frequency[0]==0)||(rc_ctrl.Rx_Frequency[1]==0)||(rc_ctrl.Rx_Frequency[2]==0)) //ң����orͨ��ʧ��
//		Gimbal_Statemachine_2_Stop(gimbal);
		
}


