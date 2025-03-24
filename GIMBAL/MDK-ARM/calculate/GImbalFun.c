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
  * @brief  斜坡函数,使目标输出值缓慢等于期望值
  * @param  期望最终输出,当前输出,变化速度(越大越快)
  * @retval 当前输出
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
  * @brief  斜坡函数,使目标输出值缓慢等于指针输入值
  * @param  要在当前输出量上累加的值,目标输出量,递增快慢
  * @retval 目标输出量
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
//	gimbal->Hanging_Mode = level ; //VB吊射模式 水平	
//}

/*键鼠选择云台模式*/
void Gimbal_Mode_Choose(Gimbal_t* gimbal)
{
		//右键按住自瞄
    if(IF_MOUSE_PRESSED_RIGH)
			gimbal->Action  = GIMBAL_AUTO;
		else
			gimbal->Action = GIMBAL_NORMAL;

}

/* Keyboard控制云台函数 */
void KeyboardControlGimbal(Gimbal_t* gimbal)
{
    static uint32_t Mouse_Pitch_Stop  = 0;//鼠标不动，结束响?
    static float Mouse_Gyro_Pitch;  //Mouse_Gyro_Yaw, 键盘陀螺仪模式下鼠标统计Pitch偏移量,此值会自己缓慢减小,防止甩头过快
	servo_ctrl(&Gimbal);
	if(gimbal->Action  == GIMBAL_NORMAL)
	{
		if(MOUSE_Y_MOVE_SPEED != 0)
        {
           Mouse_Gyro_Pitch -= MOUSE_Y_MOVE_SPEED * 0.000005f;//pitch仍旧使用机械模式
        }
        else if(MOUSE_Y_MOVE_SPEED == 0)
        {
            Mouse_Pitch_Stop ++ ;
            if(Mouse_Pitch_Stop > 5) //鼠标长时间停留，停止移动
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

/************************状态机控制*****************/

/* 把底盘状态机切入Stop模式 */
void Gimbal_Statemachine_2_Stop(Gimbal_t* gimbal)
{
	if(gimbal->State != GIMB_Stop)
	{
		//处理一下，比如清空pid
//		Clean_Gimbal_CtrlMsg(gimbal);
		
		//切过去
		gimbal->State = GIMB_Stop;
	}
}

/* 把底盘状态机切入remote模式 */
void Gimbal_Statemachine_2_remote(Gimbal_t* gimbal)
{
	if(gimbal->State != GIMB_Remote)
	{
//		Clean_Gimbal_CtrlMsg(gimbal);
		
		gimbal->State = GIMB_Remote;
	}
}


/* 把底盘状态机切入键盘模式 */
void Gimbal_Statemachine_2_keyboard(Gimbal_t* gimbal)
{
	if(gimbal->State != GIMB_Keyboard)
	{
//		Clean_Gimbal_CtrlMsg(gimbal);
		
		gimbal->State = GIMB_Keyboard;
	}
}

/* 把底盘状态机切入初始化模式 */
void Gimbal_Statemachine_2_Init(Gimbal_t* gimbal)
{
	if(gimbal->State != GIMB_Init)
	{
//		Clean_Gimbal_CtrlMsg(gimbal);
		
		gimbal->State = GIMB_Init;
	}
}

float angle_test;
		
/* 状态机控制器 */
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
//	if((gimbal->flag.remotelose_flag) ||(rc_ctrl.Rx_Frequency[0]==0)||(rc_ctrl.Rx_Frequency[1]==0)||(rc_ctrl.Rx_Frequency[2]==0)) //遥控器or通信失联
//		Gimbal_Statemachine_2_Stop(gimbal);
		
}


