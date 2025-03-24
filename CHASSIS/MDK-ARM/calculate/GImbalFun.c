#include "GimbalFun.h"
#include "Gimbal_Task.h"
#include "Chassis_task.h"
#include "ChassisFun.h"
#include "pid.h"
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Filter.h"
#include "math.h"

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

/********************keyboard云台控制********************/

void Gimbal_Mode_Choose(Gimbal_t* gimbal)
{
		//右键按住自瞄
    if(IF_MOUSE_PRESSED_RIGH&&gimbal->IMUData.find_bool == '1')
		gimbal->Action  = GIMBAL_AUTO;
	else 
		gimbal->Action = GIMBAL_NORMAL;
}


/**/
/**
  * @brief  键鼠控制Yaw函数
  * @param  void
  * @retval void
  * @attention
  */
float TURN_180_Gimbal_Thistime = 0;
void KeyboardControlGimbal(Gimbal_t* gimbal)
{
	
	static portTickType  Key_Ctrl_CurrentTime = 0;
//	static TickType_t Turn_Time = 0;  //按下X时的时间
	static uint32_t Mouse_Yaw_Stop  = 0;
//	static uint32_t XTurn180_ms  = 0;//90°,250ms延时响应,1秒最多按4下
//	static bool Xturn180_f = 0;      //Xturn180_f 置为0表示旋转完成
	float Mouse_Gyro_Yaw; //键盘陀螺仪模式下鼠标统计yaw偏移量,此值会自己缓慢减小,防止甩头过快
	
	if(gimbal->Action == GIMBAL_NORMAL)
	{
		Key_Ctrl_CurrentTime = xTaskGetTickCount();//获取实时时间,用来做按键延时判断

		/////普通控制部分
		if(MOUSE_X_MOVE_SPEED != 0)
		{
			Mouse_Gyro_Yaw -= MOUSE_X_MOVE_SPEED * 0.000005f;//yaw仍旧使用机械模式
		}
		else if(MOUSE_X_MOVE_SPEED == 0)
		{
			Mouse_Yaw_Stop ++ ;
			if(Mouse_Yaw_Stop > 5) //鼠标长时间停留，停止移动
			{
				Mouse_Gyro_Yaw = RAMP_float(0, Mouse_Gyro_Yaw, 100);
			}
		}
		else
		{
			Mouse_Gyro_Yaw = RAMP_float(0, Mouse_Gyro_Yaw, 100);
		}
		motor.Gimbal_Yaw.Apid.goal = RampInc_float( &Mouse_Gyro_Yaw, motor.Gimbal_Yaw.Apid.goal, 10);
	}	
	else if(gimbal->Action == GIMBAL_AUTO)
	{
//		if(Gimbal.IMUData.find_bool =='1')
//		{
//					gimbal->IMUData.yaw_target = gimbal->IMUData.Vision_yaw_target;			
//			if(fabsf(gimbal->IMUData.Vision_yaw_target - gimbal->IMUData.IMU_Yaw_angle)<(180.f*(0.4f*0.075f/Gimbal.IMUData.distance_t.distance)/PI))
//				gimbal->IMUData.yaw_ok = 1;
//			else
//				gimbal->IMUData.yaw_ok = 0;
//		}
//		else
//		{
//				gimbal->IMUData.yaw_target = gimbal->IMUData.IMU_Yaw_angle;	
//				gimbal->IMUData.yaw_ok = 0;
//		}
	}
}



bool Gimbal_Movejudge_keyboard(Gimbal_t* gimbal)
{
	static uint32_t time;
	static uint32_t lasttime;

	if(MOUSE_X_MOVE_SPEED != 0)
        gimbal->flag.Move_flag = 1;
	else
	{  
	
			if(gimbal->flag.Move_flag == 1)                  //如果是第一次
			{
					lasttime = xTaskGetTickCount();              //更新时间
					time = xTaskGetTickCount();
					gimbal->flag.Move_flag = 0;
			}
			else
			{
					time = xTaskGetTickCount();
			}
			
	}

		//零漂状态机 消除因陀螺仪漂移而导致的云台漂移--normal和follow模式都要解决
        if((!IF_MOUSE_PRESSED_RIGH) && MOUSE_X_MOVE_SPEED == 0 && Chassis.Action != CHASS_GYROSCOPE &&!IF_KEY_PRESSED_G) 
        {
					if(IF_KEY_PRESSED_W || IF_KEY_PRESSED_S || IF_KEY_PRESSED_A || IF_KEY_PRESSED_D || IF_KEY_PRESSED_X ||IF_KEY_PRESSED_SHIFT)
						lasttime = time;
						if((time - lasttime) > 2000) //静止超过2秒 使用编码模式
							{
								 return 1;
							}
							else
							{
								 return 0;	
							}
				}
	else
{
			return 0;
}

}				


///*判断是否长时间未移动，如是则返回1*/
//bool Gimbal_Movejudge_remote(Gimbal_t* gimbal)
//{
//		static uint32_t time;
//    static uint32_t lasttime;
//	
//    if(RC_Ctl.rc.ch0 != 0)
//        gimbal->flag.Move_flag = 1;
//    else                                           			 //如果静止
//    {
//        if(gimbal->flag.Move_flag == 1)                  //如果是第一次
//        {
//            lasttime = xTaskGetTickCount();              //更新时间
//            time = xTaskGetTickCount();
//            gimbal->flag.Move_flag = 0;
//        }
//        else
//        {
//            time = xTaskGetTickCount();
//        }
//    }
//		
//	if((!IF_MOUSE_PRESSED_RIGH) &&(RC_Ctl.rc.ch0 == 0&&RC_Ctl.rc.ch1 == 0&&RC_Ctl.rc.ch2 == 0&&RC_Ctl.rc.ch3 == 0)&&!(Chassis.Action == CHASS_GYROSCOPE)) //如果不在自瞄模式 &&不动&& 不在小陀螺
//{
//		if((time - lasttime) > 2000) //静止超过2秒  使用编码模式
//		{
//			 return 1;
//		}
//		else
//		{
//			 return 0;	
//		}
//}
//	else
//{
//			return 0;
//}

//}

bool Gimbal_Movejudge(Gimbal_t* gimbal)
{
	if(gimbal->State == GIMB_Keyboard)
		return Gimbal_Movejudge_keyboard(gimbal);
//	else if(gimbal->State == GIMB_Remote )
//		return Gimbal_Movejudge_remote(gimbal);
	else
		return 0;
}

float test_tg;
///*********************云台输出计算*********************/
//void Gimbal_clc(Gimbal_t* gimbal)
//{
//	
//	//Yaw轴计算	
//	if(Gimbal_Movejudge(gimbal)&&Gimbal.Action == GIMBAL_NORMAL)//防止零飘
//	{
//		gimbal->flag.ControlMode = ENCODER;
//		if(!gimbal->flag.Fist_encoder)
//		{
//		Motor_4310_EnYaw.target_angle = Motor_4310_EnYaw.angle;
//		PID_clear(&DM4310_Yaw_Enangleloop);
//		PID_clear(&DM4310_Yaw_Enspeedloop);
//		}
//		
//		
//			if(IF_KEY_PRESSED_V&&!IF_KEY_PRESSED_CTRL)
//				Motor_4310_EnYaw.target_angle +=0.0015f;
//			if(IF_KEY_PRESSED_B&&!IF_KEY_PRESSED_CTRL)
//				Motor_4310_EnYaw.target_angle -=0.0015f;



//		PID_Angle_calc(&DM4310_Yaw_Enangleloop,Motor_4310_EnYaw.angle,Motor_4310_EnYaw.target_angle,-180.f,180.f);	
//		Motor_4310_EnYaw.target_speed_rpm = DM4310_Yaw_Enangleloop.out;
//		PID_calc(&DM4310_Yaw_Enspeedloop,Motor_4310_EnYaw.speed_rpm,Motor_4310_EnYaw.target_speed_rpm);	
//		Motor_4310_EnYaw.target_torque = DM4310_Yaw_Enspeedloop.out;

//		gimbal->flag.Fist_encoder = 1;
//			

//	}
//	else
//	{
//		gimbal->flag.ControlMode = IMU;
//		
//		if(gimbal->flag.Fist_encoder) //刚从编码器模式退出，目标值为当前值
//		gimbal->IMUData.yaw_target = gimbal->IMUData.IMU_Yaw_angle;

//		gimbal->flag.Fist_encoder = 0;

//			if(IF_KEY_PRESSED_V&&!IF_KEY_PRESSED_CTRL)
//				gimbal->IMUData.yaw_target+=0.0015f;
//			if(IF_KEY_PRESSED_B&&!IF_KEY_PRESSED_CTRL)
//				gimbal->IMUData.yaw_target-=0.0015f;


//		//Yaw双环
//		AngleLoop_f(&gimbal->IMUData.yaw_target,360.f);
//		Motor_4310_Yaw.target_angle = gimbal->IMUData.yaw_target;
//		PID_Angle_calc(&DM4310_Yaw_angleloop, gimbal->IMUData.IMU_Yaw_angle, Motor_4310_Yaw.target_angle, -180.f, 180.f); //带回环PID	
//		Motor_4310_Yaw.target_speed_rpm = DM4310_Yaw_angleloop.out;		
//		PID_calc(&DM4310_Yaw_speedloop,Motor_4310_Yaw.speed_rpm ,Motor_4310_Yaw.target_speed_rpm);
//		Motor_4310_Yaw.target_torque = DM4310_Yaw_speedloop.out;

//		}	
//	
//}


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

/* 状态机控制器 */
void GimbalSelate(Gimbal_t* gimbal)
{

	if(rc_ctrl.rc.s[1] == 1||rc_ctrl.rc.s[1] == 3)
	{
		Gimbal_Statemachine_2_remote(gimbal);
	}
	else if(rc_ctrl.rc.s[1] == 2 )
	{	
		Gimbal_Statemachine_2_keyboard(gimbal);
	}

//	else if(rc_ctrl.rc.s[0] == 2 && rc_ctrl.rc.s[1] == 2)
//	{
//		Gimbal_Statemachine_2_Stop(gimbal);
//	}
	
	if(Chassis.flag.remotelose_flag||( Gimbal.IMUData.Rx_Frequency <50))//遥控器or板间通信失联保护
		Gimbal_Statemachine_2_Stop(gimbal);

}


