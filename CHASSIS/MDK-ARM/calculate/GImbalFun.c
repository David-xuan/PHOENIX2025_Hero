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

/********************keyboard��̨����********************/

void Gimbal_Mode_Choose(Gimbal_t* gimbal)
{
		//�Ҽ���ס����
    if(IF_MOUSE_PRESSED_RIGH&&gimbal->IMUData.find_bool == '1')
		gimbal->Action  = GIMBAL_AUTO;
	else 
		gimbal->Action = GIMBAL_NORMAL;
}


/**/
/**
  * @brief  �������Yaw����
  * @param  void
  * @retval void
  * @attention
  */
float TURN_180_Gimbal_Thistime = 0;
void KeyboardControlGimbal(Gimbal_t* gimbal)
{
	
	static portTickType  Key_Ctrl_CurrentTime = 0;
//	static TickType_t Turn_Time = 0;  //����Xʱ��ʱ��
	static uint32_t Mouse_Yaw_Stop  = 0;
//	static uint32_t XTurn180_ms  = 0;//90��,250ms��ʱ��Ӧ,1����ఴ4��
//	static bool Xturn180_f = 0;      //Xturn180_f ��Ϊ0��ʾ��ת���
	float Mouse_Gyro_Yaw; //����������ģʽ�����ͳ��yawƫ����,��ֵ���Լ�������С,��ֹ˦ͷ����
	
	if(gimbal->Action == GIMBAL_NORMAL)
	{
		Key_Ctrl_CurrentTime = xTaskGetTickCount();//��ȡʵʱʱ��,������������ʱ�ж�

		/////��ͨ���Ʋ���
		if(MOUSE_X_MOVE_SPEED != 0)
		{
			Mouse_Gyro_Yaw -= MOUSE_X_MOVE_SPEED * 0.000005f;//yaw�Ծ�ʹ�û�еģʽ
		}
		else if(MOUSE_X_MOVE_SPEED == 0)
		{
			Mouse_Yaw_Stop ++ ;
			if(Mouse_Yaw_Stop > 5) //��곤ʱ��ͣ����ֹͣ�ƶ�
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
	
			if(gimbal->flag.Move_flag == 1)                  //����ǵ�һ��
			{
					lasttime = xTaskGetTickCount();              //����ʱ��
					time = xTaskGetTickCount();
					gimbal->flag.Move_flag = 0;
			}
			else
			{
					time = xTaskGetTickCount();
			}
			
	}

		//��Ư״̬�� ������������Ư�ƶ����µ���̨Ư��--normal��followģʽ��Ҫ���
        if((!IF_MOUSE_PRESSED_RIGH) && MOUSE_X_MOVE_SPEED == 0 && Chassis.Action != CHASS_GYROSCOPE &&!IF_KEY_PRESSED_G) 
        {
					if(IF_KEY_PRESSED_W || IF_KEY_PRESSED_S || IF_KEY_PRESSED_A || IF_KEY_PRESSED_D || IF_KEY_PRESSED_X ||IF_KEY_PRESSED_SHIFT)
						lasttime = time;
						if((time - lasttime) > 2000) //��ֹ����2�� ʹ�ñ���ģʽ
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


///*�ж��Ƿ�ʱ��δ�ƶ��������򷵻�1*/
//bool Gimbal_Movejudge_remote(Gimbal_t* gimbal)
//{
//		static uint32_t time;
//    static uint32_t lasttime;
//	
//    if(RC_Ctl.rc.ch0 != 0)
//        gimbal->flag.Move_flag = 1;
//    else                                           			 //�����ֹ
//    {
//        if(gimbal->flag.Move_flag == 1)                  //����ǵ�һ��
//        {
//            lasttime = xTaskGetTickCount();              //����ʱ��
//            time = xTaskGetTickCount();
//            gimbal->flag.Move_flag = 0;
//        }
//        else
//        {
//            time = xTaskGetTickCount();
//        }
//    }
//		
//	if((!IF_MOUSE_PRESSED_RIGH) &&(RC_Ctl.rc.ch0 == 0&&RC_Ctl.rc.ch1 == 0&&RC_Ctl.rc.ch2 == 0&&RC_Ctl.rc.ch3 == 0)&&!(Chassis.Action == CHASS_GYROSCOPE)) //�����������ģʽ &&����&& ����С����
//{
//		if((time - lasttime) > 2000) //��ֹ����2��  ʹ�ñ���ģʽ
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
///*********************��̨�������*********************/
//void Gimbal_clc(Gimbal_t* gimbal)
//{
//	
//	//Yaw�����	
//	if(Gimbal_Movejudge(gimbal)&&Gimbal.Action == GIMBAL_NORMAL)//��ֹ��Ʈ
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
//		if(gimbal->flag.Fist_encoder) //�մӱ�����ģʽ�˳���Ŀ��ֵΪ��ǰֵ
//		gimbal->IMUData.yaw_target = gimbal->IMUData.IMU_Yaw_angle;

//		gimbal->flag.Fist_encoder = 0;

//			if(IF_KEY_PRESSED_V&&!IF_KEY_PRESSED_CTRL)
//				gimbal->IMUData.yaw_target+=0.0015f;
//			if(IF_KEY_PRESSED_B&&!IF_KEY_PRESSED_CTRL)
//				gimbal->IMUData.yaw_target-=0.0015f;


//		//Yaw˫��
//		AngleLoop_f(&gimbal->IMUData.yaw_target,360.f);
//		Motor_4310_Yaw.target_angle = gimbal->IMUData.yaw_target;
//		PID_Angle_calc(&DM4310_Yaw_angleloop, gimbal->IMUData.IMU_Yaw_angle, Motor_4310_Yaw.target_angle, -180.f, 180.f); //���ػ�PID	
//		Motor_4310_Yaw.target_speed_rpm = DM4310_Yaw_angleloop.out;		
//		PID_calc(&DM4310_Yaw_speedloop,Motor_4310_Yaw.speed_rpm ,Motor_4310_Yaw.target_speed_rpm);
//		Motor_4310_Yaw.target_torque = DM4310_Yaw_speedloop.out;

//		}	
//	
//}


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

/* ״̬�������� */
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
	
	if(Chassis.flag.remotelose_flag||( Gimbal.IMUData.Rx_Frequency <50))//ң����or���ͨ��ʧ������
		Gimbal_Statemachine_2_Stop(gimbal);

}


