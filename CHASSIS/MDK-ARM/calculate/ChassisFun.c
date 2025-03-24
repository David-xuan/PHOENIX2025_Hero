#include "ChassisFun.h"
#include "Chassis_task.h"
#include "Gimbal_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pid.h"
#include "remote_control.h"
#include "math.h"
#include "bsp_can.h"
#include "judge.h"

/*���ֱ�������*/
//Chassis_Speed_t orispeed;				//����ǰԭʼĿ���ٶ�
int16_t chassis_setspeed[4];    //����Ŀ��ת��


/* �õ��ĺ������� */
extern chass_t Chassis;
void Chassis_Statemachine_2_Stop(chass_t* chassis);
void Chassis_Statemachine_2_remote(chass_t* chassis);



//flag�Լ�״̬��ʼ��
void Chassis_flag_Init(chass_t* chassis)
{
	chassis->flag.remotelose_flag =1;
	chassis->flag.gimbal_follow = GIMBAL_HEAD;

	
	chassis->State 	= CHASS_Init;
	chassis->Action =	CHASS_NORMAL;
}


/*�жϵ�ǰ����*/
void Gimbal_follow_Judge(chass_t* chassis)
{
		if(fabsf(motor.Gimbal_Yaw.Apid.now)<(PI/2))
      chassis->flag.gimbal_follow = GIMBAL_HEAD;
		else
			chassis->flag.gimbal_follow = GIMBAL_TAIL;
}
	
void Chassis_Choose_keyboard(chass_t* chassis)
{
	if(!IF_KEY_PRESSED_F)
			rc_ctrl.flag.F = 0;
	if(IF_KEY_PRESSED_F && rc_ctrl.flag.F == 0) 
	{
		rc_ctrl.flag.F = 1;
		if(chassis->Action == CHASS_NORMAL)
		{
			chassis->Action = CHASS_FOLLOW;
		}
		else
		{
			chassis->Action = CHASS_NORMAL;
		}
	
	}

	
	if(IF_KEY_PRESSED_SHIFT)
	{
		chassis->Action = CHASS_GYROSCOPE;
	}
	else if(!IF_KEY_PRESSED_SHIFT&&chassis->Action == CHASS_GYROSCOPE)
	{
		chassis->Action = CHASS_FOLLOW;
	}
}

/**
  * @brief  ����ģʽ�µ����˶�����
  * @param  �ٶ���������    �����ٶ�(���293)
  * @retval void
  * @attention  ���̿���ǰ������ƽ��,ƽ���޻�е��������ģʽ֮��
  *             ��Ҫ��ȡʱ��������б�º�������
  */
/************���̸���ģʽ��һЩ��������*************/
float    Chassis_Standard_Move_Max;                 //����ǰ������ƽ������
int16_t  timeXFron, timeXBack, timeYLeft, timeYRigh;//����  s  w  d  a

//����ģʽ��ȫ���ƶ�����,б����
float Slope_Chassis_Move_Fron, Slope_Chassis_Move_Back;
float Slope_Chassis_Move_Left, Slope_Chassis_Move_Righ;
static void Chassis_Keyboard_Move_Calculate( int16_t sMoveMax, int16_t sMoveRamp_inc, int16_t sMoveRamp_dec )
{
    static portTickType  ulCurrentTime = 0;
    static uint32_t  ulDelay = 0;

	static uint16_t w_cnt = 0;
	static bool W = 0;
	static uint16_t s_cnt = 0;
	static bool S = 0;
    static uint16_t a_cnt = 0;
    static bool A = 0;
    static uint16_t d_cnt = 0;
    static bool D = 0;
	

    ulCurrentTime = xTaskGetTickCount();//��ǰϵͳʱ��

    if (ulCurrentTime >= ulDelay)//ÿ10ms�仯һ��б����
    {
        ulDelay = ulCurrentTime + 10;

        if (IF_KEY_PRESSED_W)
        {
            w_cnt = 0;
            W = 1;
            timeXBack = 0;//����ǰ�������б�¹���,�����´μ������б��
        }
        else
        {
            w_cnt++;
        }
        if(w_cnt > 10)
        {
            w_cnt = 0;
            W = 0;
        }

        if (IF_KEY_PRESSED_S)
        {
            s_cnt = 0;
            S = 1;
            timeXFron = 0;//ͬ��
        }
        else
        {
            s_cnt++;
        }
        if(s_cnt > 10)
        {
            s_cnt = 0;
            S = 0;
        }

        if (IF_KEY_PRESSED_D)
        {
            d_cnt = 0;
            D=1;
            timeYRigh = 0;
        }
        else
        {
            d_cnt++;
        }
        if(d_cnt > 10)
        {
            d_cnt = 0;
            D = 0;
        }

        if (IF_KEY_PRESSED_A)
        {
            a_cnt = 0;
            A=1;
            timeYLeft = 0;
        }
        else
        {
            a_cnt++;
        }
        if(a_cnt > 10)
        {
            a_cnt = 0;
            A = 0;
        }

        Slope_Chassis_Move_Fron = (int16_t)( sMoveMax *
                                             Chassis_Key_MoveRamp( W, &timeXFron, sMoveRamp_inc / 1.5f, sMoveRamp_dec ) );
        
		Slope_Chassis_Move_Back = (int16_t)( -sMoveMax *
                                             Chassis_Key_MoveRamp( S, &timeXBack, sMoveRamp_inc / 1.5f, sMoveRamp_dec ) );

        Slope_Chassis_Move_Left = (int16_t)( -sMoveMax *
                                             Chassis_Key_MoveRamp( A, &timeYRigh, sMoveRamp_inc / 2.0f, sMoveRamp_dec ) );

        Slope_Chassis_Move_Righ = (int16_t)( sMoveMax *
                                             Chassis_Key_MoveRamp( D, &timeYLeft, sMoveRamp_inc / 2.0f, sMoveRamp_dec ) );

		if(Chassis.Action == CHASS_NORMAL)
        {
//			if(cap_info.cap_status == CAP_STATUS_FLAG && cap_info.switch_status == CAP_SWITCH_OPEN)
//			{
				direc.front = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) / 2.0f;
				direc.right = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) / 2.0f;
				direc.rotate = 0;
				ChassisSolving(&direc,&motion);
//			}
//			else
//			{
//				orispeed.vx  = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) / 7000.0f; //ǰ�����
//				orispeed.vy  = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) / 7000.0f; //���Ҽ���
//			}
		}
		else if(Chassis.Action == CHASS_FOLLOW)
        {   
//			if(cap_info.cap_status == CAP_STATUS_FLAG && cap_info.switch_status == CAP_SWITCH_OPEN)
//			{
				direc.front = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) / 2.0f; 
				direc.right = 0;
				direc.rotate = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) / 4.0f;
				ChassisSolving(&direc,&motion);
//			}
//			else
//			{
//				orispeed.vx  = -(Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) / 7000.0f; //ǰ�����
//				orispeed.vy  = -(Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) / 7000.0f; //���Ҽ���
//			}
		} 
        else if(Chassis.Action == CHASS_GYROSCOPE)
        {
//		    if(cap_info.cap_status == CAP_STATUS_FLAG && cap_info.switch_status == CAP_SWITCH_OPEN)
//		    {
				direc.front = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) / 2.0f;
				direc.right = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) / 4.0f;
				direc.rotate = 2000.0f;
				ChassisSolving(&direc,&motion);
//		    }
//		    else
//		    {
//				orispeed.vx  = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) / 7000.0f; //ǰ�����
//				orispeed.vy  = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) / 7000.0f; //���Ҽ���
//		    }
		}
	}
}

//б�º���
void LimitValue_f(float*VALUE, float MAX, float MIN)
{
    if(*VALUE > MAX)
        *VALUE = MAX;
    else if(*VALUE < MIN)
        *VALUE = MIN;
}

/**
  * @brief  ���̼���б�º���
  * @param  �жϰ����Ƿ񱻰���, ʱ����, ÿ�����ӵ���, һ��Ҫ��С����
  * @retval б�±���ϵ��
  * @attention  0~1
  */
float Chassis_Key_MoveRamp( uint8_t status, int16_t *time, int16_t inc, int16_t dec )
{
    float  factor = 0;
    factor = 0.1 * sqrt( 0.1 * (*time) );  //�����ٶ�б��,time�ۼӵ�296.3б�¾����

    if (status == 1){  //����������
		
        if (factor < 1)//��ֹtime̫��
			*time += inc;
		
    }else{  //�����ɿ�
        if (factor > 0)
        {
            *time -= dec;
            if (*time < 0)
				*time = 0;
        }
    }
    LimitValue_f(&factor,1,0);//ע��һ����float�����޷�
	
    return factor;  //ע�ⷽ��
}





/**
  * @brief  ���̿��Ƶ���ģʽ
  * @param  void
  * @retval void
  * @attention
  */
void KeyboardControlChassis(chass_t* chassis)
{
		if(IF_KEY_PRESSED_Z)
		{
			Chassis_Keyboard_Move_Calculate(4500, 10, 2000);
		}
		else
		{
			if(chassis->Action == CHASS_NORMAL||chassis->Action == CHASS_FOLLOW)
			{
//				if(cap_info.cap_status == CAP_STATUS_FLAG && cap_info.switch_status == CAP_SWITCH_OPEN &&JUDGE_fGetRemainEnergy()>10)
//					Chassis_Keyboard_Move_Calculate(15000, 10, 2000);
//				else
					Chassis_Keyboard_Move_Calculate(12000, 10, 2000);
//				if(!Gimbal.flag.Fist_encoder&&chassis->Action == CHASS_FOLLOW)
//					orispeed.vw = RAMP_float(PID_calc(&Chassis_Follow_PID, Find_Y_AnglePNY(), 0), orispeed.vw, 0.0008);
//				else
//				{
//					orispeed.vw = 0;
//				}					
			}
			else if(chassis->Action == CHASS_GYROSCOPE)
			{
//				if(cap_info.cap_status == CAP_STATUS_FLAG && cap_info.switch_status == CAP_SWITCH_OPEN)
//					Chassis_Keyboard_Move_Calculate(15000, 8, 30);
//				else
					Chassis_Keyboard_Move_Calculate(10000, 8, 30);
				uint16_t power_limit = JUDGE_usGetPowerLimit();

//				if(power_limit <= 70)
//					orispeed.vw=RAMP_float(0.0025, orispeed.vw, 0.00005);
//				else if(power_limit>70 && power_limit <100)
//					orispeed.vw=RAMP_float(0.0030, orispeed.vw, 0.00005);
//				else
//					orispeed.vw=RAMP_float(0.0045, orispeed.vw, 0.00005);
			}
		}
}


void get_chassis_power_and_buffer(float *chassis_power,float *chassis_power_buffer)
{
    *chassis_power=JUDGE_fGetChassisPower();//��ȡʵʱ����
    *chassis_power_buffer=JUDGE_fGetRemainEnergy();//��ȡʣ�ཹ������
}

float test_t,//Ͷ�����Ĺ���ֵ
	  test_p=60,//�趨ֵ
      test_now;//��ǰֵ

void Chassis_Power_Limit(void)
{
    uint16_t max_power_limit=50;
    float chassis_max_power=0;
    float initial_give_power[4];
    float initial_total_power=0;
    float scaled_give_power[4];
    float chassis_power=0.0f;
    float chassis_power_buffer=0.0f;
    float toque_coefficient=1.99688994e-6f;
    float k1=1.453e-07;
    //ʵ������k2   1.453e-07;  1.255e-07;           1.653e-07
    float k2=1.23e-07;
    //ʵ������k1   1.23e-07;    1.44e-07;           1.43e-07
    float constant=3.343f;
    //             4.081f;       3.343;С���ݲ���   2.081f  ���Ҳ�ǣ����ǲ���С���ݾ���
    get_chassis_power_and_buffer(&chassis_power,&chassis_power_buffer);
		max_power_limit=JUDGE_usGetPowerLimit();
//		max_power_limit=test_p;
//		if(chassis_power_buffer<30)
//		chassis_max_power = max_power_limit -5;
//		else if(chassis_power_buffer<10)
//			chassis_max_power = max_power_limit -15;
//		else
//			chassis_max_power = max_power_limit ;
		if(max_power_limit <= 60)
			chassis_max_power = max_power_limit - 2;
		else if(max_power_limit <= 90)
			chassis_max_power = max_power_limit - 5;
		else if(max_power_limit <= 120)
			chassis_max_power = max_power_limit - 12;
		test_t = chassis_max_power;
		
    for(uint8_t i=0;i<4;i++)
    {
        initial_give_power[i]=motor.Motor[i].Vpid.out*toque_coefficient*motor.Motor[i].Vpid.now
        +k2*motor.Motor[i].Vpid.now*motor.Motor[i].Vpid.now
        +k1*motor.Motor[i].Vpid.out*motor.Motor[i].Vpid.out
        +constant;
    
    
    if(initial_give_power[i]<0)
        continue;
    initial_total_power+=initial_give_power[i];
    }
    test_now = initial_total_power;
    if(initial_total_power>chassis_max_power)
    {
    float power_scale =chassis_max_power/initial_total_power;
    for(uint8_t i=0;i<4;i++)
        {
            scaled_give_power[i]=initial_give_power[i]*power_scale;
           if(scaled_give_power[i]<0)
              continue;
              
               float b=toque_coefficient * motor.Motor[i].Vpid.now;
               float c=k2*motor.Motor[i].Vpid.now*motor.Motor[i].Vpid.now-scaled_give_power[i]+constant;
        
        if(motor.Motor[i].Vpid.out>0)
            {
                float temp=(-b+sqrt(b*b-4*k1*c))/(2*k1);
                if(temp>16000)

                {
                  motor.Motor[i].Vpid.out=16000;  
                }
                else motor.Motor[i].Vpid.out=temp;
                
            }
                else 
                {
            float temp=(-b-sqrt(b*b-4*k1*c))/(2*k1);
                if(temp<-16000)
                {
                motor.Motor[i].Vpid.out=-16000;
                }
                else motor.Motor[i].Vpid.out=temp;
                }
            }
    }
}

/*************************************************************************״̬������*************************************************************************/

/* ״̬�������� */
void ChassisSelate(chass_t* chassis)
{
	
		if(rc_ctrl.rc.s[1] == 1||rc_ctrl.rc.s[1] == 3)
			Chassis_Statemachine_2_remote(chassis);
		else if(rc_ctrl.rc.s[1] == 2)
			Chassis_Statemachine_2_keyboard(chassis);
//		else if(rc_ctrl.rc.s[0] == 2&rc_ctrl.rc.s[0] == 2)
//			Chassis_Statemachine_2_Stop(chassis);
		
		if(rc_ctrl.misstimeout > 50)//ң����ʧ������
		{
			chassis->flag.remotelose_flag = 1;
			Chassis_Statemachine_2_Stop(chassis);			
		}
		else
		{
			chassis->flag.remotelose_flag = 0;		
		}
}

/* �ѵ���״̬������Stopģʽ */
void Chassis_Statemachine_2_Stop(chass_t* chassis)
{
	if(chassis->State != CHASS_Stop)
	{
		//����һ�£��������pid
//		Clean_Chassis_CtrlMsg(chassis);
		
		//�й�ȥ
		chassis->State = CHASS_Stop;
	}
}

/* �ѵ���״̬������remoteģʽ */
void Chassis_Statemachine_2_remote(chass_t* chassis)
{
	if(chassis->State != CHASS_Remote)
	{
//		Clean_Chassis_CtrlMsg(chassis);
		
		chassis->State = CHASS_Remote;
	}
}


/* �ѵ���״̬���������ģʽ */
void Chassis_Statemachine_2_keyboard(chass_t* chassis)
{
	if(chassis->State != CHASS_Keyboard)
	{
//		Clean_Chassis_CtrlMsg(chassis);
		
		chassis->State = CHASS_Keyboard;
	}
}

/* �ѵ���״̬�������ʼ��ģʽ */
void Chassis_Statemachine_2_Init(chass_t* chassis)
{
	if(chassis->State != CHASS_Init)
	{
//		Clean_Chassis_CtrlMsg(chassis);
		chassis->State = CHASS_Init;
	}
}


