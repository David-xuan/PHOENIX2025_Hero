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

/*部分变量定义*/
//Chassis_Speed_t orispeed;				//解算前原始目标速度
int16_t chassis_setspeed[4];    //最终目标转速


/* 用到的函数声明 */
extern chass_t Chassis;
void Chassis_Statemachine_2_Stop(chass_t* chassis);
void Chassis_Statemachine_2_remote(chass_t* chassis);



//flag以及状态初始化
void Chassis_flag_Init(chass_t* chassis)
{
	chassis->flag.remotelose_flag =1;
	chassis->flag.gimbal_follow = GIMBAL_HEAD;

	
	chassis->State 	= CHASS_Init;
	chassis->Action =	CHASS_NORMAL;
}


/*判断当前朝向*/
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
  * @brief  键盘模式下底盘运动计算
  * @param  速度最大输出量    增加速度(最大293)
  * @retval void
  * @attention  键盘控制前后左右平移,平移无机械和陀螺仪模式之分
  *             需要获取时间来进行斜坡函数计算
  */
/************底盘各类模式的一些辅助定义*************/
float    Chassis_Standard_Move_Max;                 //底盘前后左右平移限速
int16_t  timeXFron, timeXBack, timeYLeft, timeYRigh;//键盘  s  w  d  a

//键盘模式下全向移动计算,斜坡量
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
	

    ulCurrentTime = xTaskGetTickCount();//当前系统时间

    if (ulCurrentTime >= ulDelay)//每10ms变化一次斜坡量
    {
        ulDelay = ulCurrentTime + 10;

        if (IF_KEY_PRESSED_W)
        {
            w_cnt = 0;
            W = 1;
            timeXBack = 0;//按下前进则后退斜坡归零,方便下次计算后退斜坡
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
            timeXFron = 0;//同理
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
//				orispeed.vx  = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) / 7000.0f; //前后计算
//				orispeed.vy  = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) / 7000.0f; //左右计算
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
//				orispeed.vx  = -(Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) / 7000.0f; //前后计算
//				orispeed.vy  = -(Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) / 7000.0f; //左右计算
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
//				orispeed.vx  = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) / 7000.0f; //前后计算
//				orispeed.vy  = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) / 7000.0f; //左右计算
//		    }
		}
	}
}

//斜坡函数
void LimitValue_f(float*VALUE, float MAX, float MIN)
{
    if(*VALUE > MAX)
        *VALUE = MAX;
    else if(*VALUE < MIN)
        *VALUE = MIN;
}

/**
  * @brief  底盘键盘斜坡函数
  * @param  判断按键是否被按下, 时间量, 每次增加的量, 一共要减小的量
  * @retval 斜坡比例系数
  * @attention  0~1
  */
float Chassis_Key_MoveRamp( uint8_t status, int16_t *time, int16_t inc, int16_t dec )
{
    float  factor = 0;
    factor = 0.1 * sqrt( 0.1 * (*time) );  //计算速度斜坡,time累加到296.3斜坡就完成

    if (status == 1){  //按键被按下
		
        if (factor < 1)//防止time太大
			*time += inc;
		
    }else{  //按键松开
        if (factor > 0)
        {
            *time -= dec;
            if (*time < 0)
				*time = 0;
        }
    }
    LimitValue_f(&factor,1,0);//注意一定是float类型限幅
	
    return factor;  //注意方向
}





/**
  * @brief  键盘控制底盘模式
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
    *chassis_power=JUDGE_fGetChassisPower();//读取实时功率
    *chassis_power_buffer=JUDGE_fGetRemainEnergy();//读取剩余焦耳能量
}

float test_t,//投入计算的功率值
	  test_p=60,//设定值
      test_now;//当前值

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
    //实际上是k2   1.453e-07;  1.255e-07;           1.653e-07
    float k2=1.23e-07;
    //实际上是k1   1.23e-07;    1.44e-07;           1.43e-07
    float constant=3.343f;
    //             4.081f;       3.343;小陀螺不稳   2.081f  这个也是，但是不开小陀螺巨稳
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

/*************************************************************************状态机控制*************************************************************************/

/* 状态机控制器 */
void ChassisSelate(chass_t* chassis)
{
	
		if(rc_ctrl.rc.s[1] == 1||rc_ctrl.rc.s[1] == 3)
			Chassis_Statemachine_2_remote(chassis);
		else if(rc_ctrl.rc.s[1] == 2)
			Chassis_Statemachine_2_keyboard(chassis);
//		else if(rc_ctrl.rc.s[0] == 2&rc_ctrl.rc.s[0] == 2)
//			Chassis_Statemachine_2_Stop(chassis);
		
		if(rc_ctrl.misstimeout > 50)//遥控器失联保护
		{
			chassis->flag.remotelose_flag = 1;
			Chassis_Statemachine_2_Stop(chassis);			
		}
		else
		{
			chassis->flag.remotelose_flag = 0;		
		}
}

/* 把底盘状态机切入Stop模式 */
void Chassis_Statemachine_2_Stop(chass_t* chassis)
{
	if(chassis->State != CHASS_Stop)
	{
		//处理一下，比如清空pid
//		Clean_Chassis_CtrlMsg(chassis);
		
		//切过去
		chassis->State = CHASS_Stop;
	}
}

/* 把底盘状态机切入remote模式 */
void Chassis_Statemachine_2_remote(chass_t* chassis)
{
	if(chassis->State != CHASS_Remote)
	{
//		Clean_Chassis_CtrlMsg(chassis);
		
		chassis->State = CHASS_Remote;
	}
}


/* 把底盘状态机切入键盘模式 */
void Chassis_Statemachine_2_keyboard(chass_t* chassis)
{
	if(chassis->State != CHASS_Keyboard)
	{
//		Clean_Chassis_CtrlMsg(chassis);
		
		chassis->State = CHASS_Keyboard;
	}
}

/* 把底盘状态机切入初始化模式 */
void Chassis_Statemachine_2_Init(chass_t* chassis)
{
	if(chassis->State != CHASS_Init)
	{
//		Clean_Chassis_CtrlMsg(chassis);
		chassis->State = CHASS_Init;
	}
}


