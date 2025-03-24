#ifndef PID_H
#define PID_H
#include "stm32f4xx.h"
#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

typedef struct
{
	float goal;
	float now;
	float kp;//比例
	float ki;//积分
	float kd;//微分
	float p;
	float i;
	float d;
	float e;
	float elast;
	float emax;
	float imax;//积分限幅
	float out;
	float outmax;//输出限幅
}Pid_struct;

typedef struct
{
	Pid_struct Ipid;//电流
	Pid_struct Vpid;//速度
	Pid_struct Apid;//角度
}Cal_struct;

typedef struct
{
	Cal_struct Motor1;
	Cal_struct Motor2;
	Cal_struct Motor3;
	Cal_struct Motor4;
	Cal_struct Gimbal_Pitch;
}Motor_struct;


void pid_I(Pid_struct *Ipid);
void pid_V(Pid_struct *Vpid);
void pid_S(Pid_struct *Spid);
void pid_DM(Pid_struct *Vpid);

void speed_control(Cal_struct *motor);
void position_control(Cal_struct *motor);

void NoCurrent_speed_control(Cal_struct *motor);
void NoCurrent_position_control(Cal_struct *motor);
void DM_position_control(Cal_struct *motor);

void Chassis_PID_Init(Cal_struct *Motor);
void Wheel_PID_Init(Cal_struct *Motor);
void Pitch_PID_Init(Cal_struct *Motor);





#endif




