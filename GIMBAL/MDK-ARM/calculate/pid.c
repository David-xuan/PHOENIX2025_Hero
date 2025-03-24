#include "pid.h"

void pid_I(Pid_struct *Ipid)//电流环
{
	if (Ipid->goal > Ipid->outmax)
		Ipid->goal = Ipid->outmax - 1;
	else
	if (Ipid->goal < -Ipid->outmax)
		Ipid->goal = -Ipid->outmax + 1;
	Ipid->elast = Ipid->e;
	Ipid->e = Ipid->goal - Ipid->now;
	Ipid->p = Ipid->kp * Ipid->e;//比例
	Ipid->d = Ipid->kd * (Ipid->e - Ipid->elast);//微分
	if (Ipid->e < Ipid->emax && Ipid->e > -Ipid->emax)//积分
		if (Ipid->i < -Ipid->imax)
			Ipid->i = -Ipid->imax;
		else 
		if (Ipid->i > Ipid->imax)
			Ipid->i = Ipid->imax;
		else
			Ipid->i += Ipid->ki * Ipid->e;
	else
		Ipid->i = 0;
	Ipid->out = Ipid->p + Ipid->i + Ipid->d;
	if (Ipid->out > Ipid->outmax)
		Ipid->out = Ipid->outmax;
	else
	if (Ipid->out < -Ipid->outmax)
		Ipid->out = -Ipid->outmax;
}

void pid_V(Pid_struct *Vpid)//速度环
{
	if (Vpid->goal > Vpid->outmax)
		Vpid->goal = Vpid->outmax - 1;
	else
	if (Vpid->goal < -Vpid->outmax)
		Vpid->goal = -Vpid->outmax + 1;
	Vpid->elast = Vpid->e;
	Vpid->e = Vpid->goal - Vpid->now;
	Vpid->p = Vpid->kp * Vpid->e;
	Vpid->d = Vpid->kd * (Vpid->e - Vpid->elast);
	if (Vpid->e < Vpid->emax && Vpid->e > -Vpid->emax)
	{
		if (Vpid->i < -Vpid->imax)
			Vpid->i = -Vpid->imax;
		else if (Vpid->i > Vpid->imax)
			Vpid->i = Vpid->imax;
		else
			Vpid->i += Vpid->ki * Vpid->e;
	}
	else
		Vpid->i = 0;
	Vpid->out = Vpid->p + Vpid->i + Vpid->d;
	if (Vpid->out > Vpid->outmax)
		Vpid->out = Vpid->outmax;
	else if (Vpid->out < -Vpid->outmax)
		Vpid->out = -Vpid->outmax;
}

void pid_S(Pid_struct *Spid)//角度环
{
//	while (Spid->goal > Spid->outmax)
//		Spid->goal -= Spid->outmax * 2;
//	while (Spid->goal < -Spid->outmax)
//		Spid->goal += Spid->outmax * 2;
	Spid->elast = Spid->e;
	Spid->e = Spid->goal - Spid->now;
	//过零处理//
	if (Spid->e > 0 && Spid->outmax * 2 - Spid->e < Spid->e)
		Spid->e -= Spid->outmax * 2;
	else if (Spid->e < 0 && Spid->outmax * 2 + Spid->e < -Spid->e)
		Spid->e += Spid->outmax * 2;
	//end//
	Spid->p = Spid->kp * Spid->e;
	Spid->d = Spid->kd * (Spid->e - Spid->elast);
	if (Spid->e < Spid->emax && Spid->e > -Spid->emax)
	{
		if (Spid->i < -Spid->imax)
		{
			Spid->i = -Spid->imax;
		}
		else if (Spid->i > Spid->imax)
			Spid->i = Spid->imax;
		else
			Spid->i += Spid->ki * Spid->e;
	}
	else
		Spid->i = 0;
	Spid->out = Spid->p + Spid->i + Spid->d;
	if (Spid->out > Spid->outmax)
		Spid->out = Spid->outmax;
	else
	if (Spid->out < -Spid->outmax)
		Spid->out = -Spid->outmax;
}

void pid_DM(Pid_struct *Vpid)
{
//	if (Vpid->goal > Vpid->outmax)
//		Vpid->goal = Vpid->outmax - 1;
//	else
//	if (Vpid->goal < -Vpid->outmax)
//		Vpid->goal = -Vpid->outmax + 1;
	Vpid->elast = Vpid->e;
	Vpid->e = Vpid->goal - Vpid->now;
	Vpid->p = Vpid->kp * Vpid->e;
	Vpid->d = Vpid->kd * (Vpid->e - Vpid->elast);
	if (Vpid->e < Vpid->emax && Vpid->e > -Vpid->emax)
	{
		if (Vpid->i < -Vpid->imax)
			Vpid->i = -Vpid->imax;
		else if (Vpid->i > Vpid->imax)
			Vpid->i = Vpid->imax;
		else
			Vpid->i += Vpid->ki * Vpid->e;
	}
	else
		Vpid->i = 0;
	Vpid->out = Vpid->p + Vpid->i + Vpid->d;
	if (Vpid->out > Vpid->outmax)
		Vpid->out = Vpid->outmax;
	else if (Vpid->out < -Vpid->outmax)
		Vpid->out = -Vpid->outmax;
}

void speed_control(Cal_struct *motor)//双环控速度
{
	pid_V(&motor->Vpid);
	motor->Ipid.goal =motor->Vpid.out;
	pid_I(&motor->Ipid);
}

void position_control(Cal_struct *motor)//三环控角度
{
	pid_S(&motor->Apid);
	motor->Vpid.goal =motor->Apid.out;
	pid_V(&motor->Vpid);
	motor->Ipid.goal =motor->Vpid.out;
	pid_I(&motor->Ipid);
}

void NoCurrent_speed_control(Cal_struct *motor)//单环控速度
{
	pid_V(&motor->Vpid);
}

void NoCurrent_position_control(Cal_struct *motor)//双环控角度
{
	pid_S(&motor->Apid);
	motor->Vpid.goal =motor->Apid.out;
	pid_V(&motor->Vpid);
}

void DM_position_control(Cal_struct *motor)
{
	pid_S(&motor->Apid);
	motor->Vpid.goal = -motor->Apid.out;
	pid_DM(&motor->Vpid);
}

void Chassis_PID_Init(Cal_struct *Motor)//底盘pid初始化
{
	Motor->Apid.kp = 100;
	Motor->Apid.ki = 0;
	Motor->Apid.kd = 0;
	Motor->Apid.emax = 1;
	Motor->Apid.imax = 1;
	Motor->Apid.outmax = 10000;
}

void Wheel_PID_Init(Cal_struct *Motor)//轮子pid初始化
{
	Motor->Ipid.kp = 50;
	Motor->Ipid.ki = 0.05;
	Motor->Ipid.kd = 20;
	Motor->Ipid.emax = 20;
	Motor->Ipid.imax = 1;
	Motor->Ipid.outmax = 16383;
	Motor->Vpid.kp = 40;
	Motor->Vpid.ki = 0.3;
	Motor->Vpid.kd = 8;
	Motor->Vpid.emax = 100;
	Motor->Vpid.imax = 1000;
	Motor->Vpid.outmax = 16383;
}

void Pitch_PID_Init(Cal_struct *Motor)
{
	Motor->Vpid.kp = 0.25;
	Motor->Vpid.ki = 0.025;
	Motor->Vpid.kd = 0;
	Motor->Vpid.emax = 30;
	Motor->Vpid.imax = 13;
	Motor->Vpid.outmax = 7;
	Motor->Apid.kp = 4;
	Motor->Apid.ki = 0;
	Motor->Apid.kd = 0.3;
	Motor->Apid.emax = 1;
	Motor->Apid.imax = 3;
	Motor->Apid.outmax = 3;
}

