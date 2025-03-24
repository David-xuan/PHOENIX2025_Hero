#include "Chassis_task.h"
#include "ChassisFun.h"

extern Direc_Struct direc;
extern Motor_struct motor;
extern Motion_Struct motion;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern RC_ctrl_t rc_ctrl;
chass_t Chassis;

static void speed_control(Motor_struct* MOTOR)
{
	NoCurrent_speed_control(&MOTOR->Motor[0]);
	NoCurrent_speed_control(&MOTOR->Motor[1]);
	NoCurrent_speed_control(&MOTOR->Motor[2]);
	NoCurrent_speed_control(&MOTOR->Motor[3]);
}

void Chassis_task(void const * argument)
{
	for(;;)
	{
		ChassisSelate(&Chassis);
		if(Chassis.State == CHASS_Keyboard)
		{
			Chassis_Choose_keyboard(&Chassis);
			KeyboardControlChassis(&Chassis);
			
			speed_control(&motor);
			Chassis_Power_Limit();			
			Set_Transmit(&hcan1,&motor);
			osDelay(1);
		}
		else if(rc_ctrl.rc.s[1]==3)
		{
			ChassisSolving(&direc,&motion);
			
			speed_control(&motor);			
			Set_Transmit(&hcan1,&motor);
			osDelay(1);
		}
		else if(rc_ctrl.rc.s[1]==1)//旋转
		{
			//motor1反转；motor2正转；motor4反转；motor3正转
			motor.Motor[0].Vpid.goal = 3*rc_ctrl.rc.ch[2];
			motor.Motor[1].Vpid.goal = 3*rc_ctrl.rc.ch[2];
			motor.Motor[2].Vpid.goal = 3*rc_ctrl.rc.ch[2];
			motor.Motor[3].Vpid.goal = 3*rc_ctrl.rc.ch[2];

			speed_control(&motor);			
			Set_Transmit(&hcan1,&motor);
			osDelay(1);
		}
	}
}
