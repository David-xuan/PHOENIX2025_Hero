#include "Shoot_task.h"
#define speed_debug 1
float speed_hou = 5350;
float speed_qian = 5300;
extern float speed_hou;
extern float speed_qian;
extern RC_ctrl_t rc_ctrl;
extern Motor_struct motor;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
	
void Shoot_task(void const *argument)
{
	for(;;)
	{
		if(rc_ctrl.rc.s[0]==3||rc_ctrl.rc.s[0]==2)
		{		
			if(!IF_KEY_PRESSED_Q)
				rc_ctrl.flag.Q = 0;
			if(IF_KEY_PRESSED_Q && rc_ctrl.flag.Q == 0)
			{
				rc_ctrl.flag.Q = 1;
				speed_qian += 50;
				speed_hou += 50;
			}
			if(!IF_KEY_PRESSED_E)
				rc_ctrl.flag.E = 0;
			if(IF_KEY_PRESSED_E && rc_ctrl.flag.E == 0)
			{
				rc_ctrl.flag.E = 1;
				speed_qian -= 50;
				speed_hou -= 50;
			}

			if(speed_debug)
			{
				motor.Motor1.Vpid.goal = speed_qian;
				motor.Motor2.Vpid.goal = speed_hou;
				motor.Motor3.Vpid.goal = -speed_hou;
				motor.Motor4.Vpid.goal = -speed_qian;
			}
			else
			{
				motor.Motor1.Vpid.goal = 5500;
				motor.Motor2.Vpid.goal = 5500;
				motor.Motor3.Vpid.goal = -5500;
				motor.Motor4.Vpid.goal = -5500;
			}

			NoCurrent_speed_control(&motor.Motor1);
			NoCurrent_speed_control(&motor.Motor2);
			NoCurrent_speed_control(&motor.Motor3);
			NoCurrent_speed_control(&motor.Motor4);
			
			Set_Transmit(&hcan1,&motor);
		}
		else
		{
			motor.Motor1.Vpid.goal = 0;
			motor.Motor2.Vpid.goal = 0;
			motor.Motor3.Vpid.goal = 0;
			motor.Motor4.Vpid.goal = 0;
			
			NoCurrent_speed_control(&motor.Motor1);
			NoCurrent_speed_control(&motor.Motor2);
			NoCurrent_speed_control(&motor.Motor3);
			NoCurrent_speed_control(&motor.Motor4);
			
			Set_Transmit(&hcan1,&motor);

		}
		osDelay(1);
	}
}
