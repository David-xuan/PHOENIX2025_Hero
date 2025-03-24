#include "Gimbal_Task.h"
#include "Chassis_task.h"
#include "DM4310.h"
#include "bsp_can.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "GimbalFun.h"

Gimbal_t Gimbal;
extern Direc_Struct direc;
extern Motor_struct motor;
extern RC_ctrl_t rc_ctrl;
int flag = 1;
float error;
void Gimbal_task(void const * argument)
{
	portTickType currentTime;
	error = motor.Gimbal_Yaw.Apid.now;
	Gimbal.flag.imu_flag = 1;
	Yaw_PID_Init(&motor.Gimbal_Yaw);
	for(;;)
	{
		while(DM4310.dm_err == 0)
		{
			Motor_DM4310_Enable(&hcan1,DM4310_ID);
			Yaw_PID_Init(&motor.Gimbal_Yaw);
		}
		currentTime = xTaskGetTickCount(); //当前系统时间
		GimbalSelate(&Gimbal);
		if(rc_ctrl.misstimeout < 50)//遥控器未失联则执行
		{
			if(Gimbal.State == GIMB_Keyboard)    //键鼠操控
			{
				Gimbal_Mode_Choose(&Gimbal);
				KeyboardControlGimbal(&Gimbal);
				
				while(motor.Gimbal_Yaw.Apid.goal < -PI)
				motor.Gimbal_Yaw.Apid.goal += 2*PI;
				while(motor.Gimbal_Yaw.Apid.goal > PI)
				motor.Gimbal_Yaw.Apid.goal -= 2*PI;
				if(Gimbal.Action == GIMBAL_NORMAL)
				{
					if (Chassis.Action == CHASS_FOLLOW||Chassis.Action == CHASS_NORMAL)
					{
						if(!Gimbal.flag.imu_flag)
						{
							Yaw_PID_Init(&motor.Gimbal_Yaw);
							motor.Gimbal_Yaw.Apid.goal = motor.Gimbal_Yaw.Apid.now;
							Gimbal.flag.imu_flag += 1;
						}
						DM_position_control(&motor.Gimbal_Yaw);
					}
					else if(Chassis.Action == CHASS_GYROSCOPE)
					{
						if(Gimbal.flag.imu_flag)
						{
							Yaw_PID_Init_imu(&motor.Gimbal_Yaw);
							motor.Gimbal_Yaw.Apid.goal = Gimbal.IMUData.IMU_Yaw_angle;
							Gimbal.flag.imu_flag -= 1;
						}
						DM_position_control_imu(&motor.Gimbal_Yaw);
					}
				}
				else if(Gimbal.Action == GIMBAL_AUTO)
				{
						motor.Gimbal_Yaw.Apid.goal = Yaw_angle_t.vision_value.value;
						DM_position_control(&motor.Gimbal_Yaw);
				}
				Motor_DM4310_send(&hcan1,DM4310_ID, 0, 0, 0, 0, motor.Gimbal_Yaw.Vpid.out);//motor.Gimbal_Yaw.Vpid.out
			}
			else if(rc_ctrl.rc.s[1] == 3||rc_ctrl.rc.s[1] == 1)//编码器模式
			{
//				if(!flag)//目标值设为当前编码器值
//				{
//					Yaw_PID_Init(&motor.Gimbal_Yaw);
//					motor.Gimbal_Yaw.Apid.goal = motor.Gimbal_Yaw.Apid.now;
//					flag += 1;
//				}
				if(rc_ctrl.rc.s[0] == 3 ||rc_ctrl.rc.s[0] == 1)//手动
				{
					motor.Gimbal_Yaw.Apid.goal -= direc.yaw;//-=direc.yaw;
					
					while(motor.Gimbal_Yaw.Apid.goal < -PI)
					motor.Gimbal_Yaw.Apid.goal += 2*PI;
					while(motor.Gimbal_Yaw.Apid.goal > PI)
					motor.Gimbal_Yaw.Apid.goal -= 2*PI;

					DM_position_control(&motor.Gimbal_Yaw);
					Motor_DM4310_send(&hcan1,DM4310_ID, 0, 0, 0, 0, motor.Gimbal_Yaw.Vpid.out);//motor.Gimbal_Yaw.Vpid.out
				}
				else if(rc_ctrl.rc.s[0] == 2)//自瞄
				{
//					motor.Gimbal_Yaw.Apid.goal = Yaw_angle_t.vision_value.value + error;
					motor.Gimbal_Yaw.Apid.goal = Yaw_angle_t.vision_value.value;
					while(motor.Gimbal_Yaw.Apid.goal < -PI)
					motor.Gimbal_Yaw.Apid.goal += 2*PI;
					while(motor.Gimbal_Yaw.Apid.goal > PI)
					motor.Gimbal_Yaw.Apid.goal -= 2*PI;
					DM_position_control(&motor.Gimbal_Yaw);
					Motor_DM4310_send(&hcan1,DM4310_ID, 0, 0, 0, 0, motor.Gimbal_Yaw.Vpid.out);//motor.Gimbal_Yaw.Vpid.out
				}
			}
//			else if(rc_ctrl.rc.s[1] == 1)//陀螺仪模式
//			{
//				if(flag)//目标值设定为当前陀螺仪yaw值
//				{
//					Yaw_PID_Init_imu(&motor.Gimbal_Yaw);
//					motor.Gimbal_Yaw.Apid.goal = Gimbal.IMUData.IMU_Yaw_angle;
//					flag -= 1;
//				}
//				motor.Gimbal_Yaw.Apid.goal -= direc.yaw;//-=direc.yaw;
//				
//				while(motor.Gimbal_Yaw.Apid.goal < -PI)
//				motor.Gimbal_Yaw.Apid.goal += 2*PI;
//				while(motor.Gimbal_Yaw.Apid.goal > PI)
//				motor.Gimbal_Yaw.Apid.goal -= 2*PI;

//				DM_position_control_imu(&motor.Gimbal_Yaw);
//				Motor_DM4310_send(&hcan1,DM4310_ID, 0, 0, 0, 0, motor.Gimbal_Yaw.Vpid.out);//motor.Gimbal_Yaw.Vpid.out
//			}
		}
		else
		{
			Motor_DM4310_send(&hcan1,DM4310_ID, 0, 0, 0, 0, 0);
		}
		osDelay(1);
		vTaskDelayUntil(&currentTime, 1); //绝对延时
	}
}
