#include "vision.h"
#include "string.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "INS_Task.h"
#include "DM4310.h"
#include "HWT606.h"
#include "can.h"

VisionValueType VisionValue;
VisionTransmitType VisionTransmit;
uint8_t vision_buf[VISION_BUFLEN];
uint8_t vision_transmit_buff[VISION_BUFLEN] = {"s000000000000000000000000000000e"};//[2]a
uint32_t* imu_quat_pointer[4];
uint32_t* imu_angle_pointer[3];
 

void vision_callback_handler(uint8_t *buff)     //0-18
{
    if(buff[0] == 's' && buff[31] == 'e' && buff[1] == 0xA0 ) //自瞄
    {
		memcpy(&(VisionValue.find_bool), &buff[2], 1);
        memcpy(&(VisionValue.yaw_value.value), &buff[3], 4);
        memcpy(&(VisionValue.pitch_value.value), &buff[7], 4);
//		memcpy(&(VisionValue.distance_t.buff[0]), &buff[11], 4);
    }
    if( (VisionValue.yaw_value.value > 100 ) || (VisionValue.pitch_value.value > 100 ) )
    {
        VisionValue.yaw_value.value = 0 ;
        VisionValue.pitch_value.value = 0 ;
    }        
}
//#define BLUE  0
//#define RED   1
//memcpy(目标数组，源，大小)
extern Gimbal_t Gimbal;
extern Motor_struct motor;
void vision_transmit(){
	
	if(Gimbal.Robot_color)
		VisionTransmit.enemy_team_color = 'b';
	else
		VisionTransmit.enemy_team_color  = 'r';
		VisionTransmit.mode  = 0;  //自瞄

		VisionTransmit.rune_flag='0';

//	imu_quat_pointer[0] = (uint32_t *)&Gimbal.IMUData.quat[0];
//	imu_quat_pointer[1] = (uint32_t *)&Gimbal.IMUData.quat[1];
//	imu_quat_pointer[2] = (uint32_t *)&Gimbal.IMUData.quat[2];
//	imu_quat_pointer[3] = (uint32_t *)&Gimbal.IMUData.quat[3];

//	imu_angle_pointer[0] = (uint32_t *)&Gimbal.IMUData.ol[0];			//yaw
//	imu_angle_pointer[1] = (uint32_t *)&Gimbal.IMUData.ol[1];		//pitch 用电机编码器映射为头的度数

	//开始通信
	vision_transmit_buff[0] = 's';
    vision_transmit_buff[1] = 0xB0;
	vision_transmit_buff[10] = VisionTransmit.enemy_team_color ;
	vision_transmit_buff[11] = VisionTransmit.mode ;
    vision_transmit_buff[12] = VisionTransmit.rune_flag;
	memcpy(&(vision_transmit_buff[12]),&VisionTransmit.rune_flag, 1);
	//四元数和角度进行切换看视觉需要
//#if  defined(_VISION_TRAN_QUAT_)&&!defined(_VISION_TRAN_YAW_PITCH_)
//	vision_transmit_buff[7] = GET_LOW_BYTE0(*imu_quat_pointer[0]);
//	vision_transmit_buff[8] = GET_LOW_BYTE1(*imu_quat_pointer[0]);	
//	vision_transmit_buff[9] = GET_LOW_BYTE2(*imu_quat_pointer[0]);
//	vision_transmit_buff[10] = GET_LOW_BYTE3(*imu_quat_pointer[0]);	
//	vision_transmit_buff[11] = GET_LOW_BYTE0(*imu_quat_pointer[1]);
//	vision_transmit_buff[12] = GET_LOW_BYTE1(*imu_quat_pointer[1]);
//	vision_transmit_buff[13] = GET_LOW_BYTE2(*imu_quat_pointer[1]);
//	vision_transmit_buff[14] = GET_LOW_BYTE3(*imu_quat_pointer[1]);
//	vision_transmit_buff[15] = GET_LOW_BYTE0(*imu_quat_pointer[2]);
//	vision_transmit_buff[16] = GET_LOW_BYTE1(*imu_quat_pointer[2]);
//	vision_transmit_buff[17] = GET_LOW_BYTE2(*imu_quat_pointer[2]);
//	vision_transmit_buff[18] = GET_LOW_BYTE3(*imu_quat_pointer[2]);
//	vision_transmit_buff[19] = GET_LOW_BYTE0(*imu_quat_pointer[3]);
//	vision_transmit_buff[20] = GET_LOW_BYTE1(*imu_quat_pointer[3]);
//	vision_transmit_buff[21] = GET_LOW_BYTE2(*imu_quat_pointer[3]);
//	vision_transmit_buff[22] = GET_LOW_BYTE3(*imu_quat_pointer[3]);
//#endif
//#define _VISION_TRAN_YAW_PITCH_				//传输顺序 yaw pitch roll;		
//#if  defined(_VISION_TRAN_YAW_PITCH_)&&!defined(_VISION_TRAN_QUAT_)
//	//yaw
//	vision_transmit_buff[2] = GET_LOW_BYTE0(*imu_angle_pointer[0]);
//	vision_transmit_buff[3] = GET_LOW_BYTE1(*imu_angle_pointer[0]);
//	vision_transmit_buff[4] = GET_LOW_BYTE2(*imu_angle_pointer[0]);
//	vision_transmit_buff[5] = GET_LOW_BYTE3(*imu_angle_pointer[0]);
//    memcpy((&VisionTransmit.yaw.buff[0]),&vision_transmit_buff[2],4);
//	//pitch
//	vision_transmit_buff[6] = GET_LOW_BYTE0(*imu_angle_pointer[1]);
//	vision_transmit_buff[7] = GET_LOW_BYTE1(*imu_angle_pointer[1]);
//	vision_transmit_buff[8] = GET_LOW_BYTE2(*imu_angle_pointer[1]);
//	vision_transmit_buff[9] = GET_LOW_BYTE3(*imu_angle_pointer[1]);
//    memcpy(&(VisionTransmit.pitch.buff[0]),&vision_transmit_buff[6],4);

//#endif
	float transmit_Pitch = -motor.Gimbal_Pitch.Apid.now;
	memcpy(&(vision_transmit_buff[6]),&transmit_Pitch, 4);	//pitch
	memcpy(&(vision_transmit_buff[2]),&Gimbal.yaw_angle, 4);       //yaw
	vision_transmit_buff[31] = 'e';
}





/*-----------------------------------------------------------------*/
	#define	COMPENSATION_YAW	0
	#define	COMPENSATION_PITCH	0
	#define COMPENSATION_PITCH_DIST 0
	float SB_K_comps = 3.f;

//角度初始化补偿
float Vision_Comps_Yaw   = COMPENSATION_YAW;
float Vision_Comps_Pitch = COMPENSATION_PITCH;//固定补偿，减小距离的影响
float Vision_Comps_Pitch_Dist = COMPENSATION_PITCH_DIST;//根据距离补偿



