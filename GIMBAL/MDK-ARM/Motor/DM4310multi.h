#ifndef __DM4310MULTI_H__
#define __DM4310MULTI_H__

#include "main.h"
// 4310反馈CANID
#define CANID_YAW 0x301
#define CANID_PITCH 0x302
#define CANID_RAMMER 0x303



typedef struct Motor_4310CANType
{
	uint32_t ID;	   // CAN ID
	uint8_t HCAN;	   // CAN1 or CAN2
	int16_t speed_rpm; // 转子转速
	int16_t target_speed_rpm;
	int16_t current;		// 返回电流
	int16_t target_current; // 设定电流
	float tempure;			// 温度
	int16_t angle;			// 机械角度
	int16_t target_angle;	// 目标角度 底盘电机一般速度单环
	uint32_t Rx_add;
	uint16_t RX_Frequancy; // 接收频率
} Motor_4310CANType;
extern Motor_4310CANType Motor_4310_yaw;
extern Motor_4310CANType Motor_4310_pitch;
extern Motor_4310CANType Motor_4310_ram;
void Motor_4310MULTI_receive(Motor_4310CANType *motor, uint8_t *temp, uint8_t CAN_ID);

void Motor_4310multi_send(CAN_HandleTypeDef *hcan, uint32_t StdID, int16_t Motor1, int16_t Motor2, int16_t Motor3, int16_t Motor4);


// DM4310一拖四反馈帧
//  标识符		数据段	 描述				说明
// 0x300+电机ID	D[0]	电机当前位置高8位	范围：0-8191，对应一圈位置
//  　			D[1]	电机当前位置低8位
//  　			D[2]	电机当前速度高8位	单位:rpm,放大一百倍
//  　			D[3]	电机当前速度低8位
//  　			D[4]	扭矩电流高8位		单位:mA
//  　			D[5]	扭矩电流低8位
//  　			D[6]	电机线圈温度		单位：℃
//  　			D[7]	错误状态			详见错误状态说明书定义

// 由于电机具有单圈绝对位置编码器，故电机在上电后会输出单圈绝对位置，在特殊场合要求设定新零点的时候就需要特殊指令来完成，可通过以下命令实现：
// 报文ID	属性	D[0]	D[1]	D[2]	D[3]
// 0x7FF	STD	   CANID_L CANID_H 0x55	   0x50
// 设定成功后，电机将自动保存当前位置为零点，并按如下格式返回数据：
// 报文ID	属性	 D[0]	 D[1]	  D[2]	 D[3]	|D[4]	D5	D[6]	D[7]|
// MST_ID	STD		CANID_L	CANID_H	 0x55	RID	   |0x00000000			    |

#endif //__DM4310MULTI_H__
