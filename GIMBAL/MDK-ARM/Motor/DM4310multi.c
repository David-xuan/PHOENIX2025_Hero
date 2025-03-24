#include "DM4310multi.h"
#include "bsp_can.h"

Motor_4310CANType Motor_4310_yaw;
Motor_4310CANType Motor_4310_pitch;
Motor_4310CANType Motor_4310_ram;

/**
 * @brief 接收并解析4310multi电机的CAN数据
 *
 * 该函数用于处理接收到的CAN数据帧，将数据解析到电机结构体中。它主要解析电机的ID、角度、转速、电流和温度。
 *
 * @param motor 指向电机结构体的指针，用于存储解析后的数据
 * @param temp 指向接收到的CAN数据帧的缓冲区
 * @param CAN_ID CAN数据帧的ID
 */
extern Motor_struct motor;
void Motor_4310MULTI_receive(Motor_4310CANType *Motor, uint8_t *temp, uint8_t CAN_ID)
{
	Motor->ID = CAN_ID;
	Motor->angle = ((uint16_t)temp[0]) << 8 | ((uint16_t)temp[1]);
	Motor->speed_rpm = (int16_t)(temp[2] << 8 | temp[3]);
	Motor->current = (((int16_t)temp[4]) << 8 | ((int16_t)temp[5]));
	Motor->tempure = temp[6];

	motor.Gimbal_Pitch.Apid.now = Motor->angle;
	motor.Gimbal_Pitch.Vpid.now = Motor->speed_rpm;

	if (Motor->Rx_add < 10000)
	{
		Motor->Rx_add++;
	}
	else
	{
	}
}

// DM4310一拖四控制帧
//  标识符	数据段	 描述	        电机ID
//  0x3FE  	D[0]	控制电流低8位	1
//  　	    D[1]	控制电流高8位
//  　		D[2]	控制电流低8位	2
//  　		D[3]	控制电流高8位
//  　		D[4]	控制电流低8位	3
//  　		D[5]	控制电流高8位
//  　		D[6]	控制电流低8位	4
//  　		D[7]	控制电流高8位
//  0x4FE	D[0]	控制电流低8位	5
//  　		D[1]	控制电流高8位
//  　		D[2]	控制电流低8位	6
//  　		D[3]	控制电流高8位
//  　		D[4]	控制电流低8位	7
//  　		D[5]	控制电流高8位
//  　		D[6]	控制电流低8位	8
//  　		D[7]	控制电流高8位
/**
 * @brief 通过CAN总线发送电机控制指令
 *
 * 本函数负责组装CAN报文并发送给电机驱动器，以控制最多四个电机的运行
 * 它将每个电机的控制指令拆分成两个字节，然后将这些字节填充到CAN报文的数据字段中
 *
 * @param hcan 指向CAN外设的句柄
 * @param StdID CAN报文的标准标识符
 * @param Motor1 第一个电机的控制指令，有符号16位整数
 * @param Motor2 第二个电机的控制指令，有符号16位整数
 * @param Motor3 第三个电机的控制指令，有符号16位整数
 * @param Motor4 第四个电机的控制指令，有符号16位整数
 */
void Motor_4310multi_send(CAN_HandleTypeDef *hcan, uint32_t StdID, int16_t Motor1, int16_t Motor2, int16_t Motor3, int16_t Motor4)
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = StdID;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = (Motor1 >> 8);
	Txtemp[1] = Motor1;
	Txtemp[2] = (Motor2 >> 8);
	Txtemp[3] = Motor2;
	Txtemp[4] = Motor3 >> 8;
	Txtemp[5] = Motor3;
	Txtemp[6] = Motor4 >> 8;
	Txtemp[7] = Motor4;

	while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)
		;
	HAL_CAN_AddTxMessage(hcan, &_TxHeader, Txtemp, (uint32_t *)CAN_TX_MAILBOX0);
}
