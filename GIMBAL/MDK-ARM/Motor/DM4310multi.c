#include "DM4310multi.h"
#include "bsp_can.h"

Motor_4310CANType Motor_4310_yaw;
Motor_4310CANType Motor_4310_pitch;
Motor_4310CANType Motor_4310_ram;

/**
 * @brief ���ղ�����4310multi�����CAN����
 *
 * �ú������ڴ�����յ���CAN����֡�������ݽ���������ṹ���С�����Ҫ���������ID���Ƕȡ�ת�١��������¶ȡ�
 *
 * @param motor ָ�����ṹ���ָ�룬���ڴ洢�����������
 * @param temp ָ����յ���CAN����֡�Ļ�����
 * @param CAN_ID CAN����֡��ID
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

// DM4310һ���Ŀ���֡
//  ��ʶ��	���ݶ�	 ����	        ���ID
//  0x3FE  	D[0]	���Ƶ�����8λ	1
//  ��	    D[1]	���Ƶ�����8λ
//  ��		D[2]	���Ƶ�����8λ	2
//  ��		D[3]	���Ƶ�����8λ
//  ��		D[4]	���Ƶ�����8λ	3
//  ��		D[5]	���Ƶ�����8λ
//  ��		D[6]	���Ƶ�����8λ	4
//  ��		D[7]	���Ƶ�����8λ
//  0x4FE	D[0]	���Ƶ�����8λ	5
//  ��		D[1]	���Ƶ�����8λ
//  ��		D[2]	���Ƶ�����8λ	6
//  ��		D[3]	���Ƶ�����8λ
//  ��		D[4]	���Ƶ�����8λ	7
//  ��		D[5]	���Ƶ�����8λ
//  ��		D[6]	���Ƶ�����8λ	8
//  ��		D[7]	���Ƶ�����8λ
/**
 * @brief ͨ��CAN���߷��͵������ָ��
 *
 * ������������װCAN���Ĳ����͸�������������Կ�������ĸ����������
 * ����ÿ������Ŀ���ָ���ֳ������ֽڣ�Ȼ����Щ�ֽ���䵽CAN���ĵ������ֶ���
 *
 * @param hcan ָ��CAN����ľ��
 * @param StdID CAN���ĵı�׼��ʶ��
 * @param Motor1 ��һ������Ŀ���ָ��з���16λ����
 * @param Motor2 �ڶ�������Ŀ���ָ��з���16λ����
 * @param Motor3 ����������Ŀ���ָ��з���16λ����
 * @param Motor4 ���ĸ�����Ŀ���ָ��з���16λ����
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
