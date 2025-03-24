#ifndef __DM4310MULTI_H__
#define __DM4310MULTI_H__

#include "main.h"
// 4310����CANID
#define CANID_YAW 0x301
#define CANID_PITCH 0x302
#define CANID_RAMMER 0x303



typedef struct Motor_4310CANType
{
	uint32_t ID;	   // CAN ID
	uint8_t HCAN;	   // CAN1 or CAN2
	int16_t speed_rpm; // ת��ת��
	int16_t target_speed_rpm;
	int16_t current;		// ���ص���
	int16_t target_current; // �趨����
	float tempure;			// �¶�
	int16_t angle;			// ��е�Ƕ�
	int16_t target_angle;	// Ŀ��Ƕ� ���̵��һ���ٶȵ���
	uint32_t Rx_add;
	uint16_t RX_Frequancy; // ����Ƶ��
} Motor_4310CANType;
extern Motor_4310CANType Motor_4310_yaw;
extern Motor_4310CANType Motor_4310_pitch;
extern Motor_4310CANType Motor_4310_ram;
void Motor_4310MULTI_receive(Motor_4310CANType *motor, uint8_t *temp, uint8_t CAN_ID);

void Motor_4310multi_send(CAN_HandleTypeDef *hcan, uint32_t StdID, int16_t Motor1, int16_t Motor2, int16_t Motor3, int16_t Motor4);


// DM4310һ���ķ���֡
//  ��ʶ��		���ݶ�	 ����				˵��
// 0x300+���ID	D[0]	�����ǰλ�ø�8λ	��Χ��0-8191����ӦһȦλ��
//  ��			D[1]	�����ǰλ�õ�8λ
//  ��			D[2]	�����ǰ�ٶȸ�8λ	��λ:rpm,�Ŵ�һ�ٱ�
//  ��			D[3]	�����ǰ�ٶȵ�8λ
//  ��			D[4]	Ť�ص�����8λ		��λ:mA
//  ��			D[5]	Ť�ص�����8λ
//  ��			D[6]	�����Ȧ�¶�		��λ����
//  ��			D[7]	����״̬			�������״̬˵���鶨��

// ���ڵ�����е�Ȧ����λ�ñ��������ʵ�����ϵ��������Ȧ����λ�ã������ⳡ��Ҫ���趨������ʱ�����Ҫ����ָ������ɣ���ͨ����������ʵ�֣�
// ����ID	����	D[0]	D[1]	D[2]	D[3]
// 0x7FF	STD	   CANID_L CANID_H 0x55	   0x50
// �趨�ɹ��󣬵�����Զ����浱ǰλ��Ϊ��㣬�������¸�ʽ�������ݣ�
// ����ID	����	 D[0]	 D[1]	  D[2]	 D[3]	|D[4]	D5	D[6]	D[7]|
// MST_ID	STD		CANID_L	CANID_H	 0x55	RID	   |0x00000000			    |

#endif //__DM4310MULTI_H__
