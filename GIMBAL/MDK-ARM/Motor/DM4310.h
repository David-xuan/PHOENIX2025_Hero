#ifndef __DM4310_H__
#define __DM4310_H__


#include "main.h"

#define DM4310_ID 0x11

typedef struct DM4310_TypeDef
{
	uint32_t 	ID;           //CAN ID
	uint8_t 	HCAN;         //CAN1 or CAN2

	int p_int;
    int v_int;
    int t_int;
	
	float	 speed_rpm;    //转子转速
	float	 target_speed_rpm;
    float    angle;
    float    torque;
	float    tempure;      //温度
	int target_angle;
	int target_torque;
	uint16_t dm_err;
	
	uint32_t msg_missing_time;
	uint32_t msg_cnt;
	uint32_t msg_frequent; //接收频率
} DM4310_TypeDef;


extern DM4310_TypeDef DM4310,Motor_4310_Rammer;


int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);

void Motor_DM4310_Enable(CAN_HandleTypeDef* hcan,uint16_t id);
void Motor_DM4310_send(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq);
void position_speed_control(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel);
void Motor_DM4310_receive(DM4310_TypeDef* motor, uint8_t* temp, uint8_t CAN_ID);

#endif
