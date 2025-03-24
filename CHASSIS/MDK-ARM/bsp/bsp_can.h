#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "stm32f4xx.h"
#include "pid.h"

#define P_MIN -3.141593
#define P_MAX 3.141593
#define V_MIN -30.0
#define V_MAX 30.0
#define KP_MIN 0.0
#define KP_MAX 500.0
#define KD_MIN 0.0
#define KD_MAX 5.0
#define T_MIN -10.0
#define T_MAX 10.0
#define PI               3.14159265358979f
#define DM4310_ID    0x05
#define CANID_IMU 	0x100
#define CANID_FLAG  0x103


typedef struct
{
	int16_t motor1;
	int16_t motor2;
	int16_t motor3;
	int16_t motor4;
}set_t;
//设定值


typedef struct
{
	uint16_t angle;
	int16_t speed;
	int16_t current;
	uint8_t temperature;
	uint8_t null;
}fdb_t;
//反馈值

typedef struct
{
	float p_des;
	float v_des;
	float Kp;
	float Kd;
	float t_ff;
}DM_set;

typedef struct
{
	uint16_t p_des;
	uint16_t v_des;
	uint16_t Kp;
	uint16_t Kd;
	uint16_t t_ff;
}DM_trans;

typedef struct
{
	uint8_t ID;
	uint8_t ERR;
	float Pos;
	float Vel;
	float T;
	float T_Mos;
	float T_Rotor;
}DM_fdb;

typedef struct
{
    union
    {
        int8_t buff[4];
        float value;
    }Yaw_angle;
	
	union
    {
        int8_t buff[4];
        float value;
    }vision_value;
	
	union
	{
		uint8_t buff[4];
		float value;
	}Yaw_4310;
	
	union
	{
		int8_t buff[4];
		float value;
	}Pitch_angle;//陀螺仪pitch值
}angle_measure_t;
extern angle_measure_t Yaw_angle_t;

void TransTxMessage(set_t *set, uint8_t *Txdata);//发送数据存入数组
void TransRxMessage(fdb_t *fdb, uint8_t *Rxdata);//接收数据存入变量
void CAN_Tx(CAN_HandleTypeDef *hcan, uint32_t *stdid, set_t *set);//发送设定值
void CAN_Rx(CAN_HandleTypeDef *hcan, uint32_t *stdid, fdb_t *fdb);//接收反馈值
void Set_Transmit(CAN_HandleTypeDef *hcan, Motor_struct *motor);//把pid输出值发送
void Set_Transmit_Gimbal(CAN_HandleTypeDef *hcan, Motor_struct *motor);//把pid输出值发送
void Get_Fdb(CAN_HandleTypeDef *hcan, Motor_struct *motor);//反馈值存到.now

void can_filter_init(void);
void can_filter_init_recv_all(CAN_HandleTypeDef* _hcan);

//DM
void DM4310_Deactivate(CAN_HandleTypeDef *hcan);

float Hex_To_Float(uint32_t *Byte);
uint32_t FloatTohex(float HEX);
int float_to_uint(float x_float, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);

void set_chassis_msg(void);
void set_rc_ch(CAN_HandleTypeDef* hcan, int16_t ch1, int16_t ch2, int16_t ch3, int16_t ch4);
void set_rc_sw(CAN_HandleTypeDef* hcan, int8_t sw1, int8_t sw2, int16_t v, int16_t wheel);
void set_rc_mouse(CAN_HandleTypeDef* hcan, int16_t x, int16_t y, int16_t z, int8_t l, int8_t r);
void set_yaw_angle(CAN_HandleTypeDef* hcan, float angle);

void Chassis_can_send(void);

void Frequency_calc(void);

#endif
