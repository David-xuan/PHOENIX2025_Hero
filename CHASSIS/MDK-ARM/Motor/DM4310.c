#include "DM4310.h"
#include "bsp_can.h"
#include "can.h"
#include "Filter.h"

// #define P_MIN -3.1415923f
// #define P_MAX 3.1415923f
// #define V_MIN -30.0f
// #define V_MAX 30.0f
// #define KP_MIN 0.0f
// #define KP_MAX 500.0f
// #define KD_MIN 0.0f
// #define KD_MAX 5.0f
// #define T_MIN -10.0f
// #define T_MAX 10.0f
 
 DM4310_TypeDef DM4310,Motor_4310_Rammer;
 


void Motor_DM4310_Enable(CAN_HandleTypeDef* hcan,uint16_t id)
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = id;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = 0xFF;
	Txtemp[1] = 0xFF;
	Txtemp[2] = 0xFF;
	Txtemp[3] = 0xFF;
	Txtemp[4] = 0xFF;
	Txtemp[5] = 0xFF;
	Txtemp[6] = 0xFF;
	Txtemp[7] = 0xFC;
	
	uint8_t count = 0;
	while( HAL_CAN_GetTxMailboxesFreeLevel( &hcan2 ) == 0 && count < 100){
			count++;
	};
		if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{
		
	}
}	

void Motor_DM4310_send(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq)
{
  uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = id;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
	kp_tmp  = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);
	
	Txtemp[0] = (pos_tmp >> 8);
	Txtemp[1] = pos_tmp;
	Txtemp[2] = (vel_tmp >> 4);
	Txtemp[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	Txtemp[4] = kp_tmp;
	Txtemp[5] = (kd_tmp >> 4);
	Txtemp[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	Txtemp[7] = tor_tmp;

	uint8_t count = 0;
	while( HAL_CAN_GetTxMailboxesFreeLevel( &hcan2 ) == 0 && count < 100){
			count++;
	};
		if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{
		
	}
}	
	
extern Motor_struct motor;
void Motor_DM4310_receive(DM4310_TypeDef* Motor, uint8_t* temp, uint8_t CAN_ID)
{


	int p_int;
	int v_int;
	int t_int;
	
	Motor->ID = CAN_ID; 
	p_int=(temp[1]<<8)|temp[2];
	v_int=(temp[3]<<4)|(temp[4]>>4);
	t_int=((temp[4]&0xF)<<8)|temp[5];

	Motor->angle = uint_to_float(p_int, P_MIN, P_MAX, 16);
	Motor->speed_rpm = uint_to_float(v_int, V_MIN, V_MAX, 12);
	Motor->torque= uint_to_float(t_int, T_MIN, T_MAX, 12);	//�������
	Motor->tempure = (float)(temp[6]);
	Motor->dm_err = temp[0] >> 4;

	if(Motor==&DM4310)
    {
//		Motor->speed_rpm = IIR_Calc(Motor->speed_rpm,&ENSPEEDFilter);
		motor.Gimbal_Yaw.Apid.now=Motor->angle;
		motor.Gimbal_Yaw.Vpid.now=Motor->speed_rpm;
	}
	else if(Motor==&Motor_4310_Rammer)
	{
		motor.Rammer.Apid.now=Motor->angle;
		motor.Rammer.Vpid.now=Motor->speed_rpm;
	}
	Motor->msg_cnt++;
	Motor->msg_missing_time=0;

}



//float uint_to_float(int x_int, float x_min, float x_max, int bits)
//{
//  float span = x_max - x_min;
//  float offset = x_min;
//  return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
//}
//    
//    
//int float_to_uint(float x, float x_min, float x_max, int bits)
//{ 
//  float span = x_max - x_min;
//  float offset = x_min;
//  return (int) ((x-offset)*((float)((1<<bits)-1))/span);
//}

		
