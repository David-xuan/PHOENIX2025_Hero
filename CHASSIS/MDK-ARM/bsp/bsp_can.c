#include "bsp_can.h"
#include "cmsis_os.h" 
#include "can.h"
#include "DM4310.h"
#include "remote_control.h"
#include "Gimbal_Task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern RC_ctrl_t rc_ctrl;
angle_measure_t Yaw_angle_t;
extern Motor_struct motor;
void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&hcan1);

    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

}

void can_filter_init_recv_all(CAN_HandleTypeDef* _hcan)
{
	//can1 &can2 use same filter config
	CAN_FilterTypeDef		sFilterConfig;

   sFilterConfig.FilterIdHigh         = 0x0000;
   sFilterConfig.FilterIdLow          = 0x0000;
   sFilterConfig.FilterMaskIdHigh     = 0x0000;
   sFilterConfig.FilterMaskIdLow      = 0x0000;
   sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
   sFilterConfig.FilterBank=0;
   sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
   sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;
   sFilterConfig.FilterActivation = ENABLE;
   sFilterConfig.SlaveStartFilterBank = 0;
	if(HAL_CAN_ConfigFilter(_hcan, &sFilterConfig) != HAL_OK)
	{

	}
}

void TransTxMessage(set_t *set, uint8_t *Txdata)//要发送的数据存入数组
{
	Txdata[0] = set->motor1 >> 8;
	Txdata[1] = set->motor1;
	Txdata[2] = set->motor2 >> 8;
	Txdata[3] = set->motor2;
	Txdata[4] = set->motor3 >> 8;
	Txdata[5] = set->motor3;
	Txdata[6] = set->motor4 >> 8;
	Txdata[7] = set->motor4;
	
	
}

void TransRxMessage(fdb_t *fdb, uint8_t *Rxdata)//接收到的数据存入反馈值
{
	fdb->angle = Rxdata[0] << 8 | Rxdata[1];
	fdb->speed = Rxdata[2] << 8 | Rxdata[3];
	fdb->current = Rxdata[4] << 8 | Rxdata[5];
	fdb->temperature = Rxdata[6];
	fdb->null = Rxdata[7];
}

void CAN_Tx(CAN_HandleTypeDef *hcan, uint32_t *stdid, set_t *set)//can发送
{
	uint8_t Txdata[8];
	TransTxMessage(set, Txdata);
	CAN_TxHeaderTypeDef Txheader;
	Txheader.StdId = *stdid;
	Txheader.IDE = CAN_ID_STD;
	Txheader.RTR = CAN_RTR_DATA;
	Txheader.DLC = 8;
	HAL_CAN_AddTxMessage(hcan, &Txheader, Txdata, (uint32_t*)CAN_TX_MAILBOX0);
}

void CAN_Rx(CAN_HandleTypeDef *hcan, uint32_t *stdid, fdb_t *fdb)//can接收
{
	uint8_t Rxdata[8];
	CAN_RxHeaderTypeDef Rxheader;
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Rxheader, Rxdata);
	if 	(Rxheader.IDE == CAN_ID_STD && Rxheader.RTR == CAN_RTR_DATA && Rxheader.DLC == 8)
	{
		*stdid = Rxheader.StdId;
		TransRxMessage(fdb, Rxdata);
	}
}

void Set_Transmit(CAN_HandleTypeDef *hcan, Motor_struct *motor)//发送计算后的设定值
{
	set_t SET;
	uint32_t stdid = 0x200;
	SET.motor1 =(int16_t)motor->Motor[0].Vpid.out;//(int16_t)motor->Motor[0].Vpid.out;
	SET.motor2 =(int16_t)motor->Motor[1].Vpid.out;//(int16_t)motor->Motor[1].Vpid.out;
	SET.motor3 =(int16_t)motor->Motor[2].Vpid.out;//(int16_t)motor->Motor[2].Vpid.out;
	SET.motor4 =(int16_t)motor->Motor[3].Vpid.out;//(int16_t)motor->Motor[3].Vpid.out;
	CAN_Tx(hcan, &stdid, &SET);
}

void Get_Fdb(CAN_HandleTypeDef *hcan, Motor_struct *motor)//反馈值存到.now
{
	fdb_t FDB;
	uint32_t stdid;
	CAN_Rx(hcan, &stdid, &FDB);
	switch(stdid)
	{
		
		case 0x201:
		{
			motor->Motor[0].Vpid.now = (float)FDB.speed;
			motor->Motor[0].Apid.now = (float)FDB.angle*360.f/8191.f;
			break;
		}
		case 0x202:
	    {
			motor->Motor[1].Vpid.now = (float)FDB.speed;
			motor->Motor[1].Apid.now = (float)FDB.angle*360.f/8191.f;
			break;
		}
		case 0x203:
	    {
			motor->Motor[2].Vpid.now = (float)FDB.speed;
			motor->Motor[2].Apid.now = (float)FDB.angle*360.f/8191.f;
			break;
		}
		case 0x204:
	    {
			motor->Motor[3].Vpid.now = (float)FDB.speed;
			motor->Motor[3].Apid.now = (float)FDB.angle*360.f/8191.f;
			break;
		}
		
		case 0x208:
	    {
			motor->Gimbal_Yaw.Vpid.now = (float)FDB.speed;
			motor->Gimbal_Yaw.Apid.now = (float)FDB.angle*360.f/8191.f;
			break;
		}

		default:break;
	}
}
//DM4310
void DM4310_Deactivate(CAN_HandleTypeDef *hcan)
{
	uint8_t Txdata[8];
	CAN_TxHeaderTypeDef Txheader;
	Txheader.StdId = 0x01;
	Txheader.IDE = CAN_ID_STD;
	Txheader.RTR = CAN_RTR_DATA;
	Txheader.DLC = 8;
	Txdata[0] = 0xFF;
	Txdata[1] = 0xFF;
	Txdata[2] = 0xFF;
	Txdata[3] = 0xFF;
	Txdata[4] = 0xFF;
	Txdata[5] = 0xFF;
	Txdata[6] = 0xFF;
	Txdata[7] = 0xFD;

	  if(HAL_CAN_AddTxMessage(hcan, &Txheader, Txdata, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	  {
		if(HAL_CAN_AddTxMessage(hcan, &Txheader, Txdata, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan, &Txheader, Txdata, (uint32_t*)CAN_TX_MAILBOX2);
    }
    }
}
/*--------------------------------------------板间通信-----------------------------------------------------------------------------*/
void set_rc_ch(CAN_HandleTypeDef* hcan, int16_t ch1, int16_t ch2, int16_t ch3, int16_t ch4)
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = 0x320;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = (ch1 >> 8);
	Txtemp[1] = ch1;
	Txtemp[2] = (ch2 >> 8);
	Txtemp[3] = ch2;
	Txtemp[4] = ch3 >> 8;
	Txtemp[5] = ch3;
	Txtemp[6] = ch4 >> 8;
	Txtemp[7] = ch4;
	while( HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0 );
	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{
		
	}

}

void set_rc_sw(CAN_HandleTypeDef* hcan, int8_t sw1, int8_t sw2, int16_t v, int16_t wheel)
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
//	int8_t color;
	_TxHeader.StdId = 0x321;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	
//	color = is_red_or_blue(); //判断颜色

	Txtemp[0] = sw1;
	Txtemp[1] = sw2;
	Txtemp[2] = (rc_ctrl.key.v >> 8);
	Txtemp[3] =rc_ctrl.key.v ;
	Txtemp[4] = wheel >> 8;
	Txtemp[5] = wheel;
	Txtemp[6] = 1;//color
	Txtemp[7] = 0;//Chassis.flag.remotelose_flag;
	while( HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0 );
	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{
		
	}

}

void set_rc_mouse(CAN_HandleTypeDef* hcan, int16_t x, int16_t y, int16_t z, int8_t l, int8_t r)
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = 0x322;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = (x >> 8);
	Txtemp[1] = x;
	Txtemp[2] = (y >> 8);
	Txtemp[3] = y;
	Txtemp[4] = z >> 8;
	Txtemp[5] = z;
	Txtemp[6] = l;
	Txtemp[7] = r;
	while( HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0 );
	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{
		
	}

}

void set_yaw_angle(CAN_HandleTypeDef* hcan, float angle)
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[4];
	_TxHeader.StdId = 0x102;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;

	Yaw_angle_t.Yaw_4310.value = angle;
	Txtemp[0] = Yaw_angle_t.Yaw_4310.buff[0];
	Txtemp[1] = Yaw_angle_t.Yaw_4310.buff[1];
	Txtemp[2] = Yaw_angle_t.Yaw_4310.buff[2];
	Txtemp[3] = Yaw_angle_t.Yaw_4310.buff[3];
	
	uint32_t TxMailboxX = CAN_TX_MAILBOX0;
    while(HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0);      //如果三个邮箱都阻塞了就等一会儿，直到其中某个邮箱空闲
    if ((hcan->Instance->TSR & CAN_TSR_TME0) != RESET)     //如果邮箱0空闲
    {
        TxMailboxX =CAN_TX_MAILBOX0;
    }
    /* Check Tx Mailbox 1 status */
    else if ((hcan->Instance->TSR & CAN_TSR_TME1) != RESET)
    {
        TxMailboxX =CAN_TX_MAILBOX1;
    }
    /* Check Tx Mailbox 2 status */
    else if ((hcan->Instance->TSR & CAN_TSR_TME2) != RESET)
    {
        TxMailboxX =CAN_TX_MAILBOX2;
    }
	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)TxMailboxX)!=HAL_OK)
	{
		Error_Handler();
	}
}

void set_chassis_msg(void)
{
	set_rc_ch(&hcan2, rc_ctrl.rc.ch[0], rc_ctrl.rc.ch[1], rc_ctrl.rc.ch[2], rc_ctrl.rc.ch[3]);
	set_rc_sw(&hcan2, rc_ctrl.rc.s[0], rc_ctrl.rc.s[1], rc_ctrl.key.v, rc_ctrl.rc.wheel);	
	set_rc_mouse(&hcan2, rc_ctrl.mouse.x, rc_ctrl.mouse.y, rc_ctrl.mouse.x, rc_ctrl.mouse.press_l, rc_ctrl.mouse.press_r);
	set_yaw_angle(&hcan2, motor.Gimbal_Yaw.Apid.now);
}

void gimbal_msg_receive(angle_measure_t* gim_msg,uint8_t temp[])
{
		Gimbal.IMUData.find_bool = temp[0];
		gim_msg->vision_value.buff[0]=temp[4];
		gim_msg->vision_value.buff[1]=temp[5];
		gim_msg->vision_value.buff[2]=temp[6];
		gim_msg->vision_value.buff[3]=temp[7];	
		
		Gimbal.IMUData.Vision_yaw_target = gim_msg->vision_value.value;

		if(Gimbal.IMUData.Rx_add<10000)
			Gimbal.IMUData.Rx_add++;
//		Gimbal.flag.boardlose_time = 0;
}	

void gimbal_flag_receive(angle_measure_t* gim_msg,uint8_t temp[])
{

		gim_msg->Pitch_angle.buff[0] = temp[0];
		gim_msg->Pitch_angle.buff[1] = temp[1];
		gim_msg->Pitch_angle.buff[2] = temp[2];
		gim_msg->Pitch_angle.buff[3] = temp[3];
		gim_msg->Yaw_angle.buff[0]=temp[4];
		gim_msg->Yaw_angle.buff[1]=temp[5];
		gim_msg->Yaw_angle.buff[2]=temp[6];
		gim_msg->Yaw_angle.buff[3]=temp[7];
	
		Gimbal.IMUData.IMU_Yaw_angle = gim_msg->Yaw_angle.value;
	
		if(Gimbal.IMUData.Rx_add_2<10000)
			Gimbal.IMUData.Rx_add_2++;
}	
uint8_t temp_test[8];
void Chassis_can_send(void)
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = 0x101;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = 0x01;
	Txtemp[1] = (rc_ctrl.rc.ch[0]>>8);
	Txtemp[2] = rc_ctrl.rc.ch[0];
	Txtemp[3] = rc_ctrl.rc.ch[1]>>8;
	Txtemp[4] = rc_ctrl.rc.ch[1];
	Txtemp[5] = rc_ctrl.mouse.x >>8;
	Txtemp[6] = rc_ctrl.mouse.x;
	Txtemp[7] =	rc_ctrl.rc.s[0];
	temp_test[0] = Txtemp[5];
	temp_test[1] = Txtemp[6];
	
	while( HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0 );
	if(HAL_CAN_AddTxMessage(&hcan2,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK){}
	

	Txtemp[0] = 0x02;
	Txtemp[1] = rc_ctrl.rc.ch[2]>>8;
	Txtemp[2] = rc_ctrl.rc.ch[2];
	Txtemp[3] = rc_ctrl.rc.ch[3]>>8;
	Txtemp[4] = rc_ctrl.rc.ch[3];
	Txtemp[5] =	rc_ctrl.mouse.y>>8;
	Txtemp[6] = rc_ctrl.mouse.y;
	Txtemp[7] =	rc_ctrl.rc.s[1];
		
	while( HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0 );
	if(HAL_CAN_AddTxMessage(&hcan2,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK){}
		
//	uint8_t color;
//	color = is_red_or_blue(); //判断颜色	
		
	Txtemp[0] = 0x03;
	Txtemp[1] =	rc_ctrl.mouse.press_l;
	Txtemp[2] = rc_ctrl.mouse.press_r;
	Txtemp[3] = rc_ctrl.key.v>>8;
	Txtemp[4] = rc_ctrl.key.v;
	Txtemp[5] = 0;//color;
	Txtemp[6] = 0;//Chassis.flag.remotelose_flag;

	while( HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0 );
	if(HAL_CAN_AddTxMessage(&hcan2,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK){}
		
}

/*----------------------------------------------------end--------------------------------------------------------------*/

float Hex_To_Float(uint32_t *Byte)
{
	return *((float*)Byte);
}

uint32_t FloatTohex(float HEX)
{
	return *( uint32_t *)&HEX;
}

int float_to_uint(float x, float x_min, float x_max, int bits){
 /// Converts a float to an unsigned int, given range and number of bits///
 float span = x_max - x_min;
 float offset = x_min;
 return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* _hcan)
{
	CAN_RxHeaderTypeDef _RxHeader;
	uint8_t temp[8];
	fdb_t FDB;
	
	HAL_CAN_GetRxMessage(_hcan,CAN_RX_FIFO0,&_RxHeader,temp);
	FDB.angle = temp[0] << 8 | temp[1];
	FDB.speed = temp[2] << 8 | temp[3];
	FDB.current = temp[4] << 8 | temp[5];
	//ignore can1 or can2.
	switch(_RxHeader.StdId)
		{
			case 0x201:
			case 0x202:
			case 0x203:
			case 0x204:
			{
				uint8_t i;
				i = _RxHeader.StdId - 0x201;
				motor.Motor[i].Vpid.now = (float)FDB.speed;
				motor.Motor[i].Apid.now = (float)FDB.angle*360.f/8191.f;
			}
			break;
			case DM4310_ID:
			Motor_DM4310_receive(&DM4310,temp,DM4310_ID);
			break;
			case DM4310_RAMMER_ID:
			Motor_DM4310_receive(&Motor_4310_Rammer,temp,DM4310_RAMMER_ID);
			break;
			case CANID_IMU:
			gimbal_msg_receive(&Yaw_angle_t,temp);
			break;
			case CANID_FLAG:
			gimbal_flag_receive(&Yaw_angle_t,temp);
			break;

		}

	/*#### add enable can it again to solve can receive only one ID problem!!!####**/

	  __HAL_CAN_ENABLE_IT(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void Frequency_calc(void)
{

	Gimbal.IMUData.Rx_Frequency = Gimbal.IMUData.Rx_add++;
	Gimbal.IMUData.Rx_add = 0;
	
	Gimbal.IMUData.Rx_Frequency_2 = Gimbal.IMUData.Rx_add_2++;
	Gimbal.IMUData.Rx_add_2 = 0;

	
	

}
