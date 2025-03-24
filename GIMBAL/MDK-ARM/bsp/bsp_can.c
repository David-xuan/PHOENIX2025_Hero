#include "bsp_can.h"
#include "remote_control.h"
#include "DM4310multi.h"
#include "DM4310.h"
#include "Gimbal_task.h"
#include "vision.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern RC_ctrl_t rc_ctrl;
extern Motor_struct motor;
angle_measure_t Yaw_angle_t;

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
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

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
	SET.motor1 =(int16_t)motor->Motor1.Vpid.out;
	SET.motor2 =(int16_t)motor->Motor2.Vpid.out;
	SET.motor3 =(int16_t)motor->Motor3.Vpid.out;
	SET.motor4 =(int16_t)motor->Motor4.Vpid.out;
	CAN_Tx(hcan, &stdid, &SET);
}

void Set_Transmit_Gimbal(CAN_HandleTypeDef *hcan, Motor_struct *motor)
{
	set_t SET;
	uint32_t stdid = 0x1FF;
	SET.motor1 =(int16_t)motor->Gimbal_Pitch.Vpid.out;
//	SET.motor4 =(int16_t)motor->Gimbal_Yaw.Vpid.out;
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
			motor->Motor1.Ipid.now = (float)FDB.current*5.f/16384.f;
			motor->Motor1.Vpid.now = (float)FDB.speed;
			motor->Motor1.Apid.now = (float)FDB.angle*360.f/8191.f;
			break;
		}
		case 0x202:
	    {
			motor->Motor2.Ipid.now = (float)FDB.current*5.f/16384.f;
			motor->Motor2.Vpid.now = (float)FDB.speed;
			motor->Motor2.Apid.now = (float)FDB.angle*360.f/8191.f;
			break;
		}
		case 0x203:
	    {
			motor->Motor3.Ipid.now = (float)FDB.current*5.f/16384.f;
			motor->Motor3.Vpid.now = (float)FDB.speed;
			motor->Motor3.Apid.now = (float)FDB.angle*360.f/8191.f;
			break;
		}
		case 0x204:
	    {
			motor->Motor4.Ipid.now = (float)FDB.current*5.f/16384.f;
			motor->Motor4.Vpid.now = (float)FDB.speed;
			motor->Motor4.Apid.now = (float)FDB.angle*360.f/8191.f;
			break;
		}
		
		case 0x205:
	    {
			motor->Gimbal_Pitch.Ipid.now = (float)FDB.current*5.f/16384.f;;
			motor->Gimbal_Pitch.Vpid.now = (float)FDB.speed;
			motor->Gimbal_Pitch.Apid.now = (float)FDB.angle*360.f/8191.f;
			break;
		}	

	}
}
/*--------------------------------------------板间通信-----------------------------------------------------------------------------*/
void dch_receive(RC_ctrl_t* _rc, uint8_t temp[])
{
	_rc->rc.ch[0] = (int16_t)(temp[0]<<8 | temp[1]);
	_rc->rc.ch[1] = (int16_t)(temp[2]<<8 | temp[3]);
	_rc->rc.ch[2] = (int16_t)(temp[4]<<8 | temp[5]);
	_rc->rc.ch[3] = (int16_t)(temp[6]<<8 | temp[7]);
	if(rc_ctrl.Rx_add[0]<10000)
	rc_ctrl.Rx_add[0]++;
}



void dmouse_receive(RC_ctrl_t* _rc, uint8_t temp[])
{
	_rc->mouse.x = (int16_t)(temp[0]<<8 | temp[1]);
	_rc->mouse.y = (int16_t)(temp[2]<<8 | temp[3]);

	_rc->mouse.z = (int16_t)(temp[4]<<8 | temp[5]);
	_rc->mouse.press_l = temp[6];
	_rc->mouse.press_r = temp[7];
	if(rc_ctrl.Rx_add[1]<10000)
	rc_ctrl.Rx_add[1]++;
}

void dsw_receive(RC_ctrl_t* _rc, uint8_t temp[])
{
	_rc->rc.s[0] = temp[0];
	_rc->rc.s[1] = temp[1];
	_rc->key.v = (int16_t)(temp[2]<<8|temp[3]);
	_rc->rc.wheel = (int16_t)(temp[4]<<8 | temp[5]);
	Gimbal.Robot_color = temp[6];
//	Gimbal.flag.remotelose_flag = temp[7];
	if(rc_ctrl.Rx_add[2]<10000)	
	rc_ctrl.Rx_add[2]++;
}
void yaw_msg_receive(angle_measure_t* yaw_msg,uint8_t temp[])
{
	yaw_msg->Yaw_4310.buff[0]=temp[0];
	yaw_msg->Yaw_4310.buff[1]=temp[1];
	yaw_msg->Yaw_4310.buff[2]=temp[2];
	yaw_msg->Yaw_4310.buff[3]=temp[3];

	Gimbal.yaw_angle = yaw_msg->Yaw_4310.value;
}
void set_gimbal_angle(CAN_HandleTypeDef* hcan, float angle)
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = 0x100;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Yaw_angle_t.Yaw_angle.value = angle;
	Yaw_angle_t.vison_yaw.value = VisionValue.yaw_value.value;
	Txtemp[0] = VisionValue.find_bool;
	Txtemp[1] = 0;//Yaw_angle_t.Yaw_angle.buff[1];
	Txtemp[2] = 0;//Yaw_angle_t.Yaw_angle.buff[2];
	Txtemp[3] = 0;//Yaw_angle_t.Yaw_angle.buff[3];

	Txtemp[4] = Yaw_angle_t.vison_yaw.buff[0];
	Txtemp[5] = Yaw_angle_t.vison_yaw.buff[1];
	Txtemp[6] = Yaw_angle_t.vison_yaw.buff[2];
	Txtemp[7] = Yaw_angle_t.vison_yaw.buff[3];


	
	while( HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0 );
	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{
	}
}

void set_gimbal_flag(CAN_HandleTypeDef* hcan,float angle1,float angle2)
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = 0x103;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Yaw_angle_t.Pitch_angle.value = angle1;
	Yaw_angle_t.Yaw_angle.value = angle2;
	Txtemp[0] = Yaw_angle_t.Pitch_angle.buff[0];
	Txtemp[1] = Yaw_angle_t.Pitch_angle.buff[1];//Gimbal.IMUData.pitch_ok;
	Txtemp[2] = Yaw_angle_t.Pitch_angle.buff[2];
	Txtemp[3] = Yaw_angle_t.Pitch_angle.buff[3];//VisionValue.distance_t.buff[1];
	Txtemp[4] = Yaw_angle_t.Yaw_angle.buff[0];//VisionValue.distance_t.buff[2]; 
	Txtemp[5] = Yaw_angle_t.Yaw_angle.buff[1];//VisionValue.distance_t.buff[3];
	Txtemp[6] = Yaw_angle_t.Yaw_angle.buff[2];
	Txtemp[7] = Yaw_angle_t.Yaw_angle.buff[3];
	
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

void Chassis_can_recevie(uint8_t temp[] , RC_ctrl_t* _rc ,Gimbal_t* gimbal,angle_measure_t* yaw_msg)
{
	switch(temp[0])
	{
		case 0x01:
		{
			_rc->rc.ch[0]  = (int16_t)(temp[1]<<8 | temp[2]);
			_rc->rc.ch[1]  = (int16_t)(temp[3]<<8 | temp[4]);		
			_rc->mouse.x = (int16_t)(temp[5]<<8 | temp[6]);
			_rc->rc.s[0]   =  temp[7];
			if(_rc->Rx_add[0]<10000)
				_rc->Rx_add[0]++;
		}
			break;
		case 0x02:
		{
			_rc->rc.ch[2]  = (int16_t)(temp[1]<<8 | temp[2]);
			_rc->rc.ch[3]  = (int16_t)(temp[3]<<8 | temp[4]);		
		    _rc->mouse.y = (int16_t)(temp[5]<<8 | temp[6]);
			_rc->rc.s[1]   =  temp[7];
				if(_rc->Rx_add[1]<10000)
				_rc->Rx_add[1]++;
		}
			break;
		case 0x03:
		{
			_rc->mouse.press_l = temp[1];
			_rc->mouse.press_r = temp[2];
			_rc->key.v = (int16_t)(temp[3]<<8 | temp[4]);
			Gimbal.Robot_color = temp[5];
//			Gimbal.flag.remotelose_flag = temp[6];
				if(_rc->Rx_add[2]<10000)
				_rc->Rx_add[2]++;			
		}
			break;
//		case 0x04:
//		{
//			yaw_msg->Yaw_4310.buff[0]=temp[1];
//			yaw_msg->Yaw_4310.buff[1]=temp[2];
//			yaw_msg->Yaw_4310.buff[2]=temp[3];
//			yaw_msg->Yaw_4310.buff[3]=temp[4];

//			Gimbal.yaw_angle = yaw_msg->Yaw_4310.value;
//		}
//			break;
		default:
			break;
	}
}

/*--------------------------------------------end-----------------------------------------------------------------------------*/

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef _RxHeader;
	uint8_t temp[8];
	HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&_RxHeader,temp);
	fdb_t FDB;
	
	HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&_RxHeader,temp);
	FDB.angle = temp[0] << 8 | temp[1];
	FDB.speed = temp[2] << 8 | temp[3];
	FDB.current = temp[4] << 8 | temp[5];


	//ignore can1 or can2.
	switch(_RxHeader.StdId)
	{
		case 0x101:
		Chassis_can_recevie(temp,&rc_ctrl,&Gimbal,&Yaw_angle_t);
		break;
		case DM4310_ID:
		Motor_DM4310_receive(&DM4310,temp,_RxHeader.StdId);
		break;
		case 0x201:
		{
			motor.Motor1.Ipid.now = (float)FDB.current*5.f/16384.f;
			motor.Motor1.Vpid.now = (float)FDB.speed;
			motor.Motor1.Apid.now = (float)FDB.angle*360.f/8191.f;
			break;
		}
		case 0x202:
	    {
			motor.Motor2.Ipid.now = (float)FDB.current*5.f/16384.f;
			motor.Motor2.Vpid.now = (float)FDB.speed;
			motor.Motor2.Apid.now = (float)FDB.angle*360.f/8191.f;
			break;
		}
		case 0x203:
	    {
			motor.Motor3.Ipid.now = (float)FDB.current*5.f/16384.f;
			motor.Motor3.Vpid.now = (float)FDB.speed;
			motor.Motor3.Apid.now = (float)FDB.angle*360.f/8191.f;
			break;
		}
		case 0x204:
	    {
			motor.Motor4.Ipid.now = (float)FDB.current*5.f/16384.f;
			motor.Motor4.Vpid.now = (float)FDB.speed;
			motor.Motor4.Apid.now = (float)FDB.angle*360.f/8191.f;
			break;
		}
		case CANID_yaw:
			 yaw_msg_receive(&Yaw_angle_t,temp);
		break;
//		case CANID_CH:
//			 dch_receive(&rc_ctrl,  temp);
//		break;
//		case CANID_SW:
//			 dsw_receive(&rc_ctrl,  temp);
//		break;
//		case CANID_MOUSE:
//			 dmouse_receive(&rc_ctrl,  temp);
//		break;	

	}
	
	__HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void Frequency_calc(void)
{
	rc_ctrl.Rx_Frequency[0] = rc_ctrl.Rx_add[0];
	rc_ctrl.Rx_add[0] = 0;		
	rc_ctrl.Rx_Frequency[1] = rc_ctrl.Rx_add[1];
	rc_ctrl.Rx_add[1] = 0;		
	rc_ctrl.Rx_Frequency[2] = rc_ctrl.Rx_add[2];
	rc_ctrl.Rx_add[2] = 0;	

	
//	Motor_6020.RX_Frequancy =  Motor_6020.Rx_add;
//	Motor_6020.Rx_add = 0;



//	Motor_3508[0].Rx_Frequancy = Motor_3508[0].Rx_add;
//	Motor_3508[0].Rx_add = 0;
//	Motor_3508[1].Rx_Frequancy = Motor_3508[1].Rx_add;
//	Motor_3508[1].Rx_add = 0;	
}
