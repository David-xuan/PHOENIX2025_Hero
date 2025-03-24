/**
  ******************************************************************************
  * @file    judge.c
  * @brief   解析裁判系统发来的数据，以获得机器人当前的状态数据。
  *
  ******************************************************************************
  * @attention
  *
  * 2021.3 已经完成了和tuxin.c的适配。
  *
  *
  *
  ******************************************************************************
  */

#include "judge.h"
#include "usart.h"
#include "CRCs.h"
#include "string.h"
#include "dma.h"


bool Judge_Data_TF = false;//裁判数据是否可用，辅助函数调用
uint8_t Judge_Self_ID;//当前机器人的ID
uint16_t Judge_SelfClient_ID;//发送者机器人对应的客户端ID
/**************裁判系统数据辅助****************/
uint16_t ShootNum=0;//统计发弹量,0x0003触发一次则认为发射了一颗
bool Hurt_Data_Update = false;//装甲板伤害数据是否更新,每受一次伤害置TRUE,然后立即置FALSE,给底盘闪避用
uint16_t Hurt_num = 0;
#define BLUE  0
#define RED   1


/* protocol包头结构体 */
frame_header_struct_t Referee_Receive_Header;

/* 0x000X */
ext_game_status_t   Game_Status;
ext_game_result_t   Game_Result;
ext_game_robot_HP_t Game_Robot_HP;

/* 0x010X */
ext_event_data_t                Event_Data;
ext_supply_projectile_action_t  Supply_Projectile_Action;
ext_supply_projectile_booking_t Supply_Projectile_Booking;
ext_referee_warning_t           Referee_Warning;
ext_dart_remaining_time_t       Dart_Remaining_Time;

/* 0x020X */
ext_game_robot_state_t Game_Robot_State;
ext_power_heat_data_t  Power_Heat_Data;
ext_game_robot_pos_t   Game_Robot_Pos;
ext_buff_musk_t        Buff_Musk;
aerial_robot_energy_t  Aerial_Robot_Energy;
ext_robot_hurt_t       Robot_Hurt;
ext_shoot_data_t       Shoot_Data;
ext_bullet_remaining_t Bullet_Remaining;
ext_rfid_status_t      RFID_Status;
ext_dart_client_cmd_t  Dart_Client_Cmd;

/* 0x030X */
ext_student_interactive_header_data_t Student_Interactive_Header_Data;
robot_interactive_data_t              Robot_Interactive_Data;
ext_robot_command_t                   Robot_Command;
ext_client_map_command_t              Client_Map_Command;


/* Private variables ---------------------------------------------------------*/
/* 裁判系统串口双缓冲区 */
uint8_t Referee_Buffer[2][REFEREE_USART_RX_BUF_LENGHT];

extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

/* 裁判系统接收数据队列 */
fifo_s_t Referee_FIFO;
uint8_t Referee_FIFO_Buffer[REFEREE_FIFO_BUF_LENGTH];

/* protocol解析包结构体 */
unpack_data_t Referee_Unpack_OBJ;


extern DMA_HandleTypeDef hdma_usart6_rx;

extern DMA_HandleTypeDef hdma_usart6_rx;

void USART6_IRQHandler_1(void)
{
		if(huart6.Instance->SR & UART_FLAG_RXNE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);
    }
    else if(USART6->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart6);

        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)//现在是缓冲区1,读取2内存
        {
            __HAL_DMA_DISABLE(&hdma_usart6_rx);
            this_time_rx_len = REFEREE_USART_RX_BUF_LENGHT - hdma_usart6_rx.Instance->NDTR;
            hdma_usart6_rx.Instance->NDTR = REFEREE_USART_RX_BUF_LENGHT;
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(&hdma_usart6_rx);
						fifo_s_puts(&Referee_FIFO, (char*)Referee_Buffer[1], this_time_rx_len);
        }
        else//现在是缓冲区2,读取1内存
        {
            __HAL_DMA_DISABLE(&hdma_usart6_rx);
            this_time_rx_len = REFEREE_USART_RX_BUF_LENGHT - hdma_usart6_rx.Instance->NDTR;
            hdma_usart6_rx.Instance->NDTR = REFEREE_USART_RX_BUF_LENGHT;
            DMA2_Stream1->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(&hdma_usart6_rx);
						fifo_s_puts(&Referee_FIFO, (char*)Referee_Buffer[0], this_time_rx_len);
        }
    }
}

/*==============================================================================
              ##### 裁判系统初始化函数 #####
  ==============================================================================
    [..]  该部分提供如下函数:
		  (+) 裁判系统结构体初始化函数 Referee_StructInit
			(+) 裁判系统串口初始化函数 Referee_UARTInit
*/
void Referee_StructInit(void)
{
	memset(&Referee_Receive_Header,          0, sizeof(Referee_Receive_Header));
	
	memset(&Game_Status,                     0, sizeof(Game_Status));
	memset(&Game_Result,                     0, sizeof(Game_Result));
	memset(&Game_Robot_HP,                   0, sizeof(Game_Robot_HP));
	
	memset(&Event_Data,                      0, sizeof(Event_Data));
	memset(&Supply_Projectile_Action,        0, sizeof(Supply_Projectile_Action));
	memset(&Supply_Projectile_Booking,       0, sizeof(Supply_Projectile_Booking));
	memset(&Referee_Warning,                 0, sizeof(Referee_Warning));
	memset(&Dart_Remaining_Time,             0, sizeof(Dart_Remaining_Time));
	
	memset(&Game_Robot_State,                0, sizeof(Game_Robot_State));
	memset(&Power_Heat_Data,                 0, sizeof(Power_Heat_Data));
	memset(&Game_Robot_Pos,                  0, sizeof(Game_Robot_Pos));
	memset(&Buff_Musk,                       0, sizeof(Buff_Musk));
	memset(&Aerial_Robot_Energy,             0, sizeof(Aerial_Robot_Energy));
	memset(&Robot_Hurt,                      0, sizeof(Robot_Hurt));
	memset(&Shoot_Data,                      0, sizeof(Shoot_Data));
	memset(&Bullet_Remaining,                0, sizeof(Bullet_Remaining));
	memset(&RFID_Status,                     0, sizeof(RFID_Status));
	memset(&Dart_Client_Cmd,                 0, sizeof(Dart_Client_Cmd));
	
	memset(&Student_Interactive_Header_Data, 0, sizeof(Student_Interactive_Header_Data));
	memset(&Robot_Interactive_Data,          0, sizeof(Robot_Interactive_Data));
	memset(&Robot_Command,                   0, sizeof(Robot_Command));
	memset(&Client_Map_Command,              0, sizeof(Client_Map_Command));
}

//buff0 1为两个缓存区 
void Referee_UARTInit(uint8_t *Buffer0, uint8_t *Buffer1, uint16_t BufferLength)
{
	/* 使能串口DMA */
	SET_BIT(Referee_UART.Instance->CR3, USART_CR3_DMAR);
	SET_BIT(Referee_UART.Instance->CR3, USART_CR3_DMAT);
	
	/* 使能串口空闲中断 */
	__HAL_UART_ENABLE_IT(&Referee_UART, UART_IT_IDLE);
	
	/* 确保DMA RX失能 */
	while(Referee_UART.hdmarx->Instance->CR & DMA_SxCR_EN)
	{
		__HAL_DMA_DISABLE(Referee_UART.hdmarx);
	}
	
	/* 清空标志位 */
	__HAL_DMA_CLEAR_FLAG(Referee_UART.hdmarx, DMA_LISR_TCIF1);

	/* 设置接收双缓冲区 */
	Referee_UART.hdmarx->Instance->PAR  = (uint32_t) & (Referee_UART.Instance->DR);
	Referee_UART.hdmarx->Instance->M0AR = (uint32_t)(Buffer0);
	Referee_UART.hdmarx->Instance->M1AR = (uint32_t)(Buffer1);
	
	/* 设置数据长度 */
	__HAL_DMA_SET_COUNTER(Referee_UART.hdmarx, BufferLength);
	
	/* 使能双缓冲区 */
	SET_BIT(Referee_UART.hdmarx->Instance->CR, DMA_SxCR_DBM);
	
	/* 使能DMA RX */
	__HAL_DMA_ENABLE(Referee_UART.hdmarx);
	
	/* 确保DMA TX失能 */
	while(Referee_UART.hdmatx->Instance->CR & DMA_SxCR_EN)
	{
		__HAL_DMA_DISABLE(Referee_UART.hdmatx);
	}
	
	Referee_UART.hdmatx->Instance->PAR  = (uint32_t) & (Referee_UART.Instance->DR);
}

/*==============================================================================
              ##### 裁判系统数据解析函数 #####
  ==============================================================================
    [..]  该部分提供如下函数:
		  (+) 裁判系统队列数据解压函数 Referee_UnpackFifoData
      (+) 裁判系统队列数据处理函数 Referee_SolveFifoData
*/
void Referee_UnpackFifoData(unpack_data_t *referee_unpack_obj, fifo_s_t *referee_fifo)
{
  uint8_t byte = 0;
  uint8_t sof  = HEADER_SOF;
	
  while(fifo_s_used(referee_fifo))
  {
    byte = fifo_s_get(referee_fifo);
    switch(referee_unpack_obj->unpack_step)
    {
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          referee_unpack_obj->unpack_step = STEP_LENGTH_LOW;
          referee_unpack_obj->protocol_packet[referee_unpack_obj->index++] = byte;
        }
        else
        {
          referee_unpack_obj->index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        referee_unpack_obj->data_len = byte;
        referee_unpack_obj->protocol_packet[referee_unpack_obj->index++] = byte;
        referee_unpack_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        referee_unpack_obj->data_len |= (byte << 8);
        referee_unpack_obj->protocol_packet[referee_unpack_obj->index++] = byte;
        if(referee_unpack_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
        {
          referee_unpack_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          referee_unpack_obj->unpack_step = STEP_HEADER_SOF;
          referee_unpack_obj->index = 0;
        }
      }break;
			
      case STEP_FRAME_SEQ:
      {
        referee_unpack_obj->protocol_packet[referee_unpack_obj->index++] = byte;
        referee_unpack_obj->unpack_step = STEP_HEADER_CRC8;
      }break;
			
      case STEP_HEADER_CRC8:
      {
        referee_unpack_obj->protocol_packet[referee_unpack_obj->index++] = byte;
        if(referee_unpack_obj->index == REF_PROTOCOL_HEADER_SIZE)
        {
          if(CRC08_Verify(referee_unpack_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE))
          {
            referee_unpack_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            referee_unpack_obj->unpack_step = STEP_HEADER_SOF;
            referee_unpack_obj->index = 0;
          }
        }
      }break;  
      
      case STEP_DATA_CRC16:
      {
        if(referee_unpack_obj->index <  (REF_HEADER_CRC_CMDID_LEN + referee_unpack_obj->data_len))
        {
           referee_unpack_obj->protocol_packet[referee_unpack_obj->index++] = byte;  
        }
        if(referee_unpack_obj->index >= (REF_HEADER_CRC_CMDID_LEN + referee_unpack_obj->data_len))
        {
          referee_unpack_obj->unpack_step = STEP_HEADER_SOF;
          referee_unpack_obj->index = 0;
          if(CRC16_Verify(referee_unpack_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + referee_unpack_obj->data_len))
          {
            Referee_SolveFifoData(referee_unpack_obj->protocol_packet);
          }
        }
      }break;

      default:
      {
				Judge_Data_TF = false ;
        referee_unpack_obj->unpack_step = STEP_HEADER_SOF;
        referee_unpack_obj->index = 0;
      }break;
    }
  }
}

referee_cmd_id_e cmd_id = GAME_STATE_CMD_ID;

void Referee_SolveFifoData(uint8_t *frame)
{

	uint8_t  index  = 0;
	
	memcpy(&Referee_Receive_Header, frame, sizeof(frame_header_struct_t));
	index += sizeof(frame_header_struct_t);
	memcpy(&cmd_id, frame + index, sizeof(uint16_t));
	index += sizeof(uint16_t);
	
	switch(cmd_id)
	{
		case GAME_STATE_CMD_ID:      	         memcpy(&Game_Status,               frame + index, sizeof(ext_game_status_t));               break;
		case GAME_RESULT_CMD_ID:               memcpy(&Game_Result,               frame + index, sizeof(ext_game_result_t));               break;
		case GAME_ROBOT_HP_CMD_ID:             memcpy(&Game_Robot_HP,             frame + index, sizeof(ext_game_robot_HP_t));             break;
		case DART_FLYING_STATE_CMD_ID:                                                                                                     break;
		
    case FIELD_EVENTS_CMD_ID:              memcpy(&Event_Data,                frame + index, sizeof(ext_event_data_t));                break;
		case SUPPLY_PROJECTILE_ACTION_CMD_ID:  memcpy(&Supply_Projectile_Action,  frame + index, sizeof(ext_supply_projectile_action_t));  break;
		case SUPPLY_PROJECTILE_BOOKING_CMD_ID: memcpy(&Supply_Projectile_Booking, frame + index, sizeof(ext_supply_projectile_booking_t)); break;
		case REFEREE_WARNING_CMD_ID:           memcpy(&Referee_Warning,           frame + index, sizeof(ext_referee_warning_t));           break;
		case DART_REMAINING_TIME_CMD_ID:       memcpy(&Dart_Remaining_Time,       frame + index, sizeof(ext_dart_remaining_time_t));       break;
		
		case ROBOT_STATE_CMD_ID:
		{
		  memcpy(&Game_Robot_State, frame + index, sizeof(ext_game_robot_state_t));
		  break;
		}
		case POWER_HEAT_DATA_CMD_ID:
		{
			memcpy(&Power_Heat_Data,           frame + index, sizeof(ext_power_heat_data_t));           
			break;
		}
		case ROBOT_POS_CMD_ID:
		{
			memcpy(&Game_Robot_Pos,            frame + index, sizeof(ext_game_robot_pos_t));            
			break;
		}
		case BUFF_MUSK_CMD_ID:                 memcpy(&Buff_Musk,                 frame + index, sizeof(ext_buff_musk_t));                 break;
		case AERIAL_ROBOT_ENERGY_CMD_ID:       memcpy(&Aerial_Robot_Energy,       frame + index, sizeof(aerial_robot_energy_t));           break;
		case ROBOT_HURT_CMD_ID:                memcpy(&Robot_Hurt,                frame + index, sizeof(ext_robot_hurt_t));                break;
		case SHOOT_DATA_CMD_ID:                Shoot_Data.RX_MSG.RX_add++;        memcpy(&Shoot_Data,                frame + index, sizeof(ext_shoot_data_t));                 break;
		case BULLET_REMAINING_CMD_ID:          memcpy(&Bullet_Remaining,          frame + index, sizeof(ext_bullet_remaining_t));          break;
		case ROBOT_RFID_STATE_CMD_ID:          memcpy(&RFID_Status,               frame + index, sizeof(ext_rfid_status_t));               break;
		case DART_CLIENT_CMD_ID:               memcpy(&Dart_Client_Cmd,           frame + index, sizeof(ext_dart_client_cmd_t));           break;
		
		case STUDENT_INTERACTIVE_DATA_CMD_ID:  memcpy(&Robot_Interactive_Data,    frame + index, sizeof(robot_interactive_data_t));        break;
		case ROBOT_COMMAND_CMD_ID:             memcpy(&Robot_Command,             frame + index, sizeof(ext_robot_command_t));             break;
		case CLIENT_MAP_COMMAND_CMD_ID:        memcpy(&Client_Map_Command,        frame + index, sizeof(ext_client_map_command_t));        break;
		
		default:                                                                                                                           break;
	}
	Judge_Data_TF = true ;
}



bool Color;
bool is_red_or_blue(void)//判断自己红蓝方
{
    Judge_Self_ID = Game_Robot_State.robot_id;//读取当前机器人ID

    if(Game_Robot_State.robot_id > 10)
    {
        return BLUE;
    }
    else
    {
        return RED;
    }
}

uint16_t determine_ID(void)
{
    Color = is_red_or_blue();
    if(Color == BLUE)
    {
        Judge_SelfClient_ID = 0x0164 + (Judge_Self_ID - 100); //计算客户端id
    }
    else if(Color == RED)
    {
        Judge_SelfClient_ID = 0x0100 + Judge_Self_ID;//计算客户端ID
    }
		return Judge_SelfClient_ID;
}

/********************裁判数据辅助判断函数***************************/

/**
  * @brief  数据是否可用
  * @param  void
  * @retval  TRUE可用   FALSE不可用
  * @attention  在裁判读取函数中实时改变返回值
  */
bool JUDGE_sGetDataState(void)
{
    return Judge_Data_TF;
}

uint8_t JUDGE_GameState(void)
{
  return Game_Status.game_progress;
}
/**
  * @brief  读取瞬时功率
  * @param  void
  * @retval 实时功率值
  * @attention
  */
float JUDGE_fGetChassisPower(void)
{
    return (Power_Heat_Data.chassis_power);
}

/**
  * @brief  读取底盘功率限制
  * @param  void
  * @retval 实时底盘功率限制
  * @attention
  */

uint8_t JUDGE_usGetPowerLimit(void)
{
	
    return (Game_Robot_State.chassis_power_limit );
}



//Power_Heat_Data

/**
  * @brief  读取剩余焦耳能量
  * @param  void
  * @retval 剩余缓冲焦耳能量(最大60)
  * @attention
  */
uint16_t JUDGE_fGetRemainEnergy(void)
{
    return (Power_Heat_Data.chassis_power_buffer);
}

/**
  * @brief  读取当前等级
  * @param  void
  * @retval 当前等级
  * @attention
  */
uint8_t JUDGE_ucGetRobotLevel(void)
{
    return	Game_Robot_State.robot_level;
}

/**
  * @brief  读取枪口热量
  * @param  void
  * @retval 42mm
  * @attention  实时热量
  */
uint16_t JUDGE_usGetRemoteHeat42(void)
{
    return Power_Heat_Data.shooter_id1_42mm_cooling_heat;
}

/**
  * @brief  读取射速
  * @param  void
  * @retval 42mm
  * @attention  实时射速
  */
float JUDGE_usGetSpeed42(void)
{
    return Shoot_Data.bullet_speed;
}

/**
  * @brief  统计发弹量
  * @param  void
  * @retval void
  * @attention
  */
float Shoot_Speed_Now = 0;
float Shoot_Speed_Last = 0;
void JUDGE_ShootNumCount(void)
{
    Shoot_Speed_Now = Shoot_Data.bullet_speed;
    if(Shoot_Speed_Last != Shoot_Speed_Now)//因为是float型，几乎不可能完全相等,所以速度不等时说明发射了一颗弹
    {
        ShootNum++;
        Shoot_Speed_Last = Shoot_Speed_Now;
    }
}

/**
  * @brief  读取发弹量
  * @param  void
  * @retval 发弹量
  * @attention 不适用于双枪管
  */
uint16_t JUDGE_usGetShootNum(void)
{
    return ShootNum;
}

/**
  * @brief  发弹量清零
  * @param  void
  * @retval void
  * @attention
  */
void JUDGE_ShootNum_Clear(void)
{
    ShootNum = 0;
}

/**
  * @brief  读取枪口热量
  * @param  void
  * @retval 当前等级42mm热量上限
  * @attention
  */
uint16_t JUDGE_usGetHeatLimit(void)
{
    return Game_Robot_State.shooter_barrel_heat_limit;
}
/**
  * @brief  读取枪口射速上限
  * @param  void
  * @retval 当前等级42mm射速上限
  * @attention
  */
uint8_t JUDGE_usGetSpeedLimit(void)
{
    return 16 ;
}
/**
  * @brief  当前等级对应的枪口每秒冷却值
  * @param  void
  * @retval 当前等级42mm冷却速度
  * @attention
  */
uint16_t JUDGE_usGetShootCold(void)
{
    return Game_Robot_State.shooter_barrel_cooling_value;
}

bool Judge_If_Death(void)
{
    if(Game_Robot_State.current_HP == 0 && JUDGE_sGetDataState() == true)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool Judge_If_Near_Death(void)
{
    if(Game_Robot_State.current_HP <= 20 && JUDGE_sGetDataState() == true)
    {
        return true;
    }
    else
    {
        return false;
    }
}

