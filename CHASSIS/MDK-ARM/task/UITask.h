#ifndef __UITASK_H__
#define __UITASK_H__

#include "main.h"
#include "judge.h"

void Info_Transmit_task(void const * argument);

///* 裁判系统串口双缓冲区 */
//extern uint8_t Referee_Buffer[2][REFEREE_USART_RX_BUF_LENGHT];

///* 裁判系统接收数据队列 */
//extern fifo_s_t Referee_FIFO;
//extern uint8_t Referee_FIFO_Buffer[REFEREE_FIFO_BUF_LENGTH];

///* protocol解析包结构体 */
//extern unpack_data_t Referee_Unpack_OBJ;

void UI_task(void const * argument);

#endif

