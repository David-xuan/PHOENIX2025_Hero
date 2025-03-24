#ifndef __HWT606_H
#define __HWT606_H
#include <stdint.h>
#include "usart.h"

typedef struct
{
	char head;
	char type;
	float roll;
	float pitch;
	float yaw;
} IMU_TYPE;

void IMU_Init(void);
void USER_UART_IRQHandler(UART_HandleTypeDef *huart);
void zero(void);
void zero_yaw(void);
void zero_pitch(void);

extern IMU_TYPE imu_t;	

static void imu_receive(uint8_t IMU_Rx_Buffer[11],IMU_TYPE *imu_t);
static void UART1_TX_DMA_Send(uint8_t *buffer, uint16_t length);
//static void UART1_TX_DMA_Send(uint8_t *buffer, uint16_t length);
#endif /* __HWT606_H */
