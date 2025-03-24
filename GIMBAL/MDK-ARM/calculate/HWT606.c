#include "HWT606.h"

#include <dma.h>
#include "cmsis_os.h"
//#include <usart.h>
//#include <arm_math.h>
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;

IMU_TYPE imu_t;			   // �������������ݽṹ��
uint8_t IMU_Rx_Buffer[14]; // ���������ǻ�������
uint8_t IMU_Rx0_Buffer[11];


/**
 * @brief ��ʼ��IMU�����Բ�����Ԫ��
 *
 * �ú���ͨ������DMA��������ʼ��IMU�豸�����ں�̨�������ݡ�
 * �������������Ч�ʣ���Ϊ�������豸�ڽ������ݵ�ͬʱִ����������
 *
 * @param huart1 UART1�豸���
 * @retval IMU_Rx_Buffer ���ؽ��յ������� 11�ֽ�����
 */
void IMU_Init(void)
{
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE); 
	HAL_UART_Receive_DMA(&huart6, IMU_Rx_Buffer, 11);
	
}


//0x55 0x53 RollL RolIH PitchL PitchH YawL YawH VL VH SUM
//����			����						��ע
//RolIL			��ת��X��8λ			��ת��X=((RollH<<8)|RollL)/32768*180(��)
//RolIH			��ת��X��8λ
//PitchL		������Y��8λ			������Y=((PitchH<<8)|PitchL)/32768*180(��)
//PitchH		������Y��8λ
//YawL			ƫ����Z��8λ			ƫ����Z=((YawH<<8)|YawL)/32768*180(��)
//YawH			ƫ����Z��8λ
//VL				�汾�� ��8λ
//VH				�汾�� ��8λ			�汾�ż��㹫ʽ:=(VH<<8)|VL
//SUM				У���						SUM=0x55+0x53+RollH+RollL+PitchH+PitchL+YawH+YawL+VH+V
//L
//*/

/**
 * @brief UART������ɻص�����
 *
 * ��UART���һ֡���ݽ���ʱ���˻ص������ᱻ���á�
 * ����Ŀ���Ǵ�����յ���IMU���ݣ��������֡ͷ�ͽ���IMU���ݡ�
 *
 * @param UartHandle ָ��UART_HandleTypeDef��ָ�룬��ʾUART�豸
 * @retval roll ��ת��X pitcht ������Y yaw ƫ����Z
 */

static void imu_receive(uint8_t IMU_Rx_Buffer[11],IMU_TYPE *imu_t)
{
	imu_t->head = IMU_Rx_Buffer[0];
	imu_t->type = IMU_Rx_Buffer[1];
	imu_t->roll= (short)(IMU_Rx_Buffer[3] << 8 | IMU_Rx_Buffer[2]) / 32768.0f * 3.1415f;
	imu_t->pitch = (short)(IMU_Rx_Buffer[5] << 8 | IMU_Rx_Buffer[4]) / 32768.0f * 3.1415f;
	imu_t->yaw = (short)(IMU_Rx_Buffer[7] << 8 | IMU_Rx_Buffer[6]) / 32768.0f * 3.1415f;
}

//�ǶȲο�
//�ǶȲο����Դ�������ǰ��ʵ��λ�ã���xy��ĽǶȹ��㣬��һ����Թ��������
//ָ��������̣�
//1.������FF AA 69 88 B5
//1.1��ʱ200ms
//2.У׼��FF AA 01 08 00
//2.1��ʱ3��
//3.����: FF AA 00 00 00

//Z������
//ע��z�������Ҫ�������㷨��ǰ���£��㷨�л���������λ�����ý����޸ģ������豸�µľ����㷨�Ǿ��ԽǶȣ����ܹ��㡣
//ָ��������̣�
//1.������FF AA 69 88 B5
//1.1��ʱ200ms
//2.У׼��FF AA 01 04 00
//2.1��ʱ3��
//3.����: FF AA 00 00 00


static void UART1_TX_DMA_Send(uint8_t *buffer, uint16_t length)
{
    //�ȴ���һ�ε����ݷ������
	while(HAL_DMA_GetState(&hdma_usart6_tx) != HAL_DMA_STATE_READY);
    //while(__HAL_DMA_GET_COUNTER(&hdma_usart1_tx));
	
    //�ر�DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    //��ʼ��������
    HAL_UART_Transmit_DMA(&huart6, buffer, length);
}
uint8_t IMU_Enable[5] =	 {0xFF,0xAA,0x69,0x88,0xB5};	 
uint8_t IMU_zeroXY[5] = {0xFF,0xAA,0x01,0x08,0x00};
uint8_t IMU_zeroZ[5] =	 {0xFF,0xAA,0x01,0x04,0x00};
uint8_t IMU_disable[5] = {0xFF,0xAA,0x00,0x00,0x00};	 

 void zero(void)
{
		UART1_TX_DMA_Send(IMU_Enable, 5);
		HAL_Delay(20);
		UART1_TX_DMA_Send(IMU_zeroXY, 5);
		HAL_Delay(300);
		UART1_TX_DMA_Send(IMU_disable, 5);
		HAL_Delay(300);
		UART1_TX_DMA_Send(IMU_Enable, 5);
		HAL_Delay(20);
		UART1_TX_DMA_Send(IMU_zeroZ, 5);
		HAL_Delay(300);
		UART1_TX_DMA_Send(IMU_disable, 5);
		HAL_Delay(300);

}

 void zero_yaw(void)
{
		UART1_TX_DMA_Send(IMU_Enable, 5);
		HAL_Delay(20);
		UART1_TX_DMA_Send(IMU_zeroZ, 5);
		HAL_Delay(300);
		UART1_TX_DMA_Send(IMU_disable, 5);
		HAL_Delay(300);
}

 void zero_pitch(void)
{
		UART1_TX_DMA_Send(IMU_Enable, 5);
		HAL_Delay(20);
		UART1_TX_DMA_Send(IMU_zeroXY, 5);
		HAL_Delay(300);
		UART1_TX_DMA_Send(IMU_disable, 5);
		HAL_Delay(300);
}

static void USAR_UART_IDLECallback(UART_HandleTypeDef *huart)
{
    HAL_UART_DMAStop(&huart6);                                                     //ֹͣ����DMA����
//	memcpy(IMU_Rx_Buffer,&IMU_Rx0_Buffer[i],11);
	// �������յ���IMU����
	if(IMU_Rx_Buffer[0]==0x55 & IMU_Rx_Buffer[1]==0x53)
	{imu_receive(IMU_Rx_Buffer,&imu_t); }                                           //������ջ�����
    HAL_UART_Receive_DMA(&huart6,IMU_Rx_Buffer, 11);                    //������ʼDMA���� ÿ��11�ֽ�����
}

void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART6)                               //�ж��Ƿ��Ǵ���6
    {
        if(RESET != __HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE))   //�ж��Ƿ��ǿ����ж�
        {
            __HAL_UART_CLEAR_IDLEFLAG(&huart6);                     //��������жϱ�־�������һֱ���Ͻ����жϣ�
            USAR_UART_IDLECallback(huart);                          //�����жϴ�����
        }
    }
}

