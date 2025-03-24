#include "HWT606.h"

#include <dma.h>
#include "cmsis_os.h"
//#include <usart.h>
//#include <arm_math.h>
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;

IMU_TYPE imu_t;			   // 声明陀螺仪数据结构体
uint8_t IMU_Rx_Buffer[14]; // 声明陀螺仪缓存数组
uint8_t IMU_Rx0_Buffer[11];


/**
 * @brief 初始化IMU（惯性测量单元）
 *
 * 该函数通过启用DMA接收来初始化IMU设备，以在后台接收数据。
 * 这样做可以提高效率，因为它允许设备在接收数据的同时执行其他任务。
 *
 * @param huart1 UART1设备句柄
 * @retval IMU_Rx_Buffer 返回接收到的数据 11字节数组
 */
void IMU_Init(void)
{
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE); 
	HAL_UART_Receive_DMA(&huart6, IMU_Rx_Buffer, 11);
	
}


//0x55 0x53 RollL RolIH PitchL PitchH YawL YawH VL VH SUM
//名称			描述						备注
//RolIL			滚转角X低8位			滚转角X=((RollH<<8)|RollL)/32768*180(°)
//RolIH			滚转角X高8位
//PitchL		俯仰角Y低8位			俯仰角Y=((PitchH<<8)|PitchL)/32768*180(°)
//PitchH		俯仰角Y高8位
//YawL			偏航角Z低8位			偏航角Z=((YawH<<8)|YawL)/32768*180(°)
//YawH			偏航角Z高8位
//VL				版本号 低8位
//VH				版本号 高8位			版本号计算公式:=(VH<<8)|VL
//SUM				校验和						SUM=0x55+0x53+RollH+RollL+PitchH+PitchL+YawH+YawL+VH+V
//L
//*/

/**
 * @brief UART接收完成回调函数
 *
 * 当UART完成一帧数据接收时，此回调函数会被调用。
 * 它的目的是处理接收到的IMU数据，包括检查帧头和解析IMU数据。
 *
 * @param UartHandle 指向UART_HandleTypeDef的指针，表示UART设备
 * @retval roll 滚转角X pitcht 俯仰角Y yaw 偏航角Z
 */

static void imu_receive(uint8_t IMU_Rx_Buffer[11],IMU_TYPE *imu_t)
{
	imu_t->head = IMU_Rx_Buffer[0];
	imu_t->type = IMU_Rx_Buffer[1];
	imu_t->roll= (short)(IMU_Rx_Buffer[3] << 8 | IMU_Rx_Buffer[2]) / 32768.0f * 3.1415f;
	imu_t->pitch = (short)(IMU_Rx_Buffer[5] << 8 | IMU_Rx_Buffer[4]) / 32768.0f * 3.1415f;
	imu_t->yaw = (short)(IMU_Rx_Buffer[7] << 8 | IMU_Rx_Buffer[6]) / 32768.0f * 3.1415f;
}

//角度参考
//角度参考是以传感器当前的实际位置，让xy轴的角度归零，做一个相对归零操作。
//指令操作流程：
//1.解锁：FF AA 69 88 B5
//1.1延时200ms
//2.校准：FF AA 01 08 00
//2.1延时3秒
//3.保存: FF AA 00 00 00

//Z轴置零
//注：z轴归零需要在六轴算法的前提下，算法切换可以在上位机配置界面修改，九轴设备下的九轴算法是绝对角度，不能归零。
//指令操作流程：
//1.解锁：FF AA 69 88 B5
//1.1延时200ms
//2.校准：FF AA 01 04 00
//2.1延时3秒
//3.保存: FF AA 00 00 00


static void UART1_TX_DMA_Send(uint8_t *buffer, uint16_t length)
{
    //等待上一次的数据发送完毕
	while(HAL_DMA_GetState(&hdma_usart6_tx) != HAL_DMA_STATE_READY);
    //while(__HAL_DMA_GET_COUNTER(&hdma_usart1_tx));
	
    //关闭DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    //开始发送数据
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
    HAL_UART_DMAStop(&huart6);                                                     //停止本次DMA传输
//	memcpy(IMU_Rx_Buffer,&IMU_Rx0_Buffer[i],11);
	// 解析接收到的IMU数据
	if(IMU_Rx_Buffer[0]==0x55 & IMU_Rx_Buffer[1]==0x53)
	{imu_receive(IMU_Rx_Buffer,&imu_t); }                                           //清零接收缓冲区
    HAL_UART_Receive_DMA(&huart6,IMU_Rx_Buffer, 11);                    //重启开始DMA传输 每次11字节数据
}

void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART6)                               //判断是否是串口6
    {
        if(RESET != __HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE))   //判断是否是空闲中断
        {
            __HAL_UART_CLEAR_IDLEFLAG(&huart6);                     //清楚空闲中断标志（否则会一直不断进入中断）
            USAR_UART_IDLECallback(huart);                          //调用中断处理函数
        }
    }
}

