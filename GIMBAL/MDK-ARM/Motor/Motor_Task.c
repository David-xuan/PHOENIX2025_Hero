#include "Motor_Task.h"
#include "DM4310.h"
#include "bsp_can.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "cmsis_os.h"



void Motor_Task(void const * argument)
{
	portTickType currentTime;
	Motor_DM4310_Enable(&hcan2,DM4310_ID);
	for(;;)
	{
		currentTime = xTaskGetTickCount(); //��ǰϵͳʱ��
		Motor_DM4310_send(&hcan2,DM4310_ID, 0,0, 0, 0, 0);
		osDelay(1);
		vTaskDelayUntil(&currentTime, 1); //������ʱ
	}
}


