#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

#include "stm32f4xx.h"
#include "remote_control.h"
#include "bsp_can.h"
#include "pid.h"
#include "motion.h"
#include "cmsis_os.h"

void Shoot_task(void const * argument);



#endif
