#ifndef MOTION_H
#define MOTION_H
#include "stm32f4xx.h"
#include "remote_control.h"
#include "bsp_can.h"

typedef struct
{
	float motor[4];
}Motion_Struct;


void ChassisSolving(Direc_Struct *direc,Motion_Struct *motion);





#endif

