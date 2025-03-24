#ifndef MOTION_H
#define MOTION_H
#include "stm32f4xx.h"
#include "remote_control.h"

typedef struct
{
	float motor[4];
}Motion_Struct;

extern Motion_Struct motion;

void ChassisSolving(Direc_Struct *direc,Motion_Struct *motion);
float calculate_sin(float x);
float calculate_cos(float x);
float calculate_asin(float x);
float calculate_atan(float x);
float invSqrt(float x);




#endif

