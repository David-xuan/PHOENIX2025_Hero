#ifndef __GIMBALFUN_H__
#define __GIMBALFUN_H__

#include "Gimbal_task.h"




extern void Gimbal_Init(Gimbal_t* gimbal);
void Gimbal_flag_Init(Gimbal_t* gimbal);
extern void Clean_Gimbal_CtrlMsg(Gimbal_t* gimbal);

extern void Gimbal_Mode_Choose(Gimbal_t* gimbal);
extern void KeyboardControlGimbal(Gimbal_t* gimbal);

extern void RemoteControlGimbal(Gimbal_t* gimbal);

extern void Gimbal_clc(Gimbal_t* gimbal);


extern void Gimbal_Statemachine_2_Stop(Gimbal_t* gimbal);
extern void Gimbal_Statemachine_2_remote(Gimbal_t* gimbal);
extern void Gimbal_Statemachine_2_keyboard(Gimbal_t* gimbal);
extern void Gimbal_Statemachine_2_Init(Gimbal_t* gimbal);
extern void GimbalSelate(Gimbal_t* gimbal);



#endif 

