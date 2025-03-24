#ifndef __CHASSISFUN_H__
#define __CHASSISFUN_H__
#include "main.h"
#include "Chassis_task.h"

extern void Chassis_Init(chass_t* chassis);
void Chassis_flag_Init(chass_t* chassis);
void Clean_Chassis_CtrlMsg(chass_t* chassis);

//static void mecanum_calc(Chassis_Speed_t *speed, int16_t *out_speed);
//static void Absolute_Cal(Chassis_Speed_t *absolute_speed, float angle );
static float Find_Y_AnglePNY(void);

void Gimbal_follow_Judge(chass_t* chassis);

extern void RemoteControlChassis(chass_t* chassis);
extern void CHASSIS_clc(chass_t* chassis);



extern void Chassis_Choose_remote(chass_t* chassis);
extern void Chassis_Choose_keyboard(chass_t* chassis);

static void Chassis_Keyboard_Move_Calculate( int16_t sMoveMax, int16_t sMoveRamp_inc, int16_t sMoveRamp_dec );
float Chassis_Key_MoveRamp( uint8_t status, int16_t *time, int16_t inc, int16_t dec );
void KeyboardControlChassis(chass_t* chassis);
extern void Chassis_Power_Limit(void);

extern void ChassisSelate(chass_t* chassis);
extern void Chassis_Statemachine_2_Init(chass_t* chassis);
extern void Chassis_Statemachine_2_Stop(chass_t* chassis);
extern void Chassis_Statemachine_2_remote(chass_t* chassis);
extern void Chassis_Statemachine_2_keyboard(chass_t* chassis);



#endif 

