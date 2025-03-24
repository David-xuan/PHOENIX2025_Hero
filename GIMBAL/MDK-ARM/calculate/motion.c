#include "motion.h"
#include "bsp_can.h"

extern Direc_Struct direc;
extern RC_ctrl_t rc_ctrl;

void ChassisSolving(Direc_Struct *direc,Motion_Struct *motion)
{
	if(rc_ctrl.rc.ch[2]==0)//前后
	{
		motion->motor[0]=-direc->front;
		motion->motor[1]=direc->front;
		motion->motor[2]=direc->front;
		motion->motor[3]=-direc->front;
	}
	else
	if(rc_ctrl.rc.ch[2]<-100&&(rc_ctrl.rc.ch[3]>100||rc_ctrl.rc.ch[3]<-100))//左前后
	{
		motion->motor[0]=0;
		motion->motor[1]=direc->front;
		motion->motor[2]=0;
		motion->motor[3]=-direc->front;
	}
	else
	if(rc_ctrl.rc.ch[2]>100&&(rc_ctrl.rc.ch[3]<-100||rc_ctrl.rc.ch[3]>100))//右前后
	{
		motion->motor[0]=-direc->front;
		motion->motor[1]=0;
		motion->motor[2]=direc->front;
		motion->motor[3]=0;
	}
	else
	if((rc_ctrl.rc.ch[3]>=-100&&rc_ctrl.rc.ch[3]<=100)&&(rc_ctrl.rc.ch[2]<0||rc_ctrl.rc.ch[2]>0))//左右
	{
		motion->motor[0]=direc->right;
		motion->motor[1]=direc->right;
		motion->motor[2]=-direc->right;
		motion->motor[3]=-direc->right;
	}
	
}
