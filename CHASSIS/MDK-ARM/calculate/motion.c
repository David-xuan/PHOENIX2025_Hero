#include "motion.h"
#include "pid.h"
#include "math.h"

extern Motor_struct motor;
Motion_Struct motion;
float angle,front,right,rotate,COS,SIN;

void ChassisSolving(Direc_Struct *direc,Motion_Struct *motion)
{
		COS = cos(angle);
		SIN = sin(angle);
		angle=motor.Gimbal_Yaw.Apid.now;
		front = direc->front*COS+direc->right*SIN;
		right = -direc->front*SIN+direc->right*COS;
		rotate = direc->rotate;
		motion->motor[0]=(-front+right)+rotate;
		motion->motor[1]=(front+right)+rotate;
		motion->motor[2]=(front-right)+rotate;
		motion->motor[3]=(-front-right)+rotate;
	
		motor.Motor[0].Vpid.goal = motion->motor[0];
		motor.Motor[1].Vpid.goal = motion->motor[1];
		motor.Motor[2].Vpid.goal = motion->motor[2];
		motor.Motor[3].Vpid.goal = motion->motor[3];

}
	

