#include "UITask.h"
#include "tuxin.h"
#include "cmsis_os.h"
#include "judge.h"
#include "Chassis_task.h"
#include "Gimbal_task.h"
#include "bsp_can.h"
//#include "cap.h"


uint8_t Robot_ID_Current = Robot_ID_Red_Hero;


void UI_task(void const * argument)
{
	/* 动态UI控制变量 */
	uint16_t UI_PushUp_Counter = 261;
	
	/* 裁判系统初始化 */
	vTaskDelay(300);
	
	/* new UI */
	while(1)
	{
		
		
		vTaskDelay(8);
		Referee_UnpackFifoData(&Referee_Unpack_OBJ, &Referee_FIFO);
    
		Robot_ID_Current = Game_Robot_State.robot_id;
		
		/* UI更新 */
		UI_PushUp_Counter++;

		
		if(UI_PushUp_Counter % 301 == 0) //添加中央标尺1
		{
			UI_Draw_Line(&UI_Graph7.Graphic[0], "001", UI_Graph_Add,    0, UI_Color_Orange, 2,  951 ,412-3 ,951 ,412+3 ); //10m竖线
			UI_Draw_Line(&UI_Graph7.Graphic[1], "002", UI_Graph_Add,    0, UI_Color_Pink, 2,  951-15 ,412 ,951+15 ,412 ); //10m横线
			UI_Draw_Line(&UI_Graph7.Graphic[2], "003", UI_Graph_Add,    0, UI_Color_Pink, 2,  948 ,470-5 ,948 ,470+5 ); //7m竖线
			UI_Draw_Line(&UI_Graph7.Graphic[3], "004", UI_Graph_Add,    0, UI_Color_Pink, 2,  948-15 ,470 ,948+15 ,470); //7m横线			
			UI_Draw_Line(&UI_Graph7.Graphic[4], "005", UI_Graph_Add,    0, UI_Color_Pink, 2,  930 ,	500-5 ,930 ,500+5 ); //7m竖线2.0
			UI_Draw_Line(&UI_Graph7.Graphic[5], "006", UI_Graph_Add,    1, UI_Color_Green, 2, 714, 0, 780, 168);
			UI_Draw_Line(&UI_Graph7.Graphic[6], "007", UI_Graph_Add,    1, UI_Color_Green,2, 1198, 0 ,1132, 168);
			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
			continue;
		}
		else if(UI_PushUp_Counter % 301 == 10)  //自瞄框
		{
			UI_Draw_Rectangle(&UI_Graph1.Graphic[0] ,"008",UI_Graph_Add,  1, UI_Color_White, 4, 666,402,1194,739);
			UI_PushUp_Graphs(1, &UI_Graph1, Robot_ID_Current);
		}
		//自瞄框
		if(UI_PushUp_Counter % 40 == 0 && UI_PushUp_Counter > 310)
		{
			if(Gimbal.IMUData.find_bool == '1')
			{
				UI_Draw_Rectangle(&UI_Graph1.Graphic[0] ,"008",UI_Graph_Change,  1, UI_Color_Pink, 4, 666,402,1194,739);
				UI_PushUp_Graphs(1, &UI_Graph1, Robot_ID_Current);			
			}
			else
			{
				UI_Draw_Rectangle(&UI_Graph1.Graphic[0] ,"008",UI_Graph_Change,  1, UI_Color_White, 4, 666,402,1194,739);
				UI_PushUp_Graphs(1, &UI_Graph1, Robot_ID_Current);				
			}
		}

		

		//是否跟随
		if(UI_PushUp_Counter % 21 == 0 && Chassis.Action == CHASS_FOLLOW) 
		{
				UI_Draw_String(&UI_String.String, "300", UI_Graph_Add,    2, UI_Color_Pink,  44, 7, 4,  100, 600, "FOLLOW "); 
				UI_PushUp_String(&UI_String, Robot_ID_Current);
		}
		if(UI_PushUp_Counter % 21 == 2) 
		{
			if(Chassis.Action == CHASS_FOLLOW)
			{
				UI_Draw_String(&UI_String.String, "300", UI_Graph_Change,    2, UI_Color_Pink,  44, 7, 4,  100, 600, "FOLLOW "); 
				UI_PushUp_String(&UI_String, Robot_ID_Current);
			}
			else
			{
				UI_Draw_String(&UI_String.String, "300", UI_Graph_Delete,    2, UI_Color_Pink,  44, 7, 4,  100, 600, "FOLLOW "); 
				UI_PushUp_String(&UI_String, Robot_ID_Current);
			}
		}
		
		//显示yaw
		if(UI_PushUp_Counter % 8 == 0)
		{
			UI_Draw_Float(&UI_Graph2.Graphic[0], "600", UI_Graph_Add, 4, UI_Color_Green , 30, 7, 4,  1600, 600, motor.Gimbal_Yaw.Apid.now);
			UI_Draw_Float(&UI_Graph2.Graphic[1], "700", UI_Graph_Add, 4, UI_Color_Green , 30, 7, 4,  1600, 800, -Yaw_angle_t.Pitch_angle.value);
			UI_PushUp_Graphs(2, &UI_Graph2, Robot_ID_Current);
		}
		if(UI_PushUp_Counter % 8 == 2)
		{
			UI_Draw_Float(&UI_Graph2.Graphic[0], "600", UI_Graph_Change, 4, UI_Color_Green , 30, 7, 4,  1600, 600, motor.Gimbal_Yaw.Apid.now);
			UI_Draw_Float(&UI_Graph2.Graphic[1], "700", UI_Graph_Change, 4, UI_Color_Green , 30, 7, 4,  1600, 800, -Yaw_angle_t.Pitch_angle.value);
			UI_PushUp_Graphs(2, &UI_Graph2, Robot_ID_Current);
		}

			//是否超电
//			if(UI_PushUp_Counter % 31 == 0 && cap_info.switch_status == 0x02)
//			{
//				UI_Draw_String(&UI_String.String, "500", UI_Graph_Add,    3, UI_Color_Pink,  44, 4, 4,  200, 800, "CAP "); 
//				UI_PushUp_String(&UI_String, Robot_ID_Current);
//			}
//			if(UI_PushUp_Counter % 31 == 2)
//			{
//				if(cap_info.switch_status == 0x02)
//				{
//					UI_Draw_String(&UI_String.String, "500", UI_Graph_Change,    3, UI_Color_Pink,  44, 4, 8,  200, 800, "CAP "); 
//					UI_PushUp_String(&UI_String, Robot_ID_Current);
//				}
//				else	
//				{ 
//					UI_Draw_String(&UI_String.String, "500", UI_Graph_Delete,    3, UI_Color_Pink,  44, 4, 8,  200, 800, "CAP"); 
//					UI_PushUp_String(&UI_String, Robot_ID_Current);			
//				}	
//			}			
		
//		if(UI_PushUp_Counter % 41 == 0)
//		{
//			UI_Draw_Float(&UI_String.String, "400", UI_Graph_Add,    2, UI_Color_Pink,  30, 3,4, 200,  800,cap_info.cap_joule_residue); 
//			UI_PushUp_String(&UI_String, Robot_ID_Current);
//		}	

	}

}






