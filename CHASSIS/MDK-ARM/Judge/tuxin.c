/**
  ******************************************************************************
  * @file    tuxin.c
  * @brief   图传界面画图用于显示状态、辅助瞄准。
  * @author  CHY
  ******************************************************************************
  * @attention
  *
  * 2021.3 设计大致完成。
  * 2021.4.11 暂时还显示不了字符，只能显示图形。已经实现了图形根据车的某些状态进行动态的变化。
  * 暂时还不知道为什么会有这么诡异的事。
  *
  *
  *
  *
  ******************************************************************************
  */
#include "judge.h"
#include "tuxin.h"
#include "CRCs.h"
#include "string.h"
#include "usart.h"


/* 绘制UI专用结构体 */
UI_Graph1_t UI_Graph1;
UI_Graph2_t UI_Graph2;
UI_Graph5_t UI_Graph5;
UI_Graph7_t UI_Graph7;
UI_String_t UI_String;
UI_Delete_t UI_Delete;



/*==============================================================================
              ##### UI基本图形绘制函数 #####
  ==============================================================================
    [..]  该部分提供如下函数:
		  (+) 绘制直线 UI_Draw_Line
      (+) 绘制矩形 UI_Draw_Rectangle
      (+) 绘制整圆 UI_Draw_Circle
      (+) 绘制椭圆 UI_Draw_Ellipse
      (+) 绘制圆弧 UI_Draw_Arc
      (+) 绘制小数 UI_Draw_Float
      (+) 绘制整数 UI_Draw_Int
      (+) 绘制字符 UI_Draw_String
*/



void UI_Draw_Line(graphic_data *Graph,        //UI图形数据结构体指针
	                char                   GraphName[3], //图形名 作为客户端的索引
									uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
									uint8_t                Layer,        //UI图形图层 [0,9]
									uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
									uint16_t               Width,        //线宽
									uint16_t               StartX,       //起始坐标X
									uint16_t               StartY,       //起始坐标Y
									uint16_t               EndX,         //截止坐标X
									uint16_t               EndY)         //截止坐标Y
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Line;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->width           = Width;
	Graph->start_x         = StartX;
	Graph->start_y         = StartY;
	Graph->end_x           = EndX;
	Graph->end_y           = EndY;
}

void UI_Draw_Rectangle(graphic_data *Graph,        //UI图形数据结构体指针
	                     char                   GraphName[3], //图形名 作为客户端的索引
									     uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
									     uint8_t                Layer,        //UI图形图层 [0,9]
							     	 	 uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
							     	   uint16_t               Width,        //线宽
							     		 uint16_t               StartX,       //起始坐标X
							     		 uint16_t               StartY,       //起始坐标Y
							     		 uint16_t               EndX,         //截止坐标X
							     		 uint16_t               EndY)         //截止坐标Y
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Rectangle;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->width           = Width;
	Graph->start_x         = StartX;
	Graph->start_y         = StartY;
	Graph->end_x           = EndX;
	Graph->end_y           = EndY;
}

void UI_Draw_Circle(graphic_data *Graph,        //UI图形数据结构体指针
	                  char                   GraphName[3], //图形名 作为客户端的索引
									  uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
									  uint8_t                Layer,        //UI图形图层 [0,9]
							     	uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
										uint16_t               Width,        //线宽
										uint16_t               CenterX,      //圆心坐标X
							      uint16_t               CenterY,      //圆心坐标Y
										uint16_t               Radius)       //半径
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Circle;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->width           = Width;
	Graph->start_x         = CenterX;
	Graph->start_y         = CenterY;
	Graph->radius          = Radius;
}

void UI_Draw_Ellipse(graphic_data *Graph,        //UI图形数据结构体指针
	                   char                   GraphName[3], //图形名 作为客户端的索引
									   uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
									   uint8_t                Layer,        //UI图形图层 [0,9]
							     	 uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
										 uint16_t               Width,        //线宽
										 uint16_t               CenterX,      //圆心坐标X
							       uint16_t               CenterY,      //圆心坐标Y
										 uint16_t               XHalfAxis,    //X半轴长
										 uint16_t               YHalfAxis)    //Y半轴长
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Ellipse;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->width           = Width;
	Graph->start_x         = CenterX;
	Graph->start_y         = CenterY;
	Graph->end_x           = XHalfAxis;
	Graph->end_y           = YHalfAxis;
}

void UI_Draw_Arc(graphic_data *Graph,        //UI图形数据结构体指针
	               char                   GraphName[3], //图形名 作为客户端的索引
							   uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
								 uint8_t                Layer,        //UI图形图层 [0,9]
							   uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
								 uint16_t               StartAngle,   //起始角度 [0,360]
								 uint16_t               EndAngle,     //截止角度 [0,360]
								 uint16_t               Width,        //线宽
								 uint16_t               CenterX,      //圆心坐标X
							   uint16_t               CenterY,      //圆心坐标Y
								 uint16_t               XHalfAxis,    //X半轴长
								 uint16_t               YHalfAxis)    //Y半轴长
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Arc;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->start_angle     = StartAngle;
	Graph->end_angle       = EndAngle;
	Graph->width           = Width;
	Graph->start_x         = CenterX;
	Graph->start_y         = CenterY;
	Graph->end_x           = XHalfAxis;
	Graph->end_y           = YHalfAxis;
}

void UI_Draw_Float(graphic_data *Graph,        //UI图形数据结构体指针
	                 char                   GraphName[3], //图形名 作为客户端的索引
							     uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
								   uint8_t                Layer,        //UI图形图层 [0,9]
							     uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
									 uint16_t               NumberSize,   //字体大小
									 uint16_t               Significant,  //有效位数
									 uint16_t               Width,        //线宽
							     uint16_t               StartX,       //起始坐标X
							     uint16_t               StartY,       //起始坐标Y
									 float                  FloatData)    //数字内容
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Float;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->start_angle     = NumberSize;
	Graph->end_angle       = Significant;
	Graph->width           = Width;
	Graph->start_x         = StartX;
	Graph->start_y         = StartY;
	int32_t IntData = FloatData * 1000;
	Graph->radius          = (IntData & 0x000003ff) >>  0;
	Graph->end_x           = (IntData & 0x001ffc00) >> 10;
	Graph->end_y           = (IntData & 0xffe00000) >> 21;
}

void UI_Draw_Int(graphic_data *Graph,        //UI图形数据结构体指针
	               char                   GraphName[3], //图形名 作为客户端的索引
							   uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
								 uint8_t                Layer,        //UI图形图层 [0,9]
							   uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
								 uint16_t               NumberSize,   //字体大小
								 uint16_t               Width,        //线宽
							   uint16_t               StartX,       //起始坐标X
							   uint16_t               StartY,       //起始坐标Y
								 int32_t                IntData)      //数字内容
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Int;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->start_angle     = NumberSize;
	Graph->width           = Width;
	Graph->start_x         = StartX;
	Graph->start_y         = StartY;
	Graph->radius          = (IntData & 0x000003ff) >>  0;
	Graph->end_x           = (IntData & 0x001ffc00) >> 10;
	Graph->end_y           = (IntData & 0xffe00000) >> 21;
}

void UI_Draw_String(string_data *String,        //UI图形数据结构体指针
	                  char                  StringName[3], //图形名 作为客户端的索引
							      uint8_t               StringOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
								    uint8_t               Layer,         //UI图形图层 [0,9]
							      uint8_t               Color,         //UI图形颜色 对应UI_Color_XXX的9种颜色
										uint16_t              CharSize,      //字体大小
									  uint16_t              StringLength,  //字符串长度
									  uint16_t              Width,         //线宽
							      uint16_t              StartX,        //起始坐标X
							      uint16_t              StartY,        //起始坐标Y
										char                 *StringData)    //字符串内容
{
	String->string_name[0] = StringName[0];
	String->string_name[1] = StringName[1];
	String->string_name[2] = StringName[2];
	String->operate_tpye   = StringOperate;
	String->graphic_tpye   = UI_Graph_String;
	String->layer          = Layer;
	String->color          = Color;
	String->start_angle    = CharSize;
	String->end_angle      = StringLength;
	String->width          = Width;
	String->start_x        = StartX;
	String->start_y        = StartY;
	for(int i = 0; i < StringLength; i ++) String->stringdata[i] = *StringData ++;
}

/*==============================================================================
              ##### UI完整图案推送函数 #####
  ==============================================================================
    [..]  该部分提供如下函数:
		  (+) 推送图案 UI_PushUp_Graphs
			(+) 推送字符 UI_PushUp_String
			(+) 删除图层 UI_PushUp_Delete
*/
uint16_t test_status;
void UI_PushUp_Graphs(uint8_t Counter /* 1,2,5,7 */, void *Graphs /* 与Counter相一致的UI_Graphx结构体头指针 */, uint8_t RobotID)
{
	UI_Graph1_t *Graph = (UI_Graph1_t *)Graphs; //假设只发一个基本图形
	
	/* 填充 frame_header */
	Graph->Referee_Transmit_Header.SOF  = HEADER_SOF;
	     if(Counter == 1) Graph->Referee_Transmit_Header.data_length = 6 + 1 * 15;
	else if(Counter == 2) Graph->Referee_Transmit_Header.data_length = 6 + 2 * 15;
	else if(Counter == 5) Graph->Referee_Transmit_Header.data_length = 6 + 5 * 15;
	else if(Counter == 7) Graph->Referee_Transmit_Header.data_length = 6 + 7 * 15;
	Graph->Referee_Transmit_Header.seq  = Graph->Referee_Transmit_Header.seq + 1;
	Graph->Referee_Transmit_Header.CRC8 = CRC08_Calculate((uint8_t *)(&Graph->Referee_Transmit_Header), 4);
	
	/* 填充 cmd_id */
	Graph->CMD_ID = STUDENT_INTERACTIVE_DATA_CMD_ID;
	
	/* 填充 student_interactive_header */
	     if(Counter == 1) Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw1;
	else if(Counter == 2) Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw2;
	else if(Counter == 5) Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw5;
	else if(Counter == 7) Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw7;
	Graph->Interactive_Header.sender_ID   = RobotID ;      //当前机器人ID
	Graph->Interactive_Header.receiver_ID = RobotID + 256; //对应操作手ID
	
	/* 填充 frame_tail 即CRC16 */
	     if(Counter == 1)
	{
		UI_Graph1_t *Graph1 = (UI_Graph1_t *)Graphs;
		Graph1->CRC16 = CRC16_Calculate((uint8_t *)Graph1, sizeof(UI_Graph1_t) - 2);
	}
	else if(Counter == 2)
	{
		UI_Graph2_t *Graph2 = (UI_Graph2_t *)Graphs;
		Graph2->CRC16 = CRC16_Calculate((uint8_t *)Graph2, sizeof(UI_Graph2_t) - 2);
	}
	else if(Counter == 5)
	{
		UI_Graph5_t *Graph5 = (UI_Graph5_t *)Graphs;
		Graph5->CRC16 = CRC16_Calculate((uint8_t *)Graph5, sizeof(UI_Graph5_t) - 2);
	}
	else if(Counter == 7)
	{
		UI_Graph7_t *Graph7 = (UI_Graph7_t *)Graphs;
		Graph7->CRC16 = CRC16_Calculate((uint8_t *)Graph7, sizeof(UI_Graph7_t) - 2);
	}
	
	/* 使用串口PushUp到裁判系统 */
	     if(Counter == 1) HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)Graph, sizeof(UI_Graph1_t));
	else if(Counter == 2) HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)Graph, sizeof(UI_Graph2_t));
	else if(Counter == 5) HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)Graph, sizeof(UI_Graph5_t));
	else if(Counter == 7) HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)Graph, sizeof(UI_Graph7_t));
	test_status = HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)Graph, sizeof(UI_Graph1_t));
}





void UI_PushUp_String(UI_String_t *String, uint8_t RobotID)
{
	/* 填充 frame_header */
	String->Referee_Transmit_Header.SOF  = HEADER_SOF;
	String->Referee_Transmit_Header.data_length = 6 + 45;
	String->Referee_Transmit_Header.seq  = String->Referee_Transmit_Header.seq + 1;
	String->Referee_Transmit_Header.CRC8 = CRC08_Calculate((uint8_t *)(&String->Referee_Transmit_Header), 4);
	
	/* 填充 cmd_id */
	String->CMD_ID = STUDENT_INTERACTIVE_DATA_CMD_ID;
	
	/* 填充 student_interactive_header */
	String->Interactive_Header.data_cmd_id = UI_DataID_DrawChar;
	String->Interactive_Header.sender_ID   = RobotID ;      //当前机器人ID
	String->Interactive_Header.receiver_ID = RobotID + 256; //对应操作手ID
	
	/* 填充 frame_tail 即CRC16 */
	String->CRC16 = CRC16_Calculate((uint8_t *)String, sizeof(UI_String_t) - 2);
	
	/* 使用串口PushUp到裁判系统 */
	HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)String, sizeof(UI_String_t));
}

void UI_PushUp_Delete(UI_Delete_t *Delete, uint8_t RobotID)
{
	/* 填充 frame_header */
	Delete->Referee_Transmit_Header.SOF  = HEADER_SOF;
	Delete->Referee_Transmit_Header.data_length = 6 + 2;
	Delete->Referee_Transmit_Header.seq  = Delete->Referee_Transmit_Header.seq + 1;
	Delete->Referee_Transmit_Header.CRC8 = CRC08_Calculate((uint8_t *)(&Delete->Referee_Transmit_Header), 4);
	
	/* 填充 cmd_id */
	Delete->CMD_ID = STUDENT_INTERACTIVE_DATA_CMD_ID;
	
	/* 填充 student_interactive_header */
	Delete->Interactive_Header.data_cmd_id = UI_DataID_Delete;
	Delete->Interactive_Header.sender_ID   = RobotID ;      //当前机器人ID
	Delete->Interactive_Header.receiver_ID = RobotID + 256; //对应操作手ID
	
	/* 填充 frame_tail 即CRC16 */
	Delete->CRC16 = CRC16_Calculate((uint8_t *)Delete, sizeof(UI_Delete_t) - 2);
	
	/* 使用串口PushUp到裁判系统 */
	HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)Delete, sizeof(UI_Delete_t));
}





