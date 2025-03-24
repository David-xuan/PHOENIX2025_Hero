#ifndef __SQ_tuxin_h
#define __SQ_tuxin_h

#include "judge.h"

/* UI绘制内容cmdID */
#define UI_DataID_Delete   0x100 //客户端删除图形
#define UI_DataID_Draw1    0x101 //客户端绘制1个图形
#define UI_DataID_Draw2    0x102 //客户端绘制2个图形
#define UI_DataID_Draw5    0x103 //客户端绘制5个图形
#define UI_DataID_Draw7    0x104 //客户端绘制7个图形
#define UI_DataID_DrawChar 0x110 //客户端绘制字符图形

/* UI删除操作 */
#define UI_Delete_Invalid 0 //空操作
#define UI_Delete_Layer   1 //删除图层
#define UI_Delete_All     2 //删除所有

/* UI图形操作 */
#define UI_Graph_invalid 0 //空操作
#define UI_Graph_Add     1 //增加图形
#define UI_Graph_Change  2 //修改图形
#define UI_Graph_Delete  3 //删除图形

/* UI图形类型 */
#define UI_Graph_Line      0 //直线
#define UI_Graph_Rectangle 1 //矩形
#define UI_Graph_Circle    2 //整圆
#define UI_Graph_Ellipse   3 //椭圆
#define UI_Graph_Arc       4 //圆弧
#define UI_Graph_Float     5 //浮点型
#define UI_Graph_Int       6 //整形
#define UI_Graph_String    7 //字符型

/* UI图形颜色 */
#define UI_Color_Main   0 //红蓝主色
#define UI_Color_Yellow 1 //黄色
#define UI_Color_Green  2 //绿色
#define UI_Color_Orange 3 //橙色
#define UI_Color_Purple 4 //紫色
#define UI_Color_Pink   5 //粉色
#define UI_Color_Cyan   6 //青色
#define UI_Color_Black  7 //黑色
#define UI_Color_White  8 //白色






/* 自定义绘制UI结构体 -------------------------------------------------------*/
typedef __packed struct  //绘制UI UI图形数据
{
	uint8_t  graphic_name[3];
	uint32_t operate_tpye:3;
	uint32_t graphic_tpye:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	uint32_t radius:10;
	uint32_t end_x:11;
	uint32_t end_y:11;
} graphic_data;

typedef __packed struct  //绘制UI UI字符串数据
{
	uint8_t  string_name[3];
	uint32_t operate_tpye:3;
	uint32_t graphic_tpye:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	uint32_t null;
	uint8_t stringdata[30];
} string_data;

typedef __packed struct  //绘制UI UI删除图形数据
{
	uint8_t operate_tpye;
	uint8_t layer;
} delete_data_struct_t;

typedef __packed struct //绘制UI 绘制1个图形完整结构体
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Interactive_Header;
	graphic_data Graphic[1];
	uint16_t CRC16;
} UI_Graph1_t;

typedef __packed struct //绘制UI 绘制2个图形完整结构体
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Interactive_Header;
	graphic_data Graphic[2];
	uint16_t CRC16;
} UI_Graph2_t;

typedef __packed struct //绘制UI 绘制5个图形完整结构体
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Interactive_Header;
	graphic_data Graphic[5];
	uint16_t CRC16;
} UI_Graph5_t;

typedef __packed struct //绘制UI 绘制7个图形完整结构体
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Interactive_Header;
	graphic_data Graphic[7];
	uint16_t CRC16;
} UI_Graph7_t;

typedef __packed struct //绘制UI 绘制1字符串完整结构体
{ 
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Interactive_Header;
	string_data String;
	uint16_t CRC16;
} UI_String_t;

typedef __packed struct  //绘制UI UI删除图形完整结构体
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Interactive_Header;
	delete_data_struct_t Delete;
	uint16_t CRC16;
} UI_Delete_t;


/* 绘制UI专用结构体 */
extern UI_Graph1_t UI_Graph1;
extern UI_Graph2_t UI_Graph2;
extern UI_Graph5_t UI_Graph5;
extern UI_Graph7_t UI_Graph7;
extern UI_String_t UI_String;
extern UI_Delete_t UI_Delete;


extern void UI_Draw_Line(graphic_data *Graph,        //UI图形数据结构体指针
	                char                   GraphName[3], //图形名 作为客户端的索引
									uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
									uint8_t                Layer,        //UI图形图层 [0,9]
									uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
									uint16_t               Width,        //线宽
									uint16_t               StartX,       //起始坐标X
									uint16_t               StartY,       //起始坐标Y
									uint16_t               EndX,         //截止坐标X
									uint16_t               EndY);        //截止坐标Y
extern void UI_Draw_Rectangle(graphic_data *Graph,        //UI图形数据结构体指针
	                     char                   GraphName[3], //图形名 作为客户端的索引
									     uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
									     uint8_t                Layer,        //UI图形图层 [0,9]
							     	 	 uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
							     	   uint16_t               Width,        //线宽
							     		 uint16_t               StartX,       //起始坐标X
							     		 uint16_t               StartY,       //起始坐标Y
							     		 uint16_t               EndX,         //截止坐标X
							     		 uint16_t               EndY);        //截止坐标Y
extern void UI_Draw_Circle(graphic_data *Graph,        //UI图形数据结构体指针
	                  char                   GraphName[3], //图形名 作为客户端的索引
									  uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
									  uint8_t                Layer,        //UI图形图层 [0,9]
							     	uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
										uint16_t               Width,        //线宽
										uint16_t               CenterX,      //圆心坐标X
							      uint16_t               CenterY,      //圆心坐标Y
										uint16_t               Radius);      //半径
extern void UI_Draw_Ellipse(graphic_data *Graph,        //UI图形数据结构体指针
	                   char                   GraphName[3], //图形名 作为客户端的索引
									   uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
									   uint8_t                Layer,        //UI图形图层 [0,9]
							     	 uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
										 uint16_t               Width,        //线宽
										 uint16_t               CenterX,      //圆心坐标X
							       uint16_t               CenterY,      //圆心坐标Y
										 uint16_t               XHalfAxis,    //X半轴长
										 uint16_t               YHalfAxis);   //Y半轴长
extern void UI_Draw_Arc(graphic_data *Graph,        //UI图形数据结构体指针
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
								 uint16_t               YHalfAxis);   //Y半轴长
extern void UI_Draw_Float(graphic_data *Graph,        //UI图形数据结构体指针
	                 char                   GraphName[3], //图形名 作为客户端的索引
							     uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
								   uint8_t                Layer,        //UI图形图层 [0,9]
							     uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
									 uint16_t               NumberSize,   //字体大小
									 uint16_t               Significant,  //有效位数
									 uint16_t               Width,        //线宽
							     uint16_t               StartX,       //起始坐标X
							     uint16_t               StartY,       //起始坐标Y
									 float                  FloatData);   //数字内容
extern void UI_Draw_Int(graphic_data *Graph,        //UI图形数据结构体指针
	               char                   GraphName[3], //图形名 作为客户端的索引
							   uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
								 uint8_t                Layer,        //UI图形图层 [0,9]
							   uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
								 uint16_t               NumberSize,   //字体大小
								 uint16_t               Width,        //线宽
							   uint16_t               StartX,       //起始坐标X
							   uint16_t               StartY,       //起始坐标Y
								 int32_t                IntData);     //数字内容
extern void UI_Draw_String(string_data *String,        //UI图形数据结构体指针
	                  char                  StringName[3], //图形名 作为客户端的索引
							      uint8_t               StringOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
								    uint8_t               Layer,         //UI图形图层 [0,9]
							      uint8_t               Color,         //UI图形颜色 对应UI_Color_XXX的9种颜色
										uint16_t              CharSize,      //字体大小
									  uint16_t              StringLength,  //字符串长度
									  uint16_t              Width,         //线宽
							      uint16_t              StartX,        //起始坐标X
							      uint16_t              StartY,        //起始坐标Y
										char                 *StringData);   //字符串内容

extern void UI_PushUp_Graphs(uint8_t Counter, void *Graphs, uint8_t RobotID);
extern void UI_PushUp_String(UI_String_t *String, uint8_t RobotID);
extern void UI_PushUp_Delete(UI_Delete_t *Delete, uint8_t RobotID);

#endif

