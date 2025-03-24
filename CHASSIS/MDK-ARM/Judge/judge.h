#ifndef __judge_h
#define __judge_h

#include "main.h"
#include "stdint.h"
#include "fifo.h"



/* Defines -------------------------------------------------------------------*/

extern uint8_t Judge_Self_ID;//当前机器人的ID
extern uint16_t Judge_SelfClient_ID;//发送者机器人对应的客户端ID
//定义帧头
//起始字节,协议固定为0xA5
#define   Judge_Data_SOF 0xA5


#define   JUDGE_DATA_ERROR      0
#define   JUDGE_DATA_CORRECT    1

#define 	LEN_HEADER 	  5				//帧头长
#define   LEN_CMDID     2       //命令码长度
#define   LEN_TAIL      2	      //帧尾CRC16

typedef __packed struct
{
    uint8_t SOF;
    uint16_t Data_Length;
    uint8_t Seq;
    uint8_t CRC8;

} frame_header_t;

typedef enum
{
    FRAME_HEADER         = 0,
    CMD_ID               = 5,
    DATA                 = 7,
    STU_HEADER					 = 7,
    STU_DATA             = 13
} JudgeFrameOffset;

//5字节帧头,偏移位置
typedef enum
{
    SOF          = 0,//起始位
    DATA_LENGTH  = 1,//帧内数据长度,根据这个来获取数据长度
    SEQ          = 3,//包序号
    CRC8         = 4 //CRC8
} frame_header_OFFSET;






#define Referee_UART huart6
#define Referee_IRQHandler USART6_IRQHandler

#define REFEREE_USART_RX_BUF_LENGHT 512
#define REFEREE_FIFO_BUF_LENGTH     1024




#define HEADER_SOF                  0xA5

#define REF_PROTOCOL_FRAME_MAX_SIZE 128
#define REF_PROTOCOL_HEADER_SIZE    sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE       2
#define REF_PROTOCOL_CRC16_SIZE     2

#define REF_HEADER_CRC_LEN          (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN    (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN        (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#pragma pack(push, 1)

typedef enum
{
	GAME_STATE_CMD_ID                 = 0x0001,  //比赛状态数据
	GAME_RESULT_CMD_ID                = 0x0002,  //比赛结果数据
	GAME_ROBOT_HP_CMD_ID              = 0x0003,  //机器人血量数据
	DART_FLYING_STATE_CMD_ID          = 0x0004,  //飞镖发射后状态
	
	FIELD_EVENTS_CMD_ID               = 0x0101,  //场地事件数据
	SUPPLY_PROJECTILE_ACTION_CMD_ID   = 0x0102,  //补给站动作标识
	SUPPLY_PROJECTILE_BOOKING_CMD_ID  = 0x0103,  //请求补给站补弹数据，由参赛队发送（RM 对抗赛尚未开放）
	REFEREE_WARNING_CMD_ID            = 0x0104,  //裁判警告信息
	DART_REMAINING_TIME_CMD_ID        = 0x0105,  //飞镖发射口倒计时
	
	ROBOT_STATE_CMD_ID                = 0x0201,  //比赛机器人状态
	POWER_HEAT_DATA_CMD_ID            = 0x0202,  //实时功率热量数据
	ROBOT_POS_CMD_ID                  = 0x0203,  //机器人位置
	BUFF_MUSK_CMD_ID                  = 0x0204,  //机器人增益
	AERIAL_ROBOT_ENERGY_CMD_ID        = 0x0205,  //空中机器人能量状态
	ROBOT_HURT_CMD_ID                 = 0x0206,  //伤害状态
	SHOOT_DATA_CMD_ID                 = 0x0207,  //实时射击信息
	BULLET_REMAINING_CMD_ID           = 0x0208,  //子弹剩余发射数
	ROBOT_RFID_STATE_CMD_ID           = 0x0209,  //机器人RFID状态
	DART_CLIENT_CMD_ID                = 0x020A,  //飞镖机器人客户端指令数据
	ROBOT_POSITION_DM_ID              = 0x020B,  //哨兵地面机器人位置数据  
	
	STUDENT_INTERACTIVE_DATA_CMD_ID   = 0x0301,  //机器人间通信
  ROBOT_COMMAND_CMD_ID              = 0x0303,  //小地图下发信息标识
  CLIENT_MAP_COMMAND_CMD_ID         = 0x0305,  //小地图接收信息标识
	
	IDCustomData,
}referee_cmd_id_e;

typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

typedef __packed struct
{
  uint8_t  SOF;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  CRC8;
} frame_header_struct_t;

typedef __packed struct
{
  frame_header_struct_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

#pragma pack(pop)


/* Referee Defines -----------------------------------------------------------*/
/* 比赛类型 */
#define Game_Type_RMUC     1 //超级对抗赛
#define Game_Type_RMUT     2 //单项赛
#define Game_Type_RMUA     3 //人工智能挑战赛
#define Game_Type_RMUL_3V3 4 //高校联盟赛3V3
#define Game_Type_RMUL_1V1 5 //高校联盟赛1V1

/* 比赛阶段 */
#define Game_Progress_Unstart   0 //未开始比赛
#define Game_Progress_Prepare   1 //准备阶段
#define Game_Progress_SelfCheck 2 //自检阶段
#define Game_Progress_5sCount   3 //5s倒计时
#define Game_Progress_Battle    4 //对战中
#define Game_Progress_Calculate 5 //比赛结算中

/* 比赛结果 */
#define Game_Result_Draw    0 //平局
#define Game_Result_RedWin  1 //红方胜利
#define Game_Result_BlueWin 2 //蓝方胜利

/* 警告信息 */
#define Warning_Yellow  1 //黄牌警告
#define Warning_Red     2 //红牌警告
#define Warning_Failure 3 //判负

/* 机器人ID */
#define Robot_ID_Red_Hero         1 //红方英雄
#define Robot_ID_Red_Engineer     2 //红方工程
#define Robot_ID_Red_Infantry3    3 //红方步兵3
#define Robot_ID_Red_Infantry4    4 //红方步兵4
#define Robot_ID_Red_Infantry5    5 //红方步兵5
#define Robot_ID_Red_Aerial       6 //红方无人机
#define Robot_ID_Red_Sentry       7 //红方哨兵
#define Robot_ID_Red_Darts        8 //红方飞镖
#define Robot_ID_Red_Radar        9 //红方雷达
#define Robot_ID_Blue_Hero      101 //蓝方英雄
#define Robot_ID_Blue_Engineer  102 //蓝方工程
#define Robot_ID_Blue_Infantry3 103 //蓝方步兵3
#define Robot_ID_Blue_Infantry4 104 //蓝方步兵4
#define Robot_ID_Blue_Infantry5 105 //蓝方步兵5
#define Robot_ID_Blue_Aerial    106 //蓝方无人机
#define Robot_ID_Blue_Sentry    107 //蓝方哨兵
#define Robot_ID_Blue_Darts     108 //蓝方飞镖
#define Robot_ID_Blue_Radar     109 //蓝方雷达

/* 机器人等级 */
#define Robot_Level_1 1 //1级
#define Robot_Level_2 2 //2级
#define Robot_Level_3 3 //3级

/* 扣血类型 */
#define Hurt_Type_ArmoredPlate     0 //装甲板伤害
#define Hurt_Type_ModuleOffline    1 //模块离线
#define Hurt_Type_OverShootSpeed   2 //枪口超射速
#define Hurt_Type_OverShootHeat    3 //枪管超热量
#define Hurt_Type_OverChassisPower 4 //底盘超功率
#define Hurt_Type_Collision        5 //装甲撞击

/* 发射机构编号 */
#define Shooter_ID1_17mm 1 //1号17mm发射机构
#define Shooter_ID2_17mm 2 //2号17mm发射机构
#define Shooter_ID1_42mm 3 //1号42mm发射机构

/* 飞镖信息 */
#define Dart_State_Open     0 //飞镖闸门开启
#define Dart_State_Close    1 //飞镖闸门关闭
#define Dart_State_Changing 2 //正在开启或者关闭中
#define Dart_Target_Outpost 0 //飞镖目标为前哨站
#define Dart_Target_Base    1 //飞镖目标为基地

/* 操作手ID */
#define Cilent_ID_Red_Hero       0x0101 //红方英雄操作手
#define Cilent_ID_Red_Engineer   0x0102 //红方工程操作手
#define Cilent_ID_Red_Infantry3  0x0103 //红方步兵3操作手
#define Cilent_ID_Red_Infantry4  0x0104 //红方步兵4操作手
#define Cilent_ID_Red_Infantry5  0x0105 //红方步兵5操作手
#define Cilent_ID_Red_Aerial     0x0106 //红方飞手
#define Cilent_ID_Blue_Hero      0x0165 //蓝方英雄操作手
#define Cilent_ID_Blue_Engineer  0x0166 //蓝方工程操作手
#define Cilent_ID_Blue_Infantry3 0x0167 //蓝方步兵3操作手
#define Cilent_ID_Blue_Infantry4 0x0168 //蓝方步兵4操作手
#define Cilent_ID_Blue_Infantry5 0x0169 //蓝方步兵5操作手
#define Cilent_ID_Blue_Aerial    0x016A //蓝方飞手

typedef __packed struct  //接收频率
{
	uint16_t RX_Frequent;
	uint16_t RX_add;
} rx_msg;


/* 0x000X --------------------------------------------------------------------*/
typedef __packed struct  //0x0001 比赛状态数据
{
	uint8_t  game_type : 4;
	uint8_t  game_progress : 4;
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
} ext_game_status_t;

typedef __packed struct  //0x0002 比赛结果数据
{
	uint8_t winner;
} ext_game_result_t;

typedef __packed struct  //0x0003 机器人血量数据
{
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP;
	uint16_t red_3_robot_HP;
	uint16_t red_4_robot_HP;
	uint16_t red_5_robot_HP;
	uint16_t red_7_robot_HP;
	uint16_t red_outpost_HP;	
	uint16_t red_base_HP;
	uint16_t blue_1_robot_HP;
	uint16_t blue_2_robot_HP;
	uint16_t blue_3_robot_HP;
	uint16_t blue_4_robot_HP;
	uint16_t blue_5_robot_HP;
	uint16_t blue_7_robot_HP;
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
} ext_game_robot_HP_t;

/* 0x010X --------------------------------------------------------------------*/
typedef __packed struct  //0x0101 场地事件数据
{
  uint32_t event_type;
} ext_event_data_t;

typedef __packed struct  //0x0102 补给站动作标识
{
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
	uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

typedef __packed struct //0x0103 请求补给站补弹数据，由参赛队发送（RM 对抗赛尚未开放）
{
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_num;
} ext_supply_projectile_booking_t;

typedef __packed struct  //0x0104 裁判警告信息
{
	uint8_t level;
	uint8_t foul_robot_id;
} ext_referee_warning_t;

typedef __packed struct  //0x0105 飞镖发射口倒计时
{
	uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

/* 0x020X --------------------------------------------------------------------*/
typedef __packed struct  //0x0201 比赛机器人状态
{
	uint8_t  robot_id;
	uint8_t  robot_level;
	uint16_t current_HP; 
	uint16_t maximum_HP;
	uint16_t shooter_barrel_cooling_value;
	uint16_t shooter_barrel_heat_limit;
	uint16_t chassis_power_limit;
	uint8_t  power_management_gimbal_output : 1;
	uint8_t  power_management_chassis_output : 1;
	uint8_t  power_management_shooter_output : 1;
} ext_game_robot_state_t;

typedef __packed struct  //0x0202 实时功率热量数据
{
	uint16_t chassis_volt;
	uint16_t chassis_current;
	float    chassis_power;
	uint16_t chassis_power_buffer;
	uint16_t shooter_id1_17mm_cooling_heat;
	uint16_t shooter_id2_17mm_cooling_heat;
	uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

typedef __packed struct  //0x0203 机器人位置
{
	float x;
	float y;
	float z;
	float angle;
} ext_game_robot_pos_t;

typedef __packed struct  //0x0204 机器人增益
{

 uint8_t recovery_buff;
 uint8_t cooling_buff;
 uint8_t defence_buff;
 uint8_t vulnerability_buff;
 uint16_t attack_buff;
} ext_buff_musk_t;

typedef __packed struct  //0x0205 空中机器人能量状态
{
	uint8_t attack_time;
} aerial_robot_energy_t;

typedef __packed struct  //0x0206 伤害状态
{
	uint8_t armor_type : 4;
	uint8_t hurt_type  : 4;
} ext_robot_hurt_t;

typedef __packed struct  //0x0207 实时射击信息
{
	uint8_t bullet_type;
	uint8_t shooter_id;
	uint8_t bullet_freq;
	float   bullet_speed;
	rx_msg RX_MSG;
} ext_shoot_data_t;

typedef __packed struct  //0x0208 子弹剩余发射数
{
	uint16_t bullet_remaining_num_17mm;
	uint16_t bullet_remaining_num_42mm;
	uint16_t coin_remaining_num;
} ext_bullet_remaining_t;

typedef __packed struct  //0x0209 机器人RFID状态
{
	uint32_t rfid_status;
} ext_rfid_status_t;

typedef __packed struct  //0x020A 飞镖机器人客户端指令数据
{
	uint8_t  dart_launch_opening_status;
	uint8_t  dart_attack_target;
	uint16_t target_change_time;
	uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;

/* 0x030X --------------------------------------------------------------------*/
typedef __packed struct  //0x0301 机器人间通信 头结构体
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

typedef __packed struct  //0x0301 机器人间通信 数据结构体
{
	uint8_t *data;
} robot_interactive_data_t;

typedef __packed struct  //0x0303 小地图下发信息标识
{
	float target_position_x;
	float target_position_y;
	uint8_t cmd_keyboard;
	uint8_t target_robot_id;
	uint8_t cmd_source;
} ext_robot_command_t;

typedef __packed struct  //0x0305 小地图接收信息标识
{
	uint16_t target_robot_ID;
	float    target_position_x;
	float    target_position_y;
} ext_client_map_command_t;


/* Structs -------------------------------------------------------------------*/
/* protocol包头结构体 */
extern frame_header_struct_t Referee_Receive_Header;

/* 0x000X */
extern ext_game_status_t   Game_Status;
extern ext_game_result_t   Game_Result;
extern ext_game_robot_HP_t Game_Robot_HP;

/* 0x010X */
extern ext_event_data_t                Event_Data;
extern ext_supply_projectile_action_t  Supply_Projectile_Action;
extern ext_supply_projectile_booking_t Supply_Projectile_Booking;
extern ext_referee_warning_t           Referee_Warning;
extern ext_dart_remaining_time_t       Dart_Remaining_Time;

/* 0x020X */
extern ext_game_robot_state_t Game_Robot_State;
extern ext_power_heat_data_t  Power_Heat_Data;
extern ext_game_robot_pos_t   Game_Robot_Pos;
extern ext_buff_musk_t        Buff_Musk;
extern aerial_robot_energy_t  Aerial_Robot_Energy;
extern ext_robot_hurt_t       Robot_Hurt;
extern ext_shoot_data_t       Shoot_Data;
extern ext_bullet_remaining_t Bullet_Remaining;
extern ext_rfid_status_t      RFID_Status;
extern ext_dart_client_cmd_t  Dart_Client_Cmd;

/* 0x030X */
extern ext_student_interactive_header_data_t Student_Interactive_Header_Data;
extern robot_interactive_data_t              Robot_Interactive_Data;
extern ext_robot_command_t                   Robot_Command;
extern ext_client_map_command_t              Client_Map_Command;

/* 裁判系统串口双缓冲区 */
extern uint8_t Referee_Buffer[2][REFEREE_USART_RX_BUF_LENGHT];

/* 裁判系统接收数据队列 */
extern fifo_s_t Referee_FIFO;
extern uint8_t Referee_FIFO_Buffer[REFEREE_FIFO_BUF_LENGTH];

/* protocol解析包结构体 */
extern unpack_data_t Referee_Unpack_OBJ;

/* Functions -----------------------------------------------------------------*/
void Referee_StructInit(void);
void Referee_UARTInit(uint8_t *Buffer0, uint8_t *Buffer1, uint16_t BufferLength);
void Referee_SolveFifoData(uint8_t *frame);
void Referee_UnpackFifoData(unpack_data_t *p_obj, fifo_s_t *referee_fifo);
void USART6_IRQHandler_1(void);




//定义帧
typedef __packed struct
{
    frame_header_t frame_header;
    uint16_t cmd_id;
    referee_cmd_id_e data;
    uint16_t  frame_tail;	//crc16整皝EＱ丒
}frame_t;
/**********************************学生机器人之间的通信******************************************/
typedef enum
{
    LEN_SEVEN_GRAPH = 105,
    LEN_STU_HEAD = 6,
    LEN_SINGLE_GRAPH = 15
}length_stu_to_judge;



typedef enum
{
    absolute_position_id						 					= 0x0200,
    radar_enermies_position_id										= 0x0201,
    aerial_enermies_position_id										= 0x0202,

    client_custom_graphic_delete_id									= 0x0100,
    client_custom_graphic_single_id									= 0x0101,
    client_custom_graphic_double_id									= 0x0102,
    client_custom_graphic_five_id									= 0x0103,
    client_custom_graphic_seven_id									= 0x0104,
    client_custom_character_id										= 0x0110
}data_cmd_id_t;



typedef struct
{
    frame_header_t frame_header;																					//帧头
    uint16_t cmd_id;																											//命令码
    ext_student_interactive_header_data_t student_interactive_header_data;//数据段头结构
    uint8_t student_interactive_data[20];																//数据内容
    uint16_t frame_tail;																									//帧尾（16位crc校验）
}send_to_teammate;


bool Judge_Read_Data(uint8_t *ReadFromUsart);
void JUDGE_Show_Data(void);
void Send_to_Teammate(void);
bool is_red_or_blue(void);
uint16_t determine_ID(void);
bool JUDGE_sGetDataState(void);
float JUDGE_fGetChassisPower(void);
uint8_t JUDGE_usGetPowerLimit(void);
uint16_t JUDGE_fGetRemainEnergy(void);
uint8_t JUDGE_ucGetRobotLevel(void);
uint16_t JUDGE_usGetRemoteHeat42(void);
float JUDGE_usGetSpeed42(void);
void JUDGE_ShootNumCount(void);
uint16_t JUDGE_usGetShootNum(void);
void JUDGE_ShootNum_Clear(void);
uint16_t JUDGE_usGetHeatLimit(void);
uint8_t JUDGE_usGetSpeedLimit(void);
uint16_t JUDGE_usGetShootCold(void);
bool JUDGE_IfArmorHurt(void);
bool Judge_If_Death(void);
bool Judge_If_Near_Death(void);
uint16_t choose_client(uint8_t robot_id);
uint8_t JUDGE_GameState(void);


#endif
