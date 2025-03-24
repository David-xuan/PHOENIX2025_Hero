#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "stm32f4xx.h"
#include "pid.h"
#include "main.h"
#include <stdbool.h>

#define CANID_CH 				0x320
#define CANID_SW 				0x321
#define CANID_MOUSE				0x322
#define CANID_yaw               0x102
#define P_MIN -3.141593
#define P_MAX 3.141593
#define V_MIN -30.0
#define V_MAX 30.0
#define KP_MIN 0.0
#define KP_MAX 500.0
#define KD_MIN 0.0
#define KD_MAX 5.0
#define T_MIN -10.0
#define T_MAX 10.0
#define PI               3.14159265358979f

/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)0x01 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)

typedef __packed struct
{
        __packed struct
        {
                int16_t ch[5];
                char s[2];
				short wheel;
        } rc;
        __packed struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
        __packed struct
        {
                uint16_t v;
        } key;
		__packed struct
		{
			bool CTRL;
			bool F;
			bool X;
			bool C;
			bool Q;
			bool E;
			bool R;
		}flag;//��־λ�������л�ģʽ����֤����˲��ֻ���뺯��һ��

		uint32_t Rx_add[3];
		uint16_t Rx_Frequency[3];
} RC_ctrl_t;
extern RC_ctrl_t rc_ctrl;
typedef struct
{
	float front;
	float right;
	float pitch;
	float yaw;
}Direc_Struct;

typedef struct
{
	int16_t motor1;
	int16_t motor2;
	int16_t motor3;
	int16_t motor4;
}set_t;
//�趨ֵ


typedef struct
{
	uint16_t angle;
	int16_t speed;
	int16_t current;
	uint8_t temperature;
	uint8_t null;
}fdb_t;
//����ֵ

typedef struct
{
    union
    {
        uint8_t buff[4];
        float value;
		
    }Yaw_angle;//������yawֵ
    union
    {
        uint8_t buff[4];
        float value;
		
    }vison_yaw;	
	union
	{
		int8_t buff[4];
		float value;
	}Yaw_4310;//���̷���yawֵ
	union
	{
		int8_t buff[4];
		float value;
	}Pitch_angle;//������pitchֵ
}angle_measure_t;

/* ��ȡ���������ƶ��ٶ� */
#define    MOUSE_X_MOVE_SPEED    (rc_ctrl.mouse.x)
#define    MOUSE_Y_MOVE_SPEED    (rc_ctrl.mouse.y)
#define    MOUSE_Z_MOVE_SPEED    (rc_ctrl.mouse.z)


/* �����갴��״̬
   ����Ϊ1��û����Ϊ0*/
#define    IF_MOUSE_PRESSED_LEFT    (rc_ctrl.mouse.press_l == 1)
#define    IF_MOUSE_PRESSED_RIGH    (rc_ctrl.mouse.press_r == 1)


/* �����̰���״̬
   ����Ӧ���������£����߼����ʽ��ֵΪ1������Ϊ0 */
#define    IF_KEY_PRESSED         (  rc_ctrl.key.v  )
#define    IF_KEY_PRESSED_W       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_W)    != 0 )
#define    IF_KEY_PRESSED_S       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_S)    != 0 )
#define    IF_KEY_PRESSED_A       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_A)    != 0 )
#define    IF_KEY_PRESSED_D       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_D)    != 0 )
#define    IF_KEY_PRESSED_Q       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)    != 0 )
#define    IF_KEY_PRESSED_E       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_E)    != 0 )
#define    IF_KEY_PRESSED_G       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_G)    != 0 )
#define    IF_KEY_PRESSED_X       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_X)    != 0 )
#define    IF_KEY_PRESSED_Z       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_Z)    != 0 )
#define    IF_KEY_PRESSED_C       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_C)    != 0 )
#define    IF_KEY_PRESSED_B       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_B)    != 0 )
#define    IF_KEY_PRESSED_V       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_V)    != 0 )
#define    IF_KEY_PRESSED_F       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_F)    != 0 )
#define    IF_KEY_PRESSED_R       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_R)    != 0 )
#define    IF_KEY_PRESSED_CTRL    ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_CTRL) != 0 )
#define    IF_KEY_PRESSED_SHIFT   ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT) != 0 )

#define    IF_KEY_PRESSED_LAST         (  rc_ctrl.key.v_last  )
#define    IF_KEY_PRESSED_W_LAST       ( (rc_ctrl.key.v_last & KEY_PRESSED_OFFSET_W)    != 0 )
#define    IF_KEY_PRESSED_S_LAST       ( (rc_ctrl.key.v_last & KEY_PRESSED_OFFSET_S)    != 0 )
#define    IF_KEY_PRESSED_A_LAST       ( (rc_ctrl.key.v_last & KEY_PRESSED_OFFSET_A)    != 0 )
#define    IF_KEY_PRESSED_D_LAST       ( (rc_ctrl.key.v_last & KEY_PRESSED_OFFSET_D)    != 0 )
#define    IF_KEY_PRESSED_Q_LAST       ( (rc_ctrl.key.v_last & KEY_PRESSED_OFFSET_Q)    != 0 )
#define    IF_KEY_PRESSED_E_LAST       ( (rc_ctrl.key.v_last & KEY_PRESSED_OFFSET_E)    != 0 )
#define    IF_KEY_PRESSED_G_LAST       ( (rc_ctrl.key.v_last & KEY_PRESSED_OFFSET_G)    != 0 )
#define    IF_KEY_PRESSED_X_LAST       ( (rc_ctrl.key.v_last & KEY_PRESSED_OFFSET_X)    != 0 )
#define    IF_KEY_PRESSED_Z_LAST       ( (rc_ctrl.key.v_last & KEY_PRESSED_OFFSET_Z)    != 0 )
#define    IF_KEY_PRESSED_C_LAST       ( (rc_ctrl.key.v_last & KEY_PRESSED_OFFSET_C)    != 0 )
#define    IF_KEY_PRESSED_B_LAST       ( (rc_ctrl.key.v_last & KEY_PRESSED_OFFSET_B)    != 0 )
#define    IF_KEY_PRESSED_V_LAST       ( (rc_ctrl.key.v_last & KEY_PRESSED_OFFSET_V)    != 0 )
#define    IF_KEY_PRESSED_F_LAST       ( (rc_ctrl.key.v_last & KEY_PRESSED_OFFSET_F)    != 0 )
#define    IF_KEY_PRESSED_R_LAST       ( (rc_ctrl.key.v_last & KEY_PRESSED_OFFSET_R)    != 0 )
#define    IF_KEY_PRESSED_CTRL_LAST    ( (rc_ctrl.key.v_last & KEY_PRESSED_OFFSET_CTRL) != 0 )
#define    IF_KEY_PRESSED_SHIFT_LAST   ( (rc_ctrl.key.v_last & KEY_PRESSED_OFFSET_SHIFT) != 0 )

void TransTxMessage(set_t *set, uint8_t *Txdata);//�������ݴ�������
void TransRxMessage(fdb_t *fdb, uint8_t *Rxdata);//�������ݴ������
void CAN_Tx(CAN_HandleTypeDef *hcan, uint32_t *stdid, set_t *set);//�����趨ֵ
void CAN_Rx(CAN_HandleTypeDef *hcan, uint32_t *stdid, fdb_t *fdb);//���շ���ֵ
void Set_Transmit(CAN_HandleTypeDef *hcan, Motor_struct *motor);//��pid���ֵ����
void Set_Transmit_Gimbal(CAN_HandleTypeDef *hcan, Motor_struct *motor);//��pid���ֵ����
void Get_Fdb(CAN_HandleTypeDef *hcan, Motor_struct *motor);//����ֵ�浽.now

void can_filter_init(void);
void can_filter_init_recv_all(CAN_HandleTypeDef* _hcan);

void set_gimbal_angle(CAN_HandleTypeDef* hcan, float angle);
void set_gimbal_flag(CAN_HandleTypeDef* hcan,float angle1,float angle2);

void Frequency_calc(void);

#endif

