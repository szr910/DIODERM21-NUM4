#ifndef __BSP_CAN
#define __BSP_CAN
#include "main.h"
#include "can.h"

/* �������ID ����*/
#define	CAN1_M3508_ID1 0x201	// ���̵��1��ǰ
#define	CAN1_M3508_ID2 0x202	// ���̵��2��ǰ
#define	CAN1_M3508_ID3 0x203	// ���̵��3���
#define	CAN1_M3508_ID4 0x204	// ���̵��4�Һ�
#define CAN1_GM6020_ID1 0x209	// ��̨�ײ����
#define CAN1_GM6020_ID2 0x20A	// ��̨������

#define	CAN2_M3508_ID1 0x201	// Ħ���ֵ�� ����
#define	CAN2_M3508_ID2 0x202	// Ħ���ֵ�� ����
#define	CAN2_M3508_ID3 0x203	// Ħ���ֵ�� ����  
#define	CAN2_M3508_ID4 0x204	// Ħ���ֵ�� ����
#define CAN2_M2006_ID1 0x205	// ���̵����
#define CAN2_M2006_ID2 0x206	// ���̵����
#define CAN2_M2006_ID3 0x207	// ���ֿ��ϵ��

#define FWAngle2Angle(x) ((x)/8191.0*360)
#define Angle2FWAngle(x) ((x)/360.0*8191)

/* ����3508״̬�ṹ�壬�洢��ǰ3508�ķ���ֵ*/
typedef struct {
    int16_t  	set_current;	// д��ĵ���
    uint16_t 	angle;			// �Ƕ�
		int16_t  	last_angle;
		float 		total_angle;
    int16_t 	speed;			// �ٶ�
    int16_t		current;		// ����
    uint8_t 	temperature;	// �¶�
		int16_t   cur_angle;
		int16_t  speed1;
		int16_t  speed2;
} CAN_M3508_TypeDef;


/* ����6020״̬�ṹ�壬�洢��ǰ6020�ķ���ֵ*/
typedef struct
{
    int16_t  set_voltage;	// д��ĵ�ѹ
    int16_t angle;			// �Ƕ�
    int16_t  speed;         // �ٶ�
    int16_t  current;       // ����
    int8_t  temperature;   // �¶�
	  int16_t cur_angle;			//�ǶȲ�
		int16_t last_angle;			//�ϴνǶ�
		int16_t angle_sum;			//�ǶȺ�
		float total_angle;
} CAN_GM6020_TypeDef;

/* ����2006״̬�ṹ�壬�洢��ǰ6020�ķ���ֵ*/
typedef struct
{
    int16_t  set_current;	// д��ĵ���
    uint16_t angle;			// �Ƕ�
		int16_t  	last_angle;
		float 		total_angle;
    int16_t  speed;         // �ٶ�
    int16_t  current;       // ����
    uint8_t  temperature;   // �¶�
} CAN_M2006_TypeDef;

extern CAN_GM6020_TypeDef 	CAN_GM6020[2];	// ��̨���
extern CAN_M2006_TypeDef 	CAN_M2006[3]	;	// ���̵��
extern CAN_M3508_TypeDef 	CAN_M3508[8];	// ǰ�ĸ��ǵ��̵�������ĸ���Ħ���ֵ��

void CAN_FilterInit(CAN_HandleTypeDef* hcan);
uint8_t CAN_SendMsg(CAN_HandleTypeDef* hcan,uint8_t ide,uint32_t id,uint8_t len,uint8_t *data);

extern float powerData[4];
extern int32_t percent;

void CAN_M3508_SetCurrent(int16_t i1,int16_t i2,int16_t i3,int16_t i4);
void get_total_angle_2006(CAN_M2006_TypeDef *p);
void get_total_angle_6020(CAN_GM6020_TypeDef *p);

void CAN_Chassis_SendCurrent(void);
void CAN_Shoot_SendCurrent(void);
void CAN_Superpower(uint16_t temPower);

#endif


