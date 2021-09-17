#ifndef __BSP_CAN
#define __BSP_CAN
#include "main.h"
#include "can.h"

/* 电机反馈ID 定义*/
#define	CAN1_M3508_ID1 0x201	// 底盘电机1左前
#define	CAN1_M3508_ID2 0x202	// 底盘电机2右前
#define	CAN1_M3508_ID3 0x203	// 底盘电机3左后
#define	CAN1_M3508_ID4 0x204	// 底盘电机4右后
#define CAN1_GM6020_ID1 0x209	// 云台底部电机
#define CAN1_GM6020_ID2 0x20A	// 云台侧面电机

#define	CAN2_M3508_ID1 0x201	// 摩擦轮电机 左上
#define	CAN2_M3508_ID2 0x202	// 摩擦轮电机 左下
#define	CAN2_M3508_ID3 0x203	// 摩擦轮电机 右上  
#define	CAN2_M3508_ID4 0x204	// 摩擦轮电机 左下
#define CAN2_M2006_ID1 0x205	// 拨盘电机左
#define CAN2_M2006_ID2 0x206	// 拨盘电机右
#define CAN2_M2006_ID3 0x207	// 弹仓开合电机

#define FWAngle2Angle(x) ((x)/8191.0*360)
#define Angle2FWAngle(x) ((x)/360.0*8191)

/* 定义3508状态结构体，存储当前3508的反馈值*/
typedef struct {
    int16_t  	set_current;	// 写入的电流
    uint16_t 	angle;			// 角度
		int16_t  	last_angle;
		float 		total_angle;
    int16_t 	speed;			// 速度
    int16_t		current;		// 电流
    uint8_t 	temperature;	// 温度
		int16_t   cur_angle;
		int16_t  speed1;
		int16_t  speed2;
} CAN_M3508_TypeDef;


/* 定义6020状态结构体，存储当前6020的反馈值*/
typedef struct
{
    int16_t  set_voltage;	// 写入的电压
    int16_t angle;			// 角度
    int16_t  speed;         // 速度
    int16_t  current;       // 电流
    int8_t  temperature;   // 温度
	  int16_t cur_angle;			//角度差
		int16_t last_angle;			//上次角度
		int16_t angle_sum;			//角度和
		float total_angle;
} CAN_GM6020_TypeDef;

/* 定义2006状态结构体，存储当前6020的反馈值*/
typedef struct
{
    int16_t  set_current;	// 写入的电流
    uint16_t angle;			// 角度
		int16_t  	last_angle;
		float 		total_angle;
    int16_t  speed;         // 速度
    int16_t  current;       // 电流
    uint8_t  temperature;   // 温度
} CAN_M2006_TypeDef;

extern CAN_GM6020_TypeDef 	CAN_GM6020[2];	// 云台电机
extern CAN_M2006_TypeDef 	CAN_M2006[3]	;	// 拨盘电机
extern CAN_M3508_TypeDef 	CAN_M3508[8];	// 前四个是底盘电机，后四个是摩擦轮电机

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


