#ifndef __RC_CONTROL_H
#define __RC_CONTROL_H
#include "main.h"
#define BLOCK_TIME 75
#define BLOCK_TRIGGER_SPEED 300.0f
#define REVERSE_TIME 100

typedef struct
{
		double speed;
		double speed_set;
		double trigger_speed_set;
		uint16_t block_time;
		uint16_t reverse_time;
}shoot_control_t;


extern uint8_t rc_flag;
extern uint8_t FollowSwitchFlag,IsFollowFlag,LastIsFollowFlag;
void RC_PC(float classis_speed,float spin_speed,float dial_speed,float shoot_speed);

void RC_Chassis(void);
void RC_Shoot(float fri_speed,float dial_speed);
void RC_Vision_aiming(void);
void RC_Singleshot(float fri_speed,uint8_t pattern);

void RC_Spin(void);
void angle_init(void);
void angle_sum(void);
static void trigger_motor_turn_back(void);
void M3508_Speed_Angle(void);
void M2006_Total_Angle(void);
void Level_Up_System(void);
#endif

