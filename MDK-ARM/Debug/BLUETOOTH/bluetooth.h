#ifndef __BT_H
#define __BT_H
#include "main.h"

void BT_SendChasisWave(void);
void BT_SendgimbalWave(void);
void BT_Check(void);

void BT_ResetPID1(float p,float i,float d);
void BT_SendDialWave(void);

void BT_SendWave(void);
void BT_SendWave2(void);
void BT_SendWave3(void);
void BT_SendPowerHeatWave(void);
void BT_Sendgimbal_3508Wave(void);
void BT_Send6020Wave(void);
#endif


