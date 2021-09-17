/**
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 * @file       power.c
 * @brief      
 * @note       PH2 PH3 PH4 PH5 
 * @Version    V1.0.0
 * @Date       2021.5      
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 */
#include "bsp_pwr.h"

// 24V可控电源不输出
void PWR24V_Off(void)
{
	HAL_GPIO_WritePin(PWR24V1_GPIO_Port,PWR24V1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PWR24V2_GPIO_Port,PWR24V2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PWR24V3_GPIO_Port,PWR24V3_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PWR24V4_GPIO_Port,PWR24V4_Pin,GPIO_PIN_RESET);
}
// 24V可控电源输出
void PWR24V_On(void)
{
	HAL_GPIO_WritePin(PWR24V1_GPIO_Port,PWR24V1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(PWR24V2_GPIO_Port,PWR24V2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PWR24V3_GPIO_Port,PWR24V3_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PWR24V4_GPIO_Port,PWR24V4_Pin,GPIO_PIN_SET);
}

// 打开激光
void LASER_On(void)
{
	HAL_GPIO_WritePin(PWR5VGND_GPIO_Port, PWR5VGND_Pin, GPIO_PIN_SET);
}
// 关闭激光
void LASER_Off(void)
{
	HAL_GPIO_WritePin(PWR5VGND_GPIO_Port, PWR5VGND_Pin, GPIO_PIN_RESET);
}
