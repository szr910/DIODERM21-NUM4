/**
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 * @file       bsp_pwm.c
 * @brief      
 * @note       TIM1 (IN1) (IN2) (IN3) (IN4)
 * 		       TIM5 (IN1) (IN2)
 * @Version    V1.0.0
 * @Date       2021.5      
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 */
#include "bsp_pwm.h"

/**
  * @brief  ���� PWM
  * @param  None
  * @retval None
  */
void PWM_init(void)
{
//    // ��ʼ���PWM �ṩ��GM6020
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

//    // ��ʼ���PWM �ṩ��Ħ����
//    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
//    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
//    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1,2000);
//    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2,2000);
//    HAL_Delay(2000);
//    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1,1000);
//    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2,1000);
//    HAL_Delay(3000);
	// ��ʼ���PWM �ṩ��������
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,1000);
}


