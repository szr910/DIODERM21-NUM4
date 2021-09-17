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
#include "bsp_tim.h"

uint16_t tim11_cnt=0;
float lastangle;
float thisangle;
float GM6020speed;

float lastangle_pit;
float thisangle_pit;
float GM6020speed_pit;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// 1msµÄÖÐ¶Ï
	if (htim->Instance == htim11.Instance)
    {
		tim11_cnt++;
		if(tim11_cnt>=tim11_cnt_max)
			tim11_cnt = tim11_cnt_max;
		
		
    }
	if(htim==(&htim2))
	{
		thisangle = imu.angle_sum;
		GM6020speed = (thisangle-lastangle)*100;
		lastangle = thisangle;
		thisangle_pit = CAN_GM6020[1].angle;
		GM6020speed_pit = (thisangle_pit-lastangle_pit)*100;
		lastangle_pit = thisangle_pit;
	}
}

