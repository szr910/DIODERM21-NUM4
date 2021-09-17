/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* 系统头文件 */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

/* User头文件 */
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Bsp头文件 */
#include "bsp_oled.h"
#include "bsp_adc.h"
#include "bsp_pwm.h"
#include "bsp_imu.h"
#include "bsp_pwr.h"
#include "bsp_can.h"
#include "bsp_rc.h"
#include "bsp_beep.h"
#include "bsp_led.h"
#include "bsp_tim.h"
#include "bsp_usart.h"

/* Control 头文件*/
#include "pid.h"
#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
#include "rc_control.h"
#include "SuperCapacity.h"
/* Debug 头文件*/
#include "bluetooth.h"
#include "debug_oled.h"
#include "usmart.h"
/* Judge 头文件*/
#include "JudgeTask.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWR5VGND_Pin GPIO_PIN_13
#define PWR5VGND_GPIO_Port GPIOG
#define OLED_MISO_Pin GPIO_PIN_4
#define OLED_MISO_GPIO_Port GPIOB
#define OLED_SCK_Pin GPIO_PIN_3
#define OLED_SCK_GPIO_Port GPIOB
#define OLED_DC_Pin GPIO_PIN_9
#define OLED_DC_GPIO_Port GPIOB
#define BT_Rx_Pin GPIO_PIN_6
#define BT_Rx_GPIO_Port GPIOD
#define BT_Tx_Pin GPIO_PIN_5
#define BT_Tx_GPIO_Port GPIOD
#define PWR24V1_Pin GPIO_PIN_2
#define PWR24V1_GPIO_Port GPIOH
#define PWR24V2_Pin GPIO_PIN_3
#define PWR24V2_GPIO_Port GPIOH
#define PWR24V3_Pin GPIO_PIN_4
#define PWR24V3_GPIO_Port GPIOH
#define PWR24V4_Pin GPIO_PIN_5
#define PWR24V4_GPIO_Port GPIOH
#define SPI5_NSS_Pin GPIO_PIN_6
#define SPI5_NSS_GPIO_Port GPIOF
#define fricftion2_Pin GPIO_PIN_11
#define fricftion2_GPIO_Port GPIOH
#define fricftion1_Pin GPIO_PIN_10
#define fricftion1_GPIO_Port GPIOH
#define beep_Pin GPIO_PIN_6
#define beep_GPIO_Port GPIOH
#define LED_R_Pin GPIO_PIN_11
#define LED_R_GPIO_Port GPIOE
#define OLED_MOSI_Pin GPIO_PIN_7
#define OLED_MOSI_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_14
#define LED_G_GPIO_Port GPIOF
#define OLED_RST_Pin GPIO_PIN_10
#define OLED_RST_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
