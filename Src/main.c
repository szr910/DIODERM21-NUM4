/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void BSP_All_Init(void);		// 所有设备初始化
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_SPI5_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM12_Init();
  MX_TIM4_Init();
  MX_USART6_UART_Init();
  MX_UART7_Init();
  MX_TIM11_Init();
  MX_TIM3_Init();
  MX_UART8_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  BSP_All_Init();
	angle_init();
	//Beep(100);
	HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void BSP_All_Init(void)	// 初始化所有硬件层
{
    uint8_t i=0;	// 计数
	
		mpu_device_init();		// 初始化IMU
    init_quaternion();
		HAL_TIM_Base_Start_IT(&htim11);	// 校准陀螺仪
	
    OLED_Init();			// 初始化OLED
    OLED_Clear(Pen_Write);

    PWR24V_On();			// 开启板子上的24V电压输出
    CAN_FilterInit(&hcan1);	// 初始化CAN1的筛选器
    CAN_FilterInit(&hcan2);	// 初始化CAN2的筛选器

    PID_Init(&PID_GM6020[0],POSITION_PID,4000,2000,5.5,0.01,0.03);	// 1000,0,1000
	  PID_Init(&PID_GM6020_speed[0],POSITION_PID,30000,10000,70,0,0);//0.0001
	
	  PID_Init(&PID_GM6020[1],POSITION_PID,2000,15000,3,0,0);		// 云台侧面电机1600
    PID_Init(&PID_GM6020_speed[1],POSITION_PID,30000,3000,20,0,0);//0.0001
	  
    PID_Init(&PID_M2006[0],DELTA_PID,16384,16384,5,0.1,2);		// 发射左拨盘电机 速度闭环
    PID_Init(&PID_M2006_ANGLE[0],POSITION_PID,1500,16384,20.0,0,0);	// 发射左拨盘电机 角度闭环
		PID_Init(&PID_M2006[1],DELTA_PID,16384,16384,3.0,0.1,0);		// 发射右拨盘电机 速度闭环
    PID_Init(&PID_M2006_ANGLE[1],POSITION_PID,1500,16384,20.0,0,0);	// 发射右拨盘电机 角度闭环
	
		PID_Init(&PID_M2006[2],DELTA_PID,16384,16384,4.0,0.5,1);		// 弹舱盖电机 速度闭环
    PID_Init(&PID_M2006_ANGLE[2],POSITION_PID,1500,16384,20.0,0,1);	// 弹舱盖电机 角度闭环
		
    for(i=0; i<4; i++)	// 四个底盘电机
    {
        PID_Init(&PID_M3508[i],DELTA_PID,20000,2000,2,0,0);//5.5 0.05 1.0
    }

    for(i=4; i<8; i++)	// 四个摩擦轮电机
    {
				PID_Init(&PID_M3508[i],POSITION_PID,16000,3000,9,0.05,5);
    }
		
    PID_Init(&PID_M3508_Follow,POSITION_PID,2500,10000,2.5,0,0.5);//6000 4 0 0.5 较为理想   10 0 0.1
		
		PID_Init(&PID_HEAT_PWM,POSITION_PID,1000,1000,50,0.005,20);
		PID_Init(&PID_Superpower,POSITION_PID,1000,1000,0.050,0.005,0.1);


    HAL_UART_Receive_IT(&huart2, (uint8_t *)aRxBuffer, 1);
		HAL_UART_Receive_IT(&huart2, (uint8_t *)aRxBuffer, 1);
		extern uint8_t tmp_vision;
		HAL_UART_Receive_DMA(&huart6, &tmp_vision, 1);
    InitJudgeUart();
    PWM_init();				// 初始化PWM输出
    dbus_uart_init();		// 初始化遥控器

//    usmart_dev.init(84);
		LASER_On();		// 开启激光

//	while(tim11_cnt!=tim11_cnt_max)
//	{
//		 IMU_Get();
//		 __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,PID_Calculate(&PID_HEAT_PWM,50,imu.temp)*100);
//	}
		HAL_TIM_Base_Stop_IT(&htim11);
		Beep(200);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
