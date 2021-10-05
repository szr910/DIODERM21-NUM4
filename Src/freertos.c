/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "pid.h"
#include "bsp_pwm.h"
#include "bsp_imu.h"
#include "gimbal.h"
#include "bsp_can.h"
#include "JudgeTask.h"
#include "kalman.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
char persent_x[5]={0};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

extern int 	setSpeed[4];
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern uint8_t RxCounter1,RxBuffer1[500],RxTemp1,F_Usart1;
extern float powerData[];
float chassis_speed=4000, chassis_speed_limit=1600;
float i=0;
double last_imu_yaw;
extern double imu_last_anglesum ;
uint16_t dialspd=2000,Startflag=0;
uint16_t Temp_dialspd,UIflag=0;
extern uint16_t remainHeat0;
uint16_t UsartCounter;
float DeltaChasisSpd;
uint16_t RC_FLAG=0;
uint16_t RcCounter=0;
uint16_t RcCounterTrue=0;
uint16_t RcCounterFalse=0;

extern VisionData datadata;
extern extKalman_t vis_data_pit;
extern extKalman_t vis_data_yaw;
extKalman_t imu_speed;
extKalman_t pit_data_speed;
float imukal_speed;
float pit_speed;
extern float vis_pitch,vis_yaw;
extern uint8_t IsSpeedUpFlag;
extern float InputPower;
extern float GM6020speed;
extern float GM6020speed_pit;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId ControlHandle;
osThreadId DebugHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartControl(void const * argument);
void StartDebug(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
    /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Control */
  osThreadDef(Control, StartControl, osPriorityNormal, 0, 128);
  ControlHandle = osThreadCreate(osThread(Control), NULL);

  /* definition and creation of Debug */
  osThreadDef(Debug, StartDebug, osPriorityNormal, 0, 128);
  DebugHandle = osThreadCreate(osThread(Debug), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
		tim11_cnt=0;
		KalmanCreate(&vis_data_pit,20,200);
		KalmanCreate(&vis_data_yaw,20,200);
		KalmanCreate(&imu_speed,200,200);
		KalmanCreate(&pit_data_speed,200,200);
		/* Infinite loop */
		for(;;)
		{
			
			vis_pitch = datadata.pitch_angle;
			vis_yaw = datadata.yaw_angle;
			imukal_speed = GM6020speed;
			pit_speed = GM6020speed_pit;
			pit_speed = KalmanFilter(&pit_data_speed,pit_speed);
				RcCounter++;
				if(rc_flag)
						RcCounterTrue++;
				else 
						RcCounterFalse++;
				if(RcCounter>=20)
				{
						if(RcCounterTrue > RcCounterFalse)
								RC_FLAG=1;
						else 
								RC_FLAG=0;
						RcCounter=0;
						RcCounterTrue=0;
						RcCounterFalse=0;
				}
				imu_last_anglesum = imu.angle_sum;
				angle_sum();
				IMU_Get();
				M2006_Total_Angle();
				if(RC_FLAG)
				{
					  if(!Startflag)
							CAN_GM6020[0].total_angle=CAN_GM6020[0].angle,Startflag=1;
            dialspd=2000;
//						if(GameRobotState.shooter_id1_17mm_cooling_limit - PowerHeat.shooter_id1_17mm_cooling_heat < 10 ||GameRobotState.shooter_id2_17mm_cooling_limit - PowerHeat.shooter_id2_17mm_cooling_heat < 10 )
//							dialspd = 0;

						Level_Up_System();
				}
				else
					Startflag=0;
		}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartControl */
/**
* @brief Function implementing the Control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControl */
void StartControl(void const * argument)
{
  /* USER CODE BEGIN StartControl */
    /* Infinite loop */
    for(;;)
    {
			BT_SendDialWave();
        if(RC_FLAG==1)
            Beep_Off();
        else {
            CAN_M3508[0].set_current=0;
            CAN_M3508[1].set_current=0;
            CAN_M3508[2].set_current=0;
            CAN_M3508[3].set_current=0;
            CAN_M3508[4].set_current=0;
            CAN_M3508[5].set_current=0;
						CAN_M3508[6].set_current=0;
            CAN_M3508[7].set_current=0;
					
            CAN_GM6020[0].set_voltage=0;
            CAN_GM6020[1].set_voltage=0;
					
						CAN_M2006[0].set_current=0;
						CAN_M2006[1].set_current=0;
						CAN_M2006[2].set_current=0;
					
            CAN_Chassis_SendCurrent();
            CAN_Shoot_SendCurrent();

        }
        rc_flag=0;
				JUDEG_UI();
				osDelay(100);
    }
  /* USER CODE END StartControl */
}

/* USER CODE BEGIN Header_StartDebug */
/**
* @brief Function implementing the Debug thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDebug */
void StartDebug(void const * argument)
{
  /* USER CODE BEGIN StartDebug */

    /* Infinite loop */
    for(;;)
    {
//			HAL_UART_Transmit(&huart6, "a", 1,1000 );

			last_imu_yaw = imu.yaw;
      IMU_Get();
      __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,PID_Calculate(&PID_HEAT_PWM,55,imu.temp)*100);
	    

		}
  /* USER CODE END StartDebug */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
