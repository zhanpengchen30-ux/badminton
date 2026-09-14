/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "BMI088driver.h"
#include "bsp_delay.h"
#include "math.h"
#include "bsp_delay.h"
#include "INS_task.h"
#include "DJI_IMU.h"
#include "user_can.h"
#include "bsp_buzzer.h"
#include "calibrate_task.h"
#include "main.h"
#include "Gimbal_task.h"
#include "pid.h"
#include "CAN_receive.h"
#include "PID.h"
#include "can.h"
#include "bsp_can_DM.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern uint8_t Data[];
extern uint8_t Data2[];
extern void Control_Task(void const * argument);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId GimbalHandle;
osThreadId IMU_Send_TaskHandle;
osThreadId imuTaskHandle;
osThreadId caliHandle;
osThreadId Gimbal1Handle;
osThreadId ControlTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Gimbal_task(void const * argument);
void IMU_Send(void const * argument);
extern void INS_task(void const * argument);
extern void calibrate_task(void const * argument);
void Gimbal1_Task(void const * argument);
void Control_Task(void const * argument);

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
  /* definition and creation of Gimbal */
  osThreadDef(Gimbal, Gimbal_task, osPriorityNormal, 0, 128);
  GimbalHandle = osThreadCreate(osThread(Gimbal), NULL);

  /* definition and creation of IMU_Send_Task */
  osThreadDef(IMU_Send_Task, IMU_Send, osPriorityHigh, 0, 128);
  IMU_Send_TaskHandle = osThreadCreate(osThread(IMU_Send_Task), NULL);

  /* definition and creation of imuTask */
  osThreadDef(imuTask, INS_task, osPriorityAboveNormal, 0, 512);
  imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

  /* definition and creation of cali */
  osThreadDef(cali, calibrate_task, osPriorityLow, 0, 512);
  caliHandle = osThreadCreate(osThread(cali), NULL);

  /* definition and creation of Gimbal1 */
  osThreadDef(Gimbal1, Gimbal1_Task, osPriorityLow, 0, 128);
  Gimbal1Handle = osThreadCreate(osThread(Gimbal1), NULL);

  /* definition and creation of ControlTask */
  osThreadDef(ControlTask, Control_Task, osPriorityNormal, 0, 512);
  ControlTaskHandle = osThreadCreate(osThread(ControlTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Gimbal_task */
/**
  * @brief  Function implementing the Gimbal thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Gimbal_task */
__weak void Gimbal_task(void const * argument)
{
  /* USER CODE BEGIN Gimbal_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Gimbal_task */
}

/* USER CODE BEGIN Header_IMU_Send */
/**
* @brief Function implementing the IMU_Send_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_Send */
void IMU_Send(void const * argument)
{
  /* USER CODE BEGIN IMU_Send */
	
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(1); 
  /* Infinite loop */
  for(;;)
  {
		if(cali_sensor[0].cali_done == CALIED_FLAG && cali_sensor[0].cali_cmd == 0)
		{
			//���������ǽǶ�
//			Euler_Fun();
			Euler_Send.yaw = INS_angle[0];
			Euler_Send.pitch = INS_angle[1];
			Euler_Send_Fun(Euler_Send);
			//���ٶ�
			Gyro_Send.Gyro_z = INS_gyro[2];
			Gyro_Send.Gyro_y = INS_gyro[1];
			Gyro_Send_Fun(Gyro_Send);
//			YawAG_Send.yaw    = INS_angle[0];
//			YawAG_Send.Gyro_z = INS_gyro[2];
//			YawAG_Send_Fun(YawAG_Send);

		}
    vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
  }
		
		

//    osDelay(1);
  
  /* USER CODE END IMU_Send */
}

/* USER CODE BEGIN Header_Gimbal1_Task */
/**
* @brief Function implementing the Gimbal1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gimbal1_Task */
void Gimbal1_Task(void const * argument)
{
  /* USER CODE BEGIN Gimbal1_Task */
  /* Infinite loop */

  for(;;)
  {		
//		Motor_control();
    osDelay(1);
	}
  /* USER CODE END Gimbal1_Task */
}

/* USER CODE BEGIN Header_Control_Task */
/**
* @brief Function implementing the ControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Control_Task */
//void Control_Task(void const * argument)
//{
//  /* USER CODE BEGIN Control_Task */
//  /* Infinite loop */
//  for(;;)
//  {
//    osDelay(1);
//  }
//  /* USER CODE END Control_Task */
//}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
