/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "usart.h"
#include "tim.h"
#include "ui.h"
#include "message.h"
#include "judge.h"
#include "vision.h"
#include "holder.h"
#include "chassis.h"
#include "shoot.h"
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
/* USER CODE BEGIN Variables */
osEvent evt;
/* USER CODE END Variables */
osThreadId LEDHandle;
osThreadId MSGHandle;
osThreadId UIHandle;
osThreadId KEYHandle;
osThreadId ChassisHandle;
osThreadId HolderHandle;
osThreadId ShootHandle;
osThreadId JudgeHandle;
osMessageQId RDtHMsgHandle;
osMessageQId RDtCMsgHandle;
osMessageQId RDtSMsgHandle;
osMessageQId HDtHMsgHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void taskLED(void const * argument);
void taskMsg(void const * argument);
void taskUI(void const * argument);
void taskKey(void const * argument);
void taskChassis(void const * argument);
void taskHolder(void const * argument);
void taskShoot(void const * argument);
void taskJudge(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
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

  /* Create the queue(s) */
  /* definition and creation of RDtHMsg */
  osMessageQDef(RDtHMsg, 16, uint32_t);
  RDtHMsgHandle = osMessageCreate(osMessageQ(RDtHMsg), NULL);

  /* definition and creation of RDtCMsg */
  osMessageQDef(RDtCMsg, 16, uint32_t);
  RDtCMsgHandle = osMessageCreate(osMessageQ(RDtCMsg), NULL);

  /* definition and creation of RDtSMsg */
  osMessageQDef(RDtSMsg, 16, uint32_t);
  RDtSMsgHandle = osMessageCreate(osMessageQ(RDtSMsg), NULL);

  /* definition and creation of HDtHMsg */
  osMessageQDef(HDtHMsg, 16, uint32_t);
  HDtHMsgHandle = osMessageCreate(osMessageQ(HDtHMsg), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of LED */
  osThreadDef(LED, taskLED, osPriorityIdle, 0, 64);
  LEDHandle = osThreadCreate(osThread(LED), NULL);

  /* definition and creation of MSG */
  osThreadDef(MSG, taskMsg, osPriorityHigh, 0, 128);
  MSGHandle = osThreadCreate(osThread(MSG), NULL);

  /* definition and creation of UI */
  osThreadDef(UI, taskUI, osPriorityBelowNormal, 0, 128);
  UIHandle = osThreadCreate(osThread(UI), NULL);

  /* definition and creation of KEY */
  osThreadDef(KEY, taskKey, osPriorityBelowNormal, 0, 128);
  KEYHandle = osThreadCreate(osThread(KEY), NULL);

  /* definition and creation of Chassis */
  osThreadDef(Chassis, taskChassis, osPriorityRealtime, 0, 512);
  ChassisHandle = osThreadCreate(osThread(Chassis), NULL);

  /* definition and creation of Holder */
  osThreadDef(Holder, taskHolder, osPriorityRealtime, 0, 512);
  HolderHandle = osThreadCreate(osThread(Holder), NULL);

  /* definition and creation of Shoot */
  osThreadDef(Shoot, taskShoot, osPriorityRealtime, 0, 512);
  ShootHandle = osThreadCreate(osThread(Shoot), NULL);

  /* definition and creation of Judge */
  osThreadDef(Judge, taskJudge, osPriorityHigh, 0, 1024);
  JudgeHandle = osThreadCreate(osThread(Judge), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_taskLED */
/**
  * @brief  Function implementing the LED thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_taskLED */
void taskLED(void const * argument)
{
  /* USER CODE BEGIN taskLED */
    HAL_GPIO_WritePin(GPIOD, LED1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, LED2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, LED3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, LED4_Pin, GPIO_PIN_SET);
    RemoteData_t *RDMsg;
    
    /* Infinite loop */
    for (;;)
    {
        HAL_GPIO_TogglePin(GPIOD, LED1_Pin);
        HAL_GPIO_TogglePin(GPIOD, LED2_Pin);
        HAL_GPIO_TogglePin(GPIOC, LED3_Pin);
        HAL_GPIO_TogglePin(GPIOC, LED4_Pin);
        
        uint8_t id = (uint8_t)JUDGE_u8GetRobotId();
        
        evt = osMessageGet(RDtCMsgHandle,0);
        if(evt.status == osEventMessage)
        {
            RDMsg = (RemoteData_t*)evt.value.v;
        }
        
        Vision_SendData(*RDMsg);
        
        osDelay(5);
    }
  /* USER CODE END taskLED */
}

/* USER CODE BEGIN Header_taskMsg */
/**
* @brief Function implementing the MSG thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_taskMsg */
void taskMsg(void const * argument)
{
  /* USER CODE BEGIN taskMsg */
    /* Infinite loop */
    for (;;)
    {
        RemoteData_t RDMsg;
        HolderData_t HDMsg;
        RemoteDataMsg_Process(&RDMsg);
        HolderDataMsg_Process(&HDMsg);
        osMessagePut(RDtHMsgHandle, (uint32_t)&RDMsg, 0);
        osMessagePut(RDtCMsgHandle, (uint32_t)&RDMsg, 0);
        osMessagePut(HDtHMsgHandle, (uint32_t)&HDMsg, 0);
        osMessagePut(RDtSMsgHandle, (uint32_t)&RDMsg, 0);
        osDelay(10);
    }
  /* USER CODE END taskMsg */
}

/* USER CODE BEGIN Header_taskUI */
/**
* @brief Function implementing the UI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_taskUI */
void taskUI(void const * argument)
{
  /* USER CODE BEGIN taskUI */
    UI_Init();
    /* Infinite loop */
    for (;;)
    {
        UI_Process();
        osDelay(10);
    }
  /* USER CODE END taskUI */
}

/* USER CODE BEGIN Header_taskKey */
/**
* @brief Function implementing the KEY thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_taskKey */
void taskKey(void const * argument)
{
  /* USER CODE BEGIN taskKey */
    /* Infinite loop */
    for (;;)
    {
        Key_Process();
        osDelay(10);
    }
  /* USER CODE END taskKey */
}

/* USER CODE BEGIN Header_taskChassis */
/**
* @brief Function implementing the Chassis thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_taskChassis */
void taskChassis(void const * argument)
{
  /* USER CODE BEGIN taskChassis */
    RemoteData_t *RDMsg;
    Chassis_Init();
    /* Infinite loop */
    for(;;)
    {
        evt = osMessageGet(RDtCMsgHandle,0);
        if(evt.status == osEventMessage)
        {
            RDMsg = (RemoteData_t*)evt.value.v;
        } 
        Chassis_Process(*RDMsg);
        osDelay(5);
    }
  /* USER CODE END taskChassis */
}

/* USER CODE BEGIN Header_taskHolder */
/**
* @brief Function implementing the Holder thread.
* @param argument: Not used
* @retval None
*/

/* USER CODE END Header_taskHolder */
void taskHolder(void const * argument)
{
  /* USER CODE BEGIN taskHolder */
    osDelay(500);
    HolderData_t *HDMsg;
    RemoteData_t *RDMsg;
    evt = osMessageGet(HDtHMsgHandle,0);
    if(evt.status == osEventMessage)
    {
        HDMsg = (HolderData_t*)evt.value.v;
    }
    
    
    Holder_Init(*HDMsg);
    /* Infinite loop */
    for(;;)
    {
        evt = osMessageGet(HDtHMsgHandle,0);
        if(evt.status == osEventMessage)
        {
            HDMsg = (HolderData_t*)evt.value.v;
        }
        evt = osMessageGet(RDtHMsgHandle,0);
        if(evt.status == osEventMessage)
        {
            RDMsg = (RemoteData_t*)evt.value.v;
        }
        Holder_Process(*HDMsg,*RDMsg);
        osDelay(1);
    }
  /* USER CODE END taskHolder */
}

/* USER CODE BEGIN Header_taskShoot */
/**
* @brief Function implementing the Shoot thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_taskShoot */
void taskShoot(void const * argument)
{
  /* USER CODE BEGIN taskShoot */
    RemoteData_t *RDMsg;
    Shoot_PidInit();
    /* Infinite loop */
    for (;;)
    {
        evt = osMessageGet(RDtSMsgHandle, 0);
        if (evt.status == osEventMessage)
        {
            RDMsg = (RemoteData_t *)evt.value.v;
        }
        Shoot_Process(*RDMsg);
        osDelay(1);
    }
  /* USER CODE END taskShoot */
}

/* USER CODE BEGIN Header_taskJudge */
/**
* @brief Function implementing the Judge thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_taskJudge */
void taskJudge(void const * argument)
{
  /* USER CODE BEGIN taskJudge */
    Judge_InitData();
    /* Infinite loop */
    for (;;)
    {
        Judge_Proccess();
        osDelay(10);
    }
  /* USER CODE END taskJudge */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
