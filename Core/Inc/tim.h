/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim14;

/* USER CODE BEGIN Private defines */
typedef struct _rate_t
{
    uint16_t CAN1_Tx_Rate;
    uint8_t CAN1_0x201_Rate;
    uint8_t CAN1_0x202_Rate;
    uint8_t CAN1_0x204_Rate;
    uint8_t CAN1_0x205_Rate;
    uint8_t CAN1_0x209_Rate;
    uint8_t CAN1_0x20A_Rate;
    
    uint16_t CAN2_Tx_Rate;
    uint8_t CAN2_0x201_Rate;
    uint8_t CAN2_0x202_Rate;
    uint8_t CAN2_0x203_Rate;
    uint8_t CAN2_0x204_Rate;
    uint8_t CAN2_0x211_Rate;
    
    uint8_t  DR16_Rate;
    int16_t  MPU6500_Value;
    float    HI216_Value;
    uint16_t CAMERA_Rate;
}rate_t;

typedef struct _obser_t
{
    rate_t Tx;
    rate_t Rx;
}obser_t;

/* USER CODE END Private defines */

void MX_TIM1_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_TIM7_Init(void);
void MX_TIM9_Init(void);
void MX_TIM14_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */
extern obser_t Observer;

void TIM1_PWMSTART(void);
void TIM3_PWMSTART(void);
void TIM9_PWMSTART(void);
void TIM1_SetPWMPluse(uint16_t pluse);
void TIM3_SetPWMPluse(uint16_t pluse);
void TIM9_SetPWMPluse(uint16_t pluse);
void TIM14_START(void);
void TIM4_PWMSTART(void);
void TIM7_Delayus(uint32_t us);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
