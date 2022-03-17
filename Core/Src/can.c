/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "tim.h"
#include "usart.h"
#include "chassis.h"
#include "holder.h"
#include "shoot.h"
#include "config.h"

#define F407_CAN_ID 0x200
#define CAN1_FIFO CAN_RX_FIFO0
#define CAN2_FIFO CAN_RX_FIFO0


float DirAngle;
/* 注意：CAN的配置与cubemx不一样 1 10 3*/

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = CAN1_RX_Pin|CAN1_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = CAN2_RX_Pin|CAN2_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_TX_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, CAN1_RX_Pin|CAN1_TX_Pin);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, CAN2_RX_Pin|CAN2_TX_Pin);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void CAN1_FilterInit(void)
{
    CAN_FilterTypeDef fcan;
    fcan.FilterBank = 0;
    fcan.FilterMode = CAN_FILTERMODE_IDMASK;
    fcan.FilterScale = CAN_FILTERSCALE_32BIT;
    
    fcan.FilterIdHigh = 0;
    fcan.FilterIdLow = 0;
    fcan.FilterMaskIdHigh = 0;
    fcan.FilterMaskIdLow = 0;
    fcan.FilterFIFOAssignment = CAN1_FIFO;
    fcan.FilterActivation = ENABLE;
    fcan.SlaveStartFilterBank = 0;
    
    HAL_CAN_ConfigFilter(&hcan1,&fcan);
    HAL_CAN_Start(&hcan1);
    
    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY);
}

void CAN2_FilterInit(void)
{
    CAN_FilterTypeDef fcan;
    fcan.FilterBank = 0;
    fcan.FilterMode = CAN_FILTERMODE_IDMASK;
    fcan.FilterScale = CAN_FILTERSCALE_32BIT;
    
    fcan.FilterIdHigh = 0;
    fcan.FilterIdLow = 0;
    fcan.FilterMaskIdHigh = 0;
    fcan.FilterMaskIdLow = 0;
    fcan.FilterFIFOAssignment = CAN2_FIFO;
    fcan.FilterActivation = ENABLE;
    fcan.SlaveStartFilterBank = 0;
    
    HAL_CAN_ConfigFilter(&hcan2,&fcan);
    HAL_CAN_Start(&hcan2);
    
    HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY );
}

CAN_TxHeaderTypeDef CAN1TxMsg;
void CAN1_Transmit(uint16_t ID,uint8_t *pData)
{
	uint32_t MailBox;
    CAN1TxMsg.StdId = ID;
    CAN1TxMsg.ExtId = 0;
    CAN1TxMsg.IDE = CAN_ID_STD;
    CAN1TxMsg.RTR = CAN_RTR_DATA;
    CAN1TxMsg.DLC = 8;
	HAL_CAN_AddTxMessage(&hcan1,&CAN1TxMsg,pData,&MailBox); 
}

CAN_TxHeaderTypeDef CAN2TxMsg;
void CAN2_Transmit(uint16_t ID,uint8_t *pData)
{
	uint32_t MailBox;
    CAN2TxMsg.StdId = ID;
    CAN2TxMsg.ExtId = 0;
    CAN2TxMsg.IDE = CAN_ID_STD;
    CAN2TxMsg.RTR = CAN_RTR_DATA;
    CAN2TxMsg.DLC = 8;
	HAL_CAN_AddTxMessage(&hcan2,&CAN2TxMsg,pData,&MailBox); 
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) 
{
    UNUSED(hcan);
    uint8_t Data[8];
    CAN_RxHeaderTypeDef RxMsg;
    if(hcan->Instance == CAN1) /* CAN1 Receive DATA shootID:0x204 pitchID:0x209 yawID: 0x20A */
    {	
        HAL_CAN_GetRxMessage(hcan, CAN1_FIFO, &RxMsg, Data);
        switch(RxMsg.StdId)
        {
            
            case 0x201:/* 上交摩擦轮 */
                Observer.Rx.CAN1_0x201_Rate++;
                M3508[0].can.Rx.Speed = Data[2] << 8 | Data[3];
                Shoot_Speed_CanTransmit_M3508();
            break;
            
            case 0x202:/* 上交摩擦轮 */
                Observer.Rx.CAN1_0x202_Rate++;
                M3508[1].can.Rx.Speed = Data[2] << 8 | Data[3];  
            break;
            
            case 0x204:
                Observer.Rx.CAN1_0x204_Rate++;
                Circle_Continue(&M2006_ID4.Mcircle_Position, Data[0] << 8 | Data[1]);
                M2006_ID4.Informarion_Tran.Rx.Speed = Data[2] << 8 | Data[3];
                Shoot_CanTransmit();
            break;  
            case 0x205:
				Observer.Rx.CAN1_0x205_Rate++;
                Circle_Continue(&M2006_ID5.Mcircle_Position, Data[0] << 8 | Data[1]);
                M2006_ID5.Informarion_Tran.Rx.Speed = Data[2] << 8 | Data[3];
                Shoot_CanTransmit();
            break;
            case 0x209:/* 云台俯仰 */
                Observer.Rx.CAN1_0x209_Rate++;
                Holder.Pitch._0x209.Rx.Angle = Data[0] << 8 | Data[1];
                Holder.Pitch._0x209.Rx.Speed = Data[2] << 8 | Data[3];
                Holder_CanTransmit(); /*< 云台控制 */
            break;
            case 0x20A:/* 云台偏航 */
                Observer.Rx.CAN1_0x20A_Rate++;
                Circle_Continue(&Holder.Yaw._0x20A.Mc, Data[0] << 8 | Data[1]);
                Holder.Yaw._0x20A.Rx.Angle = 8192*Holder.Yaw._0x20A.Mc.Circle + Holder.Yaw._0x20A.Mc.Angle;

                Holder.Yaw._0x20A.Rx.Speed = Data[2] << 8 | Data[3];

				DirAngle = (float)(Holder.Yaw._0x20A.Mc.Angle * 360/8192) + ANGLE_BIAS;
                Holder_CanTransmit(); /*< 云台控制 */
            break;
        }
    }
    
    if(hcan->Instance == CAN2)/* CAN2 Receive DATA ID:0x201 0x202 0x203 0x204 */
    {	
        HAL_CAN_GetRxMessage(hcan, CAN2_FIFO, &RxMsg, Data);
        switch(RxMsg.StdId)
        {
            case 0x201:/* 底盘前左 */
                Observer.Rx.CAN2_0x201_Rate++;
                Chassis.M3508[0].Rx.Speed   = Data[2] << 8 | Data[3];
                Chassis.M3508[0].Rx.Current = Data[4] << 8 | Data[5];
                Chassis_CanTransmit();/*< 底盘控制 */
            break;
            case 0x202:/* 底盘前右 */
                Observer.Rx.CAN2_0x202_Rate++;
                Chassis.M3508[1].Rx.Speed   = Data[2] << 8 | Data[3];
                Chassis.M3508[1].Rx.Current = Data[4] << 8 | Data[5];
            break;
            case 0x203:/* 底盘后左 */
                Observer.Rx.CAN2_0x203_Rate++;
                Chassis.M3508[2].Rx.Speed   = Data[2] << 8 | Data[3];
                Chassis.M3508[2].Rx.Current = Data[4] << 8 | Data[5];            
            break;
            case 0x204:/* 底盘后右 */
                Observer.Rx.CAN2_0x204_Rate++;
                Chassis.M3508[3].Rx.Speed   = Data[2] << 8 | Data[3];
                Chassis.M3508[3].Rx.Current = Data[4] << 8 | Data[5];
            break;
            case 0x211:/* 超级电容 */
                Observer.Rx.CAN2_0x211_Rate++;
                Chassis.SuperCap.InVol = Data[1] << 8 | Data[0];
                Chassis.SuperCap.CapVol = Data[3] << 8 | Data[2];
                Chassis.SuperCap.InCur = Data[5] << 8 | Data[4];
                Chassis.SuperCap.Power = Data[7] << 8 | Data[6];
            break;
        }
    }
}

float Get_fDirAngle(void)
{
    DirAngle -= DirAngle>180? 360:0;
    DirAngle += DirAngle<-180? 360:0;
	return DirAngle;
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
