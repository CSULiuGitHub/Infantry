/**
  ******************************************************************************
  * @file    holder.h
  * @author  sy xl qj
  * @brief   云台任务
  * @date      
  ******************************************************************************
  * @attention
  *
  * Copyright (c) CSU_RM_FYT
  * All rights reserved.
  *
  * This software component is licensed by SY under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  * opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _HOLDER_H
#define _HOLDER_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* includes ------------------------------------------------------------------*/
#include "motor.h"
#include "message.h"

/* typedef -------------------------------------------------------------------*/
typedef struct _Yaw_t
{
    GM6020_t _0x20A;    
    PID_AbsoluteType PidSpeed;
    PID_PreDifferentialType PidSpeed_pre;
    PID_AbsoluteType PidAngle;
	PID_AbsoluteType PidVisionAngle; 
    PID_IncrementType PidSpeed_Inc;
    PID_IncrementType PidAngle_Inc;
    int32_t TarAngle;
    int32_t TarSpeed;
    int32_t TarSpeedLpf;
    int32_t Angle;
    int32_t Speed;
    int32_t Output;
    int32_t OutputLpf;
}Yaw_t;

typedef struct _Pitch_t
{
    GM6020_t _0x209;
    PID_AbsoluteType PidSpeed;
    PID_PreDifferentialType PidSpeed_pre;
    PID_AbsoluteType PidAngle;
	PID_AbsoluteType PidVisionAngle;
    PID_IncrementType PidSpeed_Inc;
    PID_IncrementType PidAngle_Inc;
    int32_t TarAngle;
    int32_t TarSpeed;
    int32_t TarSpeedLpf;
    int32_t Angle;
    int32_t ImuAngle;
    int32_t Speed;
    int32_t Output;
    int32_t OutputLpf;
}Pitch_t;

typedef struct _Holder_t
{
    Yaw_t Yaw;
    Pitch_t Pitch;
    uint8_t CanData[8];
}Holder_t;

/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
/* function ------------------------------------------------------------------*/
void Holder_Init(HolderData_t HDMsg);
void Holder_Process(HolderData_t HDMsg,RemoteData_t RDMsg);

/*********************************云台辅助函数*********************************/
void Holder_Angle_Reset(void);
void Holder_Angle_Protect(RemoteData_t RDMsg);
void Holder_CanTransmit(void);
extern Holder_t Holder;
void Holder_Debug(void); /*< 云台调试 波形*/

void Holder_Vision_Prediction(void);
//void Send_Vision(void);
#ifdef __cplusplus
}
#endif

#endif /* */
  
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
