/**
  ******************************************************************************
  * @file    
  * @author  sy
  * @brief   
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
#ifndef _SHOOT_H
#define _SHOOT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* includes ------------------------------------------------------------------*/
#include "motor.h"
#include "message.h"
#include "pid.h"
#include "FreeRTOS.h"
#include "task.h"
/* typedef -------------------------------------------------------------------*/
typedef struct _Shoot_t
{
      Mcircle_t Mcircle_Position;
      M2006_t Informarion_Tran;
	  int32_t Speed;
      int32_t RmfSpeed; /*< 递推均值滤波 */
      int32_t Angle; 
	  int32_t TarPosition;
      int32_t Position;
	  int32_t TarSpeed;
      int32_t TarSpeedLpf;
	  int32_t Output;
      int32_t OutputLpf;
      
	  uint8_t CanData[8];
      PID_AbsoluteType PidSpeed;
	  PID_AbsoluteType PidPosition;
}Shoot_t;

typedef struct _Shoot_Speed_t
{
      M3508_t can;
	  int32_t Speed;
      int32_t RmfSpeed; /*< 加权滤波 */
	  int32_t TarSpeed;
      int32_t TarSpeedLpf;
	  int32_t Output;
      int32_t OutputLpf;
	  uint8_t CanData[8];
      PID_AbsoluteType PidSpeed;
}Shoot_Speed_t;

/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
extern Shoot_t M2006_ID4;
extern Shoot_t M2006_ID5;
extern Shoot_Speed_t M3508[2]; 
/* function ------------------------------------------------------------------*/
void Shoot_Freq_Pid_Reset(void); /*< 拨弹电机PID误差清零 */
void  Shoot_PidInit(void);
void Shoot_Process(RemoteData_t RDMsg);
portTickType REVOL_uiGetRevolTime(void);
void Bullet_Prepare(RemoteData_t RDMsg);
void Lid_Open(void);
void Lid_Close(void);
void Shoot_CanTransmit(void);
void Shoot_Freq_Control(RemoteData_t RDMsg); /*< 连发速率控制*/
void Shoot_Speed_Control_Realtime(void); /*< 实时射速控制*/
void Shoot_Speed_PidInit_M3508(void);
void Shoot_Speed_CanTransmit_M3508(void);

#ifdef __cplusplus
}
#endif

#endif /* define*/
