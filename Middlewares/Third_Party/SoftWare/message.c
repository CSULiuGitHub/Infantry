/**
  ******************************************************************************
  * @file    
  * @author  sy
  * @brief
  * @date     
  ******************************************************************************
  * @attention
  *
  * Copyright (c) CSU_RM_FYT.
  * All rights reserved.
  *
  * This software component is licensed by SY under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  * opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* includes ------------------------------------------------------------------*/
#include "message.h"
#include "imu_data_decode.h"
#include "dma.h"
#include "usart.h"
#include "tim.h"
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
static Mcircle_t mc_imu_yaw = {0};
static Mcircle_t mc_imu_pitch = {0};
static float Eular[3] = {0};
static int16_t Gyro[3] = {0};
/* function ------------------------------------------------------------------*/
void RemoteDataMsg_Process(RemoteData_t *RDMsg)
{    
    RDMsg->Ch0  = (  (int16_t) uart1_buf[0]       | ( (int16_t) uart1_buf[1]  << 8 )) & 0x07FF;
    RDMsg->Ch0 -= 1024;
    RDMsg->Ch1  = (( (int16_t) uart1_buf[1] >> 3) | ( (int16_t) uart1_buf[2]  << 5 )) & 0x07FF;
    RDMsg->Ch1 -= 1024;
    RDMsg->Ch2  = (( (int16_t) uart1_buf[2] >> 6) | ( (int16_t) uart1_buf[3]  << 2 )  
                                                  | ( (int16_t) uart1_buf[4]  << 10)) & 0x07FF;
    RDMsg->Ch2 -= 1024;
    RDMsg->Ch3  = (( (int16_t) uart1_buf[4] >> 1) | ( (int16_t) uart1_buf[5]  << 7 )) & 0x07FF;
    RDMsg->Ch3 -= 1024;
    RDMsg->S1   = (            uart1_buf[5] >> 6)                                     & 0x03;
    RDMsg->S2   = (            uart1_buf[5] >> 4)                                     & 0x03;
         
    RDMsg->Mouse_x = ( (int16_t) uart1_buf[6] | (int16_t) uart1_buf[7] << 8);
    RDMsg->Mouse_y = ( (int16_t) uart1_buf[8] | (int16_t) uart1_buf[9] << 8);
       
    RDMsg->MouseClick_left  = uart1_buf[12];
    RDMsg->MouseClick_right = uart1_buf[13]; 
      
    RDMsg->Key     = ( (int16_t) uart1_buf[14] |   (int16_t) uart1_buf[15] << 8 );
    RDMsg->Wheel   = ( (int16_t) uart1_buf[16] | ( (int16_t) uart1_buf[17] << 8 )) & 0x07FF;
    RDMsg->Wheel   = -RDMsg->Wheel + 1024;

    if(RDMsg->Ch0 > 660 || RDMsg->Ch0 < -660)
        RDMsg->Ch0 = 0;
    if(RDMsg->Ch1 > 660 || RDMsg->Ch1 < -660)
        RDMsg->Ch1 = 0;
    if(RDMsg->Ch2 > 660 || RDMsg->Ch2 < -660)
        RDMsg->Ch2 = 0;
    if(RDMsg->Ch3 > 660 || RDMsg->Ch3 < -660)
        RDMsg->Ch3 = 0;
    if(RDMsg->Wheel > 660 || RDMsg->Wheel < -660)
        RDMsg->Wheel = 0;    
    
    RDMsg->KeyBoard.w     = RDMsg->Key & KEY_PRESSED_OFFSET_W;
    RDMsg->KeyBoard.s     = (RDMsg->Key & KEY_PRESSED_OFFSET_S)>>1;
    RDMsg->KeyBoard.a     = (RDMsg->Key & KEY_PRESSED_OFFSET_A)>>2;
    RDMsg->KeyBoard.d     = (RDMsg->Key & KEY_PRESSED_OFFSET_D)>>3;
    RDMsg->KeyBoard.shift = (RDMsg->Key & KEY_PRESSED_OFFSET_SHIFT)>>4;
    RDMsg->KeyBoard.ctrl  = (RDMsg->Key & KEY_PRESSED_OFFSET_CTRL)>>5;
    RDMsg->KeyBoard.q     = (RDMsg->Key & KEY_PRESSED_OFFSET_Q)>>6;
    RDMsg->KeyBoard.e     = (RDMsg->Key & KEY_PRESSED_OFFSET_E)>>7;
    RDMsg->KeyBoard.r     = (RDMsg->Key & KEY_PRESSED_OFFSET_R)>>8;
    RDMsg->KeyBoard.f     = (RDMsg->Key & KEY_PRESSED_OFFSET_F)>>9;
    RDMsg->KeyBoard.g     = (RDMsg->Key & KEY_PRESSED_OFFSET_G)>>10;
    RDMsg->KeyBoard.z     = (RDMsg->Key & KEY_PRESSED_OFFSET_Z)>>11;
    RDMsg->KeyBoard.x     = (RDMsg->Key & KEY_PRESSED_OFFSET_X)>>12;
    RDMsg->KeyBoard.c     = (RDMsg->Key & KEY_PRESSED_OFFSET_C)>>13;
    RDMsg->KeyBoard.v     = (RDMsg->Key & KEY_PRESSED_OFFSET_V)>>14;
    RDMsg->KeyBoard.b     = (RDMsg->Key & KEY_PRESSED_OFFSET_B)>>15; 
}

static float IMUAngle_Continue(Mcircle_t *mc_imu,float imu_angle)
{
	float out_data = 0; 

	Circle_Continue(mc_imu, (imu_angle + 180.0f) * 22.7555556f);
	out_data = 819 * (float)(mc_imu->Circle + (float)(mc_imu->Angle) / 8192);

	return out_data;    
}

void HolderDataMsg_Process(HolderData_t *HDMsg)
{
    get_eular(Eular);
    
    HDMsg->Angle[2] = IMUAngle_Continue(&mc_imu_yaw,Eular[2]);
    HDMsg->Angle[0] = IMUAngle_Continue(&mc_imu_pitch,Eular[0]);
    
    get_raw_gyo(Gyro);
    HDMsg->Gyro[2] = -(Gyro[2]);
    HDMsg->Gyro[0] = -Gyro[0];
}


/*********************************ÏûÏ¢¸¨Öúº¯Êý*********************************/

Mcircle_t* Get_Mc_Imu_Pitch(void)
{
    return &mc_imu_pitch;
}

/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
