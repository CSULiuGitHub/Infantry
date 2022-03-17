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
#include "misc_func.h"
#include "vision.h"
#include "config.h"
#include "string.h"
#include "judge.h"
#include "cmsis_os.h"
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
vData_t vData = {0};
uint8_t global_vision_state = VISION_MANUAL;
/* function ------------------------------------------------------------------*/

float vData_x_RMF4[4];
float vData_y_RMF4[4];
float vData_z_RMF4[4];
int16_t vData_x;
int16_t vData_y;
int16_t vData_z;
uint8_t vData_buf[16];

#if defined(INFANTRY_4) || defined(INFANTRY_3) || defined(INFANTRY_6)
void Vision_RecvData(uint8_t byte)
{
    vData.buf[vData.index] = byte;
    if (vData.buf[vData.index] == 0x03 && vData.buf[vData.index - 1] == 0xFC)
    {
        if (vData.buf[vData.index - 15] == 0X03 && vData.buf[vData.index - 14] == 0XFC)
        {
            memcpy(&vData.Pos, &vData.buf[vData.index - 13], 12);
            vData.index = 0;
        }
    }
    
    vData_x = vData.Pos.x * 100*10;
    vData_y = vData.Pos.y * 100*10;
    vData_z = vData.Pos.z * 100;
    
    vData.index++;
    if (vData.index == 200)
        vData.index = 0;
}
#endif

#ifdef INFANTRY_5
void Vision_RecvData(uint8_t byte)
{
    static uint8_t count = 0;
    
    vData_buf[count] = byte;
    if (vData_buf[count] == 0x7b || count>0)
    {
        count++;
    }
    else
    {
        count = 0;
    }
    
    if (count == 8)
    {
        count = 0; /*< 重新开始接收 */
        if (vData_buf[7] == 0x7d)
        {
            vData_x = ((vData_buf[1]<<8) + vData_buf[2])*10;
            vData_y = ((vData_buf[3]<<8) + vData_buf[4])*10;  
            vData_z = ((vData_buf[5]<<8) + vData_buf[6]);  
        }
    }
}
#endif

void Vision_SendData(RemoteData_t RDMsg)
{
    uint8_t tmp_data[6]; 
	static uint16_t tx_tick = 0;
    static uint8_t delay_tick = 0;
    static uint8_t color_state  = 0; 
    static uint8_t detect_state = 0;
    uint8_t id = (uint8_t)JUDGE_u8GetRobotId();
    
    if (id <= 7 && id >0)
    {
        color_state = 1;
    }
    else if (id > 7)
    {
        color_state = 0;
    }
    
    if (delay_tick < 10)
    {
        delay_tick++;
    }
    
    if (RDMsg.KeyBoard.v == 1 && detect_state == 0 && delay_tick == 10)
    {
        detect_state = 1;
        delay_tick = 0;
    }
    else if (RDMsg.KeyBoard.v == 1 && detect_state == 1 && delay_tick == 10)
    {
        detect_state = 0;
        delay_tick = 0;
    }
    
    switch(color_state)
    {
        case 0:
            tmp_data[2] = 0x00; /*< 红色*/
        break;

        case 1:
            tmp_data[2] = 0x01; /*< 蓝色*/
        break;
        
        default:
            tmp_data[2] = 0x00; /*< 红色*/
        break;
    }
    
    
    switch(detect_state)
    {
        case 0:
            tmp_data[3] = 0x00; /*< 自瞄*/
            global_vision_state = VISION_AUTO;
	 	    JudgeUI_vision_state = 'A';
        break;

        case 1:
            tmp_data[3] = 0x01; /*< 大符*/
            global_vision_state = VISION_WINDMILL;
		    JudgeUI_vision_state = 'W';
        break;
        
        default:
            tmp_data[3] = 0x00; /*< 自瞄*/
            global_vision_state = VISION_AUTO;
		    JudgeUI_vision_state = 'A';
        break;
    }

    /* 帧头帧尾 */
    tmp_data[0] = 0x03;
    tmp_data[1] = 0xFC;

    tmp_data[4] = 0xFC;
    tmp_data[5] = 0x03;
	tx_tick++;
	if(tx_tick > 100)
	{
		HAL_UART_Transmit_IT(&huart6, tmp_data, 6);
		tx_tick = 0;
	}
    osDelay(1);
}
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
