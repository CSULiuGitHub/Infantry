/**
  ******************************************************************************
  * @file    
  * @author  v5.0hxl
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
#include "judge_tx.h"
#include "dma_unpack.h"
#include "chassis.h"
#include "holder.h"
#include "string.h"
#include "vision.h"
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
uint8_t StrTmp[30] = {0}; /*�ַ����������*/
ext_client_graphic_drawseven_t ext_client_graphic_drawseven;
ext_client_graphic_drawtext_t judge_txmsg;
/* function ------------------------------------------------------------------*/
/*
    �������,��ʾ������,�ݱ���
	ext_client_graphic_draw.ext_client_graphic_draw.radius = (numberint)&0x3FF;
    ext_client_graphic_draw.ext_client_graphic_draw.end_x = (numberint>>10)&0x7FF;
    ext_client_graphic_draw.ext_client_graphic_draw.end_y = (numberint>>21)&0x3FF;
*/
/**
  * @brief  ��ö�Ӧ���տͻ���ID
  * @param  uint16_t sender_ID
  * @retval uint16_t client_ID
  * @attention 
  */
static uint16_t get_client_ID(uint16_t sender_ID)
{
    uint16_t client_ID;
    switch (sender_ID)
    {
    case RED1_robotID:
        client_ID = 0x0101;
        break;
    case RED2_robotID:
        client_ID = 0x0102;
        break;
    case RED3_robotID:
        client_ID = 0x0103;
        break;
    case RED4_robotID:
        client_ID = 0x0104;
        break;
    case RED5_robotID:
        client_ID = 0x0105;
        break;
    case RED6_robotID:
        client_ID = 0x0106;
        break;

    case BLUE1_robotID:
        client_ID = 0x0165;
        break;
    case BLUE2_robotID:
        client_ID = 0x0166;
        break;
    case BLUE3_robotID:
        client_ID = 0x0167;
        break;
    case BLUE4_robotID:
        client_ID = 0x0168;
        break;
    case BLUE5_robotID:
        client_ID = 0x0169;
        break;
    case BLUE6_robotID:
        client_ID = 0x016A;
        break;
    }
    return client_ID;
}

/**
  * @brief  ���͵��ָ�����
  * @param  uint16_t sender_ID���Ͷ�ID
  * @param  uint8_t Lid_State���ָ�״̬
  * @param  uint8_t operate_tpye��ͼ��Ĳ���
  * @retval void
  * @attention 
  */
void JudgeUI_SendLidSt(uint16_t sender_ID, uint8_t Lid_State, uint8_t operate_tpye)
{
    uint8_t i;
    judge_txmsg.ext_client_graphic_basic.data_comment_ID = 0x0110;
    judge_txmsg.ext_client_graphic_basic.sender_ID = sender_ID;
    judge_txmsg.ext_client_graphic_basic.client_ID = get_client_ID(sender_ID);
    judge_txmsg.ext_client_graphic_draw.graphic_name[0] = '0';
    judge_txmsg.ext_client_graphic_draw.graphic_name[1] = '0';
    judge_txmsg.ext_client_graphic_draw.graphic_name[2] = '1';
    judge_txmsg.ext_client_graphic_draw.operate_tpye = operate_tpye;
    judge_txmsg.ext_client_graphic_draw.start_angle = 25;
    judge_txmsg.ext_client_graphic_draw.end_angle = 6;
    judge_txmsg.ext_client_graphic_draw.graphic_tpye = 7;
    judge_txmsg.ext_client_graphic_draw.layer = 9;
    judge_txmsg.ext_client_graphic_draw.color = 0;
    judge_txmsg.ext_client_graphic_draw.width = 5;
    judge_txmsg.ext_client_graphic_draw.start_x = 200;
    judge_txmsg.ext_client_graphic_draw.start_y = 650;
    judge_txmsg.ext_client_graphic_draw.radius = 0;
    judge_txmsg.ext_client_graphic_draw.end_x = 0;
    judge_txmsg.ext_client_graphic_draw.end_y = 0;
    for (i = 0; i < 30; i++)
    {
        judge_txmsg.text[i] = 0;
    }
    judge_txmsg.text[0] = 'L';
    judge_txmsg.text[1] = 'i';
    judge_txmsg.text[2] = 'd';
    judge_txmsg.text[3] = ':';
    judge_txmsg.text[4] = Lid_State;

    if (Lid_State == 'G') /*���ָ�û����Ϊ��ɫ*/
        judge_txmsg.ext_client_graphic_draw.color = 8;

    /*���ݷ�װ*/
    data_packet_pack((0x0301), (uint8_t *)&judge_txmsg, sizeof(ext_client_graphic_drawtext_t), 0xA5);

    /*DMA����*/
    HAL_UART_Transmit_DMA(&huart3, uart3_tx_buf, sizeof(uart3_tx_buf));
}

/**
  * @brief  ����С����״̬
  * @param  uint16_t sender_ID���Ͷ�ID, 
  * @param  uint8_t spin_stateС����״̬,
  * @param  uint8_t operate_tpye��ͼ��Ĳ���
  * @retval void
  * @attention 
  */
void JudgeUI_SendSpinMode(uint16_t sender_ID, uint8_t spin_state, uint8_t operate_tpye)
{
    int i;
    judge_txmsg.ext_client_graphic_basic.data_comment_ID = 0x0110;
    judge_txmsg.ext_client_graphic_basic.sender_ID = sender_ID;
    judge_txmsg.ext_client_graphic_basic.client_ID = get_client_ID(sender_ID);
    judge_txmsg.ext_client_graphic_draw.graphic_name[0] = '0';
    judge_txmsg.ext_client_graphic_draw.graphic_name[1] = '0';
    judge_txmsg.ext_client_graphic_draw.graphic_name[2] = '2';
    judge_txmsg.ext_client_graphic_draw.operate_tpye = operate_tpye;
    judge_txmsg.ext_client_graphic_draw.start_angle = 25;
    judge_txmsg.ext_client_graphic_draw.end_angle = 6;
    judge_txmsg.ext_client_graphic_draw.graphic_tpye = 7;
    judge_txmsg.ext_client_graphic_draw.layer = 8;
    judge_txmsg.ext_client_graphic_draw.color = 0;
    judge_txmsg.ext_client_graphic_draw.width = 5;
    judge_txmsg.ext_client_graphic_draw.start_x = 200;
    judge_txmsg.ext_client_graphic_draw.start_y = 600;
    judge_txmsg.ext_client_graphic_draw.radius = 0;
    judge_txmsg.ext_client_graphic_draw.end_x = 0;
    judge_txmsg.ext_client_graphic_draw.end_y = 0;
    for (i = 0; i < 30; i++)
    {
        judge_txmsg.text[i] = 0;
    }
    judge_txmsg.text[0] = 'M';
    judge_txmsg.text[1] = 'o';
    judge_txmsg.text[2] = 'd';
    judge_txmsg.text[3] = 'e';
    judge_txmsg.text[4] = ':';
    judge_txmsg.text[5] = spin_state;

    if (spin_state == 'f') /*����ģʽΪ��ɫ*/
        judge_txmsg.ext_client_graphic_draw.color = 8;

    /*���ݷ�װ*/
    data_packet_pack((0x0301), (uint8_t *)&judge_txmsg, sizeof(ext_client_graphic_drawtext_t), 0xA5);

    /*DMA����*/
    HAL_UART_Transmit_DMA(&huart3, uart3_tx_buf, sizeof(uart3_tx_buf));
}

/**
  * @brief  ���ʹ��״̬
  * @param  uint16_t sender_ID���Ͷ�ID, 
  * @param  uint8_t vision_state���״̬,
  * @param  uint8_t operate_tpye��ͼ��Ĳ���
  * @retval void
  * @attention 
  */
void JudgeUI_SendVisionState(uint16_t sender_ID, uint8_t vision_state, uint8_t operate_tpye)
{
    int i;
    judge_txmsg.ext_client_graphic_basic.data_comment_ID = 0x0110;
    judge_txmsg.ext_client_graphic_basic.sender_ID = sender_ID;
    judge_txmsg.ext_client_graphic_basic.client_ID = get_client_ID(sender_ID);
    judge_txmsg.ext_client_graphic_draw.graphic_name[0] = '0';
    judge_txmsg.ext_client_graphic_draw.graphic_name[1] = '0';
    judge_txmsg.ext_client_graphic_draw.graphic_name[2] = '5';
    judge_txmsg.ext_client_graphic_draw.operate_tpye = operate_tpye;
    judge_txmsg.ext_client_graphic_draw.start_angle = 25;
    judge_txmsg.ext_client_graphic_draw.end_angle = 6;
    judge_txmsg.ext_client_graphic_draw.graphic_tpye = 7;
    judge_txmsg.ext_client_graphic_draw.layer = 8;
    judge_txmsg.ext_client_graphic_draw.color = 0;
    judge_txmsg.ext_client_graphic_draw.width = 5;
    judge_txmsg.ext_client_graphic_draw.start_x = 200;
    judge_txmsg.ext_client_graphic_draw.start_y = 550;
    judge_txmsg.ext_client_graphic_draw.radius = 0;
    judge_txmsg.ext_client_graphic_draw.end_x = 0;
    judge_txmsg.ext_client_graphic_draw.end_y = 0;
    for (i = 0; i < 30; i++)
    {
        judge_txmsg.text[i] = 0;
    }
    judge_txmsg.text[0] = 'V';
    judge_txmsg.text[1] = 'i';
    judge_txmsg.text[2] = 's';
    judge_txmsg.text[3] = 'i';
    judge_txmsg.text[4] = ':';
    judge_txmsg.text[5] = vision_state;

    if (vision_state == 'A') /*����ģʽΪ��ɫ*/
        judge_txmsg.ext_client_graphic_draw.color = 8;

    /*���ݷ�װ*/
    data_packet_pack((0x0301), (uint8_t *)&judge_txmsg, sizeof(ext_client_graphic_drawtext_t), 0xA5);

    /*DMA����*/
    HAL_UART_Transmit_DMA(&huart3, uart3_tx_buf, sizeof(uart3_tx_buf));
}



/**
  * @brief  ���ͳ�����������
  * @param  uint16_t sender_ID���Ͷ�ID,uint8_t *Str,float tmp,uint8_t operate_tpye��ͼ��Ĳ���
  * @retval void
  * @attention 
  */
void JudgeUI_SendSuperCapData(uint16_t sender_ID, uint8_t *Str, float tmp, uint8_t operate_tpye)
{
    uint8_t i;
    judge_txmsg.ext_client_graphic_basic.data_comment_ID = 0x0110;
    judge_txmsg.ext_client_graphic_basic.sender_ID = sender_ID;
    judge_txmsg.ext_client_graphic_basic.client_ID = get_client_ID(sender_ID);
    judge_txmsg.ext_client_graphic_draw.graphic_name[0] = '0';
    judge_txmsg.ext_client_graphic_draw.graphic_name[1] = '0';
    judge_txmsg.ext_client_graphic_draw.graphic_name[2] = '3';
    judge_txmsg.ext_client_graphic_draw.operate_tpye = operate_tpye;
    judge_txmsg.ext_client_graphic_draw.graphic_tpye = 7;
    judge_txmsg.ext_client_graphic_draw.start_angle = 25;
    judge_txmsg.ext_client_graphic_draw.end_angle = 6;
    judge_txmsg.ext_client_graphic_draw.layer = 9;
    judge_txmsg.ext_client_graphic_draw.color = 0;
    judge_txmsg.ext_client_graphic_draw.width = 3;
    judge_txmsg.ext_client_graphic_draw.start_x = 1400;
    judge_txmsg.ext_client_graphic_draw.start_y = 700;
    judge_txmsg.ext_client_graphic_draw.radius = 0;
    judge_txmsg.ext_client_graphic_draw.end_x = 0;
    judge_txmsg.ext_client_graphic_draw.end_y = 0;
    for (i = 0; i < 30; i++)
        judge_txmsg.text[i] = 0;
    memcpy(judge_txmsg.text, Str, sizeof(Str) + 1);
    sprintf((char *)&StrTmp, "%4.2f", tmp);
    memcpy(judge_txmsg.text + sizeof(Str) + 1, StrTmp, sizeof(StrTmp));

    /*���ݷ�װ*/
    data_packet_pack(0x0301, (uint8_t *)&judge_txmsg, sizeof(judge_txmsg), 0xA5);
    /*DMA���ͣ�����ַ���*/
    HAL_UART_Transmit_DMA(&huart3, uart3_tx_buf, sizeof(uart3_tx_buf));
    memset(&StrTmp, 0, sizeof(StrTmp));
}

///**
//  * @brief  ����Pitch����������
//  * @param  uint16_t sender_ID���Ͷ�ID,uint8_t *Str,float tmp,uint8_t operate_tpye��ͼ��Ĳ���
//  * @retval void
//  * @attention 
//  */
//void JudgeUI_SendHolderPitchData(uint16_t sender_ID, uint8_t *Str, float tmp, uint8_t operate_tpye)
//{
//    uint8_t i;
//    judge_txmsg.ext_client_graphic_basic.data_comment_ID = 0x0110;
//    judge_txmsg.ext_client_graphic_basic.sender_ID = sender_ID;
//    judge_txmsg.ext_client_graphic_basic.client_ID = get_client_ID(sender_ID);
//    judge_txmsg.ext_client_graphic_draw.graphic_name[0] = '0';
//    judge_txmsg.ext_client_graphic_draw.graphic_name[1] = '0';
//    judge_txmsg.ext_client_graphic_draw.graphic_name[2] = '4';
//    judge_txmsg.ext_client_graphic_draw.operate_tpye = operate_tpye;
//    judge_txmsg.ext_client_graphic_draw.graphic_tpye = 7;
//    judge_txmsg.ext_client_graphic_draw.start_angle = 25;
//    judge_txmsg.ext_client_graphic_draw.end_angle = 6;
//    judge_txmsg.ext_client_graphic_draw.layer = 9;
//    judge_txmsg.ext_client_graphic_draw.color = 0;
//    judge_txmsg.ext_client_graphic_draw.width = 3;
//    judge_txmsg.ext_client_graphic_draw.start_x = 200;
//    judge_txmsg.ext_client_graphic_draw.start_y = 800;
//    judge_txmsg.ext_client_graphic_draw.radius = 0;
//    judge_txmsg.ext_client_graphic_draw.end_x = 0;
//    judge_txmsg.ext_client_graphic_draw.end_y = 0;
//    for (i = 0; i < 30; i++)
//        judge_txmsg.text[i] = 0;
//    memcpy(judge_txmsg.text, Str, sizeof(Str) + 3);
//    sprintf((char *)&StrTmp, "%4.2f", tmp);
//    memcpy(judge_txmsg.text + sizeof(Str) + 3, StrTmp, sizeof(StrTmp));

//    /*���ݷ�װ*/
//    data_packet_pack(0x0301, (uint8_t *)&judge_txmsg, sizeof(judge_txmsg), 0xA5);
//    /*DMA���ͣ�����ַ���*/
//    HAL_UART_Transmit_DMA(&huart3, uart3_tx_buf, sizeof(uart3_tx_buf));
//    memset(&StrTmp, 0, sizeof(StrTmp));
//}

/**
  * @brief  ����Visionz����
  * @param  uint16_t sender_ID���Ͷ�ID,uint8_t *Str,float tmp,uint8_t operate_tpye��ͼ��Ĳ���
  * @retval void
  * @attention 
  */
void JudgeUI_SendVisionzData(uint16_t sender_ID, uint8_t *Str, float tmp, uint8_t operate_tpye)
{
    uint8_t i;
    judge_txmsg.ext_client_graphic_basic.data_comment_ID = 0x0110;
    judge_txmsg.ext_client_graphic_basic.sender_ID = sender_ID;
    judge_txmsg.ext_client_graphic_basic.client_ID = get_client_ID(sender_ID);
    judge_txmsg.ext_client_graphic_draw.graphic_name[0] = '0';
    judge_txmsg.ext_client_graphic_draw.graphic_name[1] = '0';
    judge_txmsg.ext_client_graphic_draw.graphic_name[2] = '4';
    judge_txmsg.ext_client_graphic_draw.operate_tpye = operate_tpye;
    judge_txmsg.ext_client_graphic_draw.graphic_tpye = 7;
    judge_txmsg.ext_client_graphic_draw.start_angle = 25;
    judge_txmsg.ext_client_graphic_draw.end_angle = 6;
    judge_txmsg.ext_client_graphic_draw.layer = 9;
    judge_txmsg.ext_client_graphic_draw.color = 0;
    judge_txmsg.ext_client_graphic_draw.width = 3;
    judge_txmsg.ext_client_graphic_draw.start_x = 200;
    judge_txmsg.ext_client_graphic_draw.start_y = 800;
    judge_txmsg.ext_client_graphic_draw.radius = 0;
    judge_txmsg.ext_client_graphic_draw.end_x = 0;
    judge_txmsg.ext_client_graphic_draw.end_y = 0;
    for (i = 0; i < 30; i++)
        judge_txmsg.text[i] = 0;
    memcpy(judge_txmsg.text, Str, sizeof(Str) + 3);
    sprintf((char *)&StrTmp, "%4.2f", tmp);
    memcpy(judge_txmsg.text + sizeof(Str) + 3, StrTmp, sizeof(StrTmp));

    /*���ݷ�װ*/
    data_packet_pack(0x0301, (uint8_t *)&judge_txmsg, sizeof(judge_txmsg), 0xA5);
    /*DMA���ͣ�����ַ���*/
    HAL_UART_Transmit_DMA(&huart3, uart3_tx_buf, sizeof(uart3_tx_buf));
    memset(&StrTmp, 0, sizeof(StrTmp));
}


/**
  * @brief  ���͵���λ������ͼ�Ρ��������Լ��Ӵ���׼����������仯��
  * @param  uint16_t sender_ID���Ͷ�ID, uint16_t len��������, uint8_t operate_tpye��ͼ��Ĳ���
  * @retval void
  * @attention 
  */
void JudgeUI_SendBKG2_SuperCap_Line(uint16_t sender_ID, uint16_t len, uint8_t operate_tpye)
{
    ext_client_graphic_drawseven.ext_client_graphic_basic.data_comment_ID = 0x0104;
    ext_client_graphic_drawseven.ext_client_graphic_basic.sender_ID = sender_ID;
    ext_client_graphic_drawseven.ext_client_graphic_basic.client_ID = get_client_ID(sender_ID);

    ext_client_graphic_drawseven.ext_client_graphic_draw[0].operate_tpye = operate_tpye;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].operate_tpye = operate_tpye;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].operate_tpye = operate_tpye;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].operate_tpye = operate_tpye;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].operate_tpye = operate_tpye; //�仯����
	ext_client_graphic_drawseven.ext_client_graphic_draw[5].operate_tpye = operate_tpye;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].operate_tpye = operate_tpye;
	
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].graphic_name[0] = '5';
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].graphic_name[2] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].layer = 9;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].color = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].width = 2;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].start_x = 500;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].start_y = 100;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].end_x = 760;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].end_y = 200;

    ext_client_graphic_drawseven.ext_client_graphic_draw[1].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].graphic_name[0] = '5';
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].graphic_name[2] = '1';
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].color = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].width = 2;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].start_x = 760;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].start_y = 200;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].end_x = 920;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].end_y = 200;

    ext_client_graphic_drawseven.ext_client_graphic_draw[2].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].graphic_name[0] = '5';
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].graphic_name[2] = '2';
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].color = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].width = 2;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].start_x = 1000;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].start_y = 200;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].end_x = 1160;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].end_y = 200;

    ext_client_graphic_drawseven.ext_client_graphic_draw[3].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].graphic_name[0] = '5';
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].graphic_name[2] = '3';
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].color = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].width = 2;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].start_x = 1160;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].start_y = 200;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].end_x = 1420;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].end_y = 100;

    ext_client_graphic_drawseven.ext_client_graphic_draw[4].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].graphic_name[0] = '5';
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].graphic_name[2] = '4';
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].layer = 9;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].color = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].width = 5;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].start_x = 1610;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].start_y = 800;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].end_x = 1610+len;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].end_y = 800;
	
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].graphic_name[0] = '5';
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].graphic_name[2] = '5';
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].color = 6;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].width = 4;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].start_x = 920;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].start_y = 550;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].end_x = 1000;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].end_y = 550;

    ext_client_graphic_drawseven.ext_client_graphic_draw[6].graphic_tpye = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].graphic_name[0] = '5';
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].graphic_name[2] = '6';
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].color = 5;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].width = 2;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].start_x = 1607;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].start_y = 803;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].end_x = 1912;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].end_y = 794;

    /*���ݷ�װ*/
    data_packet_pack((0x0301), (uint8_t *)&ext_client_graphic_drawseven, sizeof(ext_client_graphic_drawseven_t), 0xA5);

    /*DMA����*/
    HAL_UART_Transmit_DMA(&huart3, uart3_tx_buf, sizeof(uart3_tx_buf));
}

/**
  * @brief  ������׼Ŀ��ͼ�Σ�����仯��
  * @param  uint16_t sender_ID���Ͷ�ID, uint16_t client_ID���ն�ID��ʱ���ù�, uint8_t operate_tpye��ͼ��Ĳ���
  * @retval void
  * @attention 
  */
void JudgeUI_SendBKG1(uint16_t sender_ID,uint8_t operate_tpye)
{

    int8_t i;
    ext_client_graphic_drawseven.ext_client_graphic_basic.data_comment_ID = 0x0104;
    ext_client_graphic_drawseven.ext_client_graphic_basic.sender_ID = sender_ID;
    ext_client_graphic_drawseven.ext_client_graphic_basic.client_ID = get_client_ID(sender_ID);
    for (i = 0; i < 7; i++)
    {
        ext_client_graphic_drawseven.ext_client_graphic_draw[i].operate_tpye = operate_tpye;
    }

    ext_client_graphic_drawseven.ext_client_graphic_draw[0].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].graphic_name[0] = '7';
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].graphic_name[2] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].color = 6;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].width = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].start_x = 950;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].start_y = 270;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].end_x = 970;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].end_y = 270;

    ext_client_graphic_drawseven.ext_client_graphic_draw[1].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].graphic_name[0] = '7';
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].graphic_name[2] = '1';
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].color = 6;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].width = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].start_x = 930;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].start_y = 300;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].end_x = 990;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].end_y = 300;

    ext_client_graphic_drawseven.ext_client_graphic_draw[2].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].graphic_name[0] = '7';
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].graphic_name[2] = '2';
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].color = 6;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].width = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].start_x = 900;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].start_y = 350;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].end_x = 1020;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].end_y = 350;

    ext_client_graphic_drawseven.ext_client_graphic_draw[3].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].graphic_name[0] = '7';
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].graphic_name[2] = '3';
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].color = 6;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].width = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].start_x = 880;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].start_y = 400;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].end_x = 1040;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].end_y = 400;

    ext_client_graphic_drawseven.ext_client_graphic_draw[4].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].graphic_name[0] = '7';
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].graphic_name[2] = '4';
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].color = 6;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].width = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].start_x = 850;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].start_y = 450;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].end_x = 1070;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].end_y = 450;

    ext_client_graphic_drawseven.ext_client_graphic_draw[5].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].graphic_name[0] = '7';
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].graphic_name[2] = '5';
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].color = 6;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].width = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].start_x = 960;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].start_y = 200;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].end_x = 960;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].end_y = 600;

    ext_client_graphic_drawseven.ext_client_graphic_draw[6].graphic_tpye = 2;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].graphic_name[0] = '7';
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].graphic_name[2] = '6';
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].color = 6;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].width = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].start_x = 960;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].start_y = 400;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].radius = 150;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].end_x = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].end_y = 0;

    /*���ݷ�װ*/
    data_packet_pack((0x0301), (uint8_t *)&ext_client_graphic_drawseven, sizeof(ext_client_graphic_drawseven_t), 0xA5);

    /*DMA����*/
    HAL_UART_Transmit_DMA(&huart3, uart3_tx_buf, sizeof(uart3_tx_buf));
}

/**
* @brief  ����ͼ2�ͳ���������������
  * @param  void
  * @retval void
  * @attention 
  */
void JudgeUI_BK2_CapLine(uint8_t robot_id)
{
	static uint8_t tick = 0;
	uint16_t length;
	uint16_t CapValue = Chassis.SuperCap.CapVol;
    length = (CapValue-1500)/3;
    if(CapValue < 1500)
        length = 0;
	if(tick < 5)
	{
		JudgeUI_SendBKG2_SuperCap_Line((uint16_t)(robot_id&0x00FF),length,2);
	}
	else
	{
		JudgeUI_SendBKG2_SuperCap_Line((uint16_t)(robot_id&0x00FF),length,1);
		if(tick == 5)
			tick = 0;
	}
	tick++;
}

/**
* @brief  ����������ֵ����
  * @param  void
  * @retval void
  * @attention 
  */
void JudgeUI_CapData(uint8_t robot_id)
{
	static uint8_t tick = 0;
	uint8_t Strc[] = "Cap: ";
	if(tick < 5)
	{
		JudgeUI_SendSuperCapData((uint16_t)(robot_id&0x00FF),Strc,(float)Chassis.SuperCap.CapVol,2);
	}
	else
	{
		JudgeUI_SendSuperCapData((uint16_t)(robot_id&0x00FF),Strc,(float)Chassis.SuperCap.CapVol,1);
		if(tick == 5)
			tick = 0;
	}
	tick++;
}

///**
//* @brief  �����Ƿ���
//  * @param  void
//  * @retval void
//  * @attention 
//  */
//void JudgeUI_PitchData(uint8_t robot_id)
//{
//	static uint8_t tick = 0;
//	uint8_t Strp[] = "Pitch: ";
//	if(tick < 5)
//	{
//		JudgeUI_SendHolderPitchData((uint16_t)(robot_id&0x00FF),Strp, (float)Holder.Pitch.Angle,2);
//	}
//	else
//	{
//		JudgeUI_SendHolderPitchData((uint16_t)(robot_id&0x00FF),Strp, (float)Holder.Pitch.Angle,1);
//		if(tick == 5)
//			tick = 0;
//	}
//	tick++;
//}

/**
* @brief  �Ӿ�����z����
  * @param  void
  * @retval void
  * @attention 
  */
void JudgeUI_VisionzData(uint8_t robot_id)
{
	static uint8_t tick = 0;
	uint8_t Strp[] = "VData_z: ";
	if(tick < 5)
	{
		JudgeUI_SendVisionzData((uint16_t)(robot_id&0x00FF),Strp, (float)vData_z,2);
	}
	else
	{
		JudgeUI_SendVisionzData((uint16_t)(robot_id&0x00FF),Strp, (float)vData_z,1);
		if(tick == 5)
			tick = 0;
	}
	tick++;
}

/**
* @brief  С����״̬���� 
  * @param  uint8_t state 'f':follow  'l':spin
  * @retval void
  * @attention 
  */
void JudgeUI_SpinMode(uint8_t robot_id, uint8_t state)
{
	static uint8_t tick = 0;
	if(tick < 5)
	{
		JudgeUI_SendSpinMode((uint16_t)(robot_id&0x00FF),state,2);
	}
	else
	{
		JudgeUI_SendSpinMode((uint16_t)(robot_id&0x00FF),state,1);
		if(tick == 5)
			tick = 0;
	}
	tick++;
}

/**
* @brief  ���״̬���� 
  * @param  uint8_t state 'W':WINDMILL  'A':AUTO
  * @retval void
  * @attention 
  */
void JudgeUI_Vision_State(uint8_t robot_id, uint8_t state)
{
	static uint8_t tick = 0;
	if(tick < 5)
	{
		JudgeUI_SendVisionState((uint16_t)(robot_id&0x00FF),state,2);
	}
	else
	{
		JudgeUI_SendVisionState((uint16_t)(robot_id&0x00FF),state,1);
		if(tick == 5)
			tick = 0;
	}
	tick++;
}

/**
  * @brief  ���ո�״̬����
  * @param  uint8_t state Y/N
  * @retval void
  * @attention 
  */
void JudgeUI_LidState(uint8_t robot_id, uint8_t Lid_St)
{
	static uint8_t tick = 0;
    if(tick < 5)
	{
		JudgeUI_SendLidSt((uint16_t)(robot_id&0x00FF),Lid_St,2);
	}
	else
	{
		JudgeUI_SendLidSt((uint16_t)(robot_id&0x00FF),Lid_St,1);
		if(tick == 5)
			tick = 0;
	}
	tick++;

}
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
