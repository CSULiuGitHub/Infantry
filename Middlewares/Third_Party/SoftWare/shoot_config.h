#ifndef _SHOOT_CONFIG_H
#define _SHOOT_CONFIG_H
#include <stdint.h>
#include "config.h"

/******************************************************************************
  * @brief  �������������ز���
  * @attention ÿ��ֻ�ܶ���һ�����β��������������ע�͵�debug��ض���
  *     1.�������2006��������ʽPID����
  *     �ٶȻ������þ���ʽPID������
  *     ���Է�����
  *         1.���� #define SHOOT_FREQ_DEBUG����Ƶ���ƣ���ͨ��debug�����޸Ĳ���
  *         2.���� #define SHOOT_FREQ_WAVE����Ƶ���ƣ�����������������
  *         3.���� #define SHOOT_SPEED_DEBUG, ���ٿ���
  *         4.���� #define SHOOT_SPEED_WAVE, ���ٲ���
  *         ����ʵ�ַ�����shoot.c�е�static void Shoot_PidRun(void)������
  *
  *     2.Ħ���ֵ��snail��������
  ****************************************************************************/

/* ���е���ǰ���붨��������������в��β���ֻ�ܶ���һ�� */
/* #define SHOOT_FREQ_DEBUG */
/* #define SHOOT_FREQ_WAVE  */

/* ��Ƶ���� */
//#define SHOOT_FREQ_DEBUG
//#define SHOOT_FREQ_WAVE 

/* ���ٿ��� */
//#define SHOOT_SPEED_DEBUG
//#define SHOOT_SPEED_WAVE 

/*****************************���л���ͨ�ò���**********************************/
int32_t Shoot_Freq_Out_Limit = 10000; /*< 2006���-10000 ~ 10000 */

uint16_t Freq_Max = 600; /*< �����Ƶ */
uint16_t Freq_Min = 50; /*< ��С��Ƶ */
float Buf_Safety  = 35;   /*< ��ȫ�������� */

/* Ħ���ֵ�������ٿ��� */
/* �����������޷ּ� */
float Tar_Speed_Lim15 = 13.5; /*< ��������Ϊ15 ��λ(m/s)*/
float Tar_Speed_Lim18 = 16.5;
float Tar_Speed_Lim22 = 20.5;
float Tar_Speed_Lim30 = 28.5;

/* 3508Ħ����ר�ã���Ԥ���� */
int16_t Shoot_Speed_Max_M3508_Lim15 = 4300;
int16_t Shoot_Speed_Max_M3508_Lim18 = 4700;
int16_t Shoot_Speed_Max_M3508_Lim22 = 5300;
int16_t Shoot_Speed_Max_M3508_Lim30 = 7000;

int32_t M3508_Out_Limit = 16384; /*< 3508�������+-16384 */


/*****************************��ͬ������ͬ����**********************************/
#ifdef INFANTRY_3
/* �ٶȻ� */
float Shoot_Freq_Speed_Kp = 10.5;
float Shoot_Freq_Speed_Ki = 0.8;
float Shoot_Freq_Speed_Kd = 8;
float Shoot_Freq_Speed_Kp_Limit = 9999;
float Shoot_Freq_Speed_Ki_Limit = 9999;

/* λ�û� */
float Shoot_Freq_Position_Kp = 1.5;
float Shoot_Freq_Position_Ki = 0;
float Shoot_Freq_Position_Kd = 0;
float Shoot_Freq_Position_Kp_Limit = 9999;
float Shoot_Freq_Position_Ki_Limit = 9999;

/* Ħ���ֵ������ռ�ձȣ����Է�ֹ���� ������500���������ֳ��ٵ����ռ�ձȣ�*/
uint16_t Pusel_Max_Lim15 = 240;
uint16_t Pusel_Max_Lim18 = 251;
uint16_t Pusel_Max_Lim22 = 305;
uint16_t Pusel_Max_Lim30 = 355;
uint16_t Pulse_Min = 235;
uint16_t Pulse_Out = 240; /*< ��ʼ���� */
#endif /*< ifdef INFANTRY_3 */

#ifdef INFANTRY_4
/* �ٶȻ� */
float Shoot_Freq_Speed_Kp = 10.5;
float Shoot_Freq_Speed_Ki = 0.8;
float Shoot_Freq_Speed_Kd = 8;
float Shoot_Freq_Speed_Kp_Limit = 9999;
float Shoot_Freq_Speed_Ki_Limit = 9999;

/* λ�û� */
float Shoot_Freq_Position_Kp = 1.5;
float Shoot_Freq_Position_Ki = 0;
float Shoot_Freq_Position_Kd = 0;
float Shoot_Freq_Position_Kp_Limit = 9999;
float Shoot_Freq_Position_Ki_Limit = 9999;

/* Ħ���ֵ������ռ�ձȣ����Է�ֹ���� */
uint16_t Pusel_Max_Lim15 = 237;
uint16_t Pusel_Max_Lim18 = 250;
uint16_t Pusel_Max_Lim22 = 290;
uint16_t Pusel_Max_Lim30 = 455;
uint16_t Pulse_Min = 230;
uint16_t Pulse_Out = 237; /*< ��ʼ���� */
#endif /*< ifdef INFANTRY_4 */

#ifdef INFANTRY_5
/* �ٶȻ� */
float Shoot_Freq_Speed_Kp = 10.5;
float Shoot_Freq_Speed_Ki = 0.8;
float Shoot_Freq_Speed_Kd = 8;
float Shoot_Freq_Speed_Kp_Limit = 9999;
float Shoot_Freq_Speed_Ki_Limit = 9999;

/* λ�û� */
float Shoot_Freq_Position_Kp = 1.5;
float Shoot_Freq_Position_Ki = 0;
float Shoot_Freq_Position_Kd = 0;
float Shoot_Freq_Position_Kp_Limit = 9999;
float Shoot_Freq_Position_Ki_Limit = 9999;

/* Ħ���ֵ������ռ�ձȣ����Է�ֹ���� */
uint16_t Pusel_Max_Lim15 = 235;
uint16_t Pusel_Max_Lim18 = 249;
uint16_t Pusel_Max_Lim22 = 290;
uint16_t Pusel_Max_Lim30 = 455;
uint16_t Pulse_Min = 235;
uint16_t Pulse_Out = 240; /*< ��ʼ���� */
#endif /*< ifdef INFANTRY_5 */

#ifdef INFANTRY_6
/* ����3508�����Ħ���֣� */
/* �ٶȻ� */
float Shoot_Freq_Speed_Kp_M3508 = 50;
float Shoot_Freq_Speed_Ki_M3508 = 0;
float Shoot_Freq_Speed_Kd_M3508 = 15;
float Shoot_Freq_Speed_Kp_Limit_M3508 = 9999;
float Shoot_Freq_Speed_Ki_Limit_M3508 = 9999;

/* ������� */
/* �ٶȻ� */
float Shoot_Freq_Speed_Kp = 10.5;
float Shoot_Freq_Speed_Ki = 0.8;
float Shoot_Freq_Speed_Kd = 8;
float Shoot_Freq_Speed_Kp_Limit = 9999;
float Shoot_Freq_Speed_Ki_Limit = 9999;

/* λ�û� */
float Shoot_Freq_Position_Kp = 1.5;
float Shoot_Freq_Position_Ki = 0;
float Shoot_Freq_Position_Kd = 0;
float Shoot_Freq_Position_Kp_Limit = 9999;
float Shoot_Freq_Position_Ki_Limit = 9999;

/* Ħ���ֵ������ռ�ձȣ����Է�ֹ���� */
uint16_t Pusel_Max_Lim15 = 235;
uint16_t Pusel_Max_Lim18 = 249;
uint16_t Pusel_Max_Lim22 = 290;
uint16_t Pusel_Max_Lim30 = 455;
uint16_t Pulse_Min = 235;
uint16_t Pulse_Out = 240; /*< ��ʼ���� */
#endif /*< ifdef INFANTRY_6 */

#endif
