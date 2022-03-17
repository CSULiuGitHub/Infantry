#ifndef _SHOOT_CONFIG_H
#define _SHOOT_CONFIG_H
#include <stdint.h>
#include "config.h"

/******************************************************************************
  * @brief  发射机构控制相关参数
  * @attention 每次只能定义一个波形参数，调试完务必注释掉debug相关定义
  *     1.拨弹电机2006单环绝对式PID控制
  *     速度环）采用绝对式PID控制器
  *     调试方法：
  *         1.定义 #define SHOOT_FREQ_DEBUG，射频控制，可通过debug在线修改参数
  *         2.定义 #define SHOOT_FREQ_WAVE，射频控制，可输出拨弹电机波形
  *         3.定义 #define SHOOT_SPEED_DEBUG, 射速控制
  *         4.定义 #define SHOOT_SPEED_WAVE, 射速波形
  *         具体实现方法在shoot.c中的static void Shoot_PidRun(void)函数中
  *
  *     2.摩擦轮电机snail开环控制
  ****************************************************************************/

/* 进行调试前必须定义下面参数，其中波形参数只能定义一个 */
/* #define SHOOT_FREQ_DEBUG */
/* #define SHOOT_FREQ_WAVE  */

/* 射频控制 */
//#define SHOOT_FREQ_DEBUG
//#define SHOOT_FREQ_WAVE 

/* 射速控制 */
//#define SHOOT_SPEED_DEBUG
//#define SHOOT_SPEED_WAVE 

/*****************************所有机器通用参数**********************************/
int32_t Shoot_Freq_Out_Limit = 10000; /*< 2006输出-10000 ~ 10000 */

uint16_t Freq_Max = 600; /*< 最大射频 */
uint16_t Freq_Min = 50; /*< 最小射频 */
float Buf_Safety  = 35;   /*< 安全热量下限 */

/* 摩擦轮电机和射速控制 */
/* 根据射速上限分级 */
float Tar_Speed_Lim15 = 13.5; /*< 射速上限为15 单位(m/s)*/
float Tar_Speed_Lim18 = 16.5;
float Tar_Speed_Lim22 = 20.5;
float Tar_Speed_Lim30 = 28.5;

/* 3508摩擦轮专用，勿预编译 */
int16_t Shoot_Speed_Max_M3508_Lim15 = 4300;
int16_t Shoot_Speed_Max_M3508_Lim18 = 4700;
int16_t Shoot_Speed_Max_M3508_Lim22 = 5300;
int16_t Shoot_Speed_Max_M3508_Lim30 = 7000;

int32_t M3508_Out_Limit = 16384; /*< 3508输出电流+-16384 */


/*****************************不同机器不同参数**********************************/
#ifdef INFANTRY_3
/* 速度环 */
float Shoot_Freq_Speed_Kp = 10.5;
float Shoot_Freq_Speed_Ki = 0.8;
float Shoot_Freq_Speed_Kd = 8;
float Shoot_Freq_Speed_Kp_Limit = 9999;
float Shoot_Freq_Speed_Ki_Limit = 9999;

/* 位置环 */
float Shoot_Freq_Position_Kp = 1.5;
float Shoot_Freq_Position_Ki = 0;
float Shoot_Freq_Position_Kd = 0;
float Shoot_Freq_Position_Kp_Limit = 9999;
float Shoot_Freq_Position_Ki_Limit = 9999;

/* 摩擦轮电机限制占空比，绝对防止超速 （测试500发，不出现超速的最大占空比）*/
uint16_t Pusel_Max_Lim15 = 240;
uint16_t Pusel_Max_Lim18 = 251;
uint16_t Pusel_Max_Lim22 = 305;
uint16_t Pusel_Max_Lim30 = 355;
uint16_t Pulse_Min = 235;
uint16_t Pulse_Out = 240; /*< 初始射速 */
#endif /*< ifdef INFANTRY_3 */

#ifdef INFANTRY_4
/* 速度环 */
float Shoot_Freq_Speed_Kp = 10.5;
float Shoot_Freq_Speed_Ki = 0.8;
float Shoot_Freq_Speed_Kd = 8;
float Shoot_Freq_Speed_Kp_Limit = 9999;
float Shoot_Freq_Speed_Ki_Limit = 9999;

/* 位置环 */
float Shoot_Freq_Position_Kp = 1.5;
float Shoot_Freq_Position_Ki = 0;
float Shoot_Freq_Position_Kd = 0;
float Shoot_Freq_Position_Kp_Limit = 9999;
float Shoot_Freq_Position_Ki_Limit = 9999;

/* 摩擦轮电机限制占空比，绝对防止超速 */
uint16_t Pusel_Max_Lim15 = 237;
uint16_t Pusel_Max_Lim18 = 250;
uint16_t Pusel_Max_Lim22 = 290;
uint16_t Pusel_Max_Lim30 = 455;
uint16_t Pulse_Min = 230;
uint16_t Pulse_Out = 237; /*< 初始射速 */
#endif /*< ifdef INFANTRY_4 */

#ifdef INFANTRY_5
/* 速度环 */
float Shoot_Freq_Speed_Kp = 10.5;
float Shoot_Freq_Speed_Ki = 0.8;
float Shoot_Freq_Speed_Kd = 8;
float Shoot_Freq_Speed_Kp_Limit = 9999;
float Shoot_Freq_Speed_Ki_Limit = 9999;

/* 位置环 */
float Shoot_Freq_Position_Kp = 1.5;
float Shoot_Freq_Position_Ki = 0;
float Shoot_Freq_Position_Kd = 0;
float Shoot_Freq_Position_Kp_Limit = 9999;
float Shoot_Freq_Position_Ki_Limit = 9999;

/* 摩擦轮电机限制占空比，绝对防止超速 */
uint16_t Pusel_Max_Lim15 = 235;
uint16_t Pusel_Max_Lim18 = 249;
uint16_t Pusel_Max_Lim22 = 290;
uint16_t Pusel_Max_Lim30 = 455;
uint16_t Pulse_Min = 235;
uint16_t Pulse_Out = 240; /*< 初始射速 */
#endif /*< ifdef INFANTRY_5 */

#ifdef INFANTRY_6
/* 两个3508电机（摩擦轮） */
/* 速度环 */
float Shoot_Freq_Speed_Kp_M3508 = 50;
float Shoot_Freq_Speed_Ki_M3508 = 0;
float Shoot_Freq_Speed_Kd_M3508 = 15;
float Shoot_Freq_Speed_Kp_Limit_M3508 = 9999;
float Shoot_Freq_Speed_Ki_Limit_M3508 = 9999;

/* 拨弹电机 */
/* 速度环 */
float Shoot_Freq_Speed_Kp = 10.5;
float Shoot_Freq_Speed_Ki = 0.8;
float Shoot_Freq_Speed_Kd = 8;
float Shoot_Freq_Speed_Kp_Limit = 9999;
float Shoot_Freq_Speed_Ki_Limit = 9999;

/* 位置环 */
float Shoot_Freq_Position_Kp = 1.5;
float Shoot_Freq_Position_Ki = 0;
float Shoot_Freq_Position_Kd = 0;
float Shoot_Freq_Position_Kp_Limit = 9999;
float Shoot_Freq_Position_Ki_Limit = 9999;

/* 摩擦轮电机限制占空比，绝对防止超速 */
uint16_t Pusel_Max_Lim15 = 235;
uint16_t Pusel_Max_Lim18 = 249;
uint16_t Pusel_Max_Lim22 = 290;
uint16_t Pusel_Max_Lim30 = 455;
uint16_t Pulse_Min = 235;
uint16_t Pulse_Out = 240; /*< 初始射速 */
#endif /*< ifdef INFANTRY_6 */

#endif
