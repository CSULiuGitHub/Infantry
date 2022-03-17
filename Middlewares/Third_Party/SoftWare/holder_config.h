#ifndef _HOLDER_CONFIG_H
#define _HOLDER_CONFIG_H


#include <stdint.h>
#include "config.h"

/******************************************************************************
  * @brief  云台控制相关参数 大小风车相关参数
  * @attention 双闭环控制 
  *     外环（角度环）采用变结构PI控制器，减小超调，加快响应
  *     内环（速度环）采用绝对式PID控制器
  *     调试方法：
  *         1.定义 #define HOLDER_DEBUG，可通过debug在线修改参数
  *         2.定义 #define HOLDER_PITCH_WAVE，可输出PITCH轴波形
  *         3.定义 #define HOLDER_YAW_WAVE, 可输出YAW轴波形
  *         4.定义 #define HOLDER_VISION_WAVE, 可输出视觉波形
  *         具体实现方法在holder.c中的static void Holder_Pid_Manual(void)函数中
  *
  *         每次只能定义一个波形参数，调试完务必注释掉debug相关定义
  ****************************************************************************/

/* 进行调试前必须定义下面参数，其中波形参数只能定义一个 */
/* #define HOLDER_DEBUG */
/* #define HOLDER_PITCH_WAVE */
/* #define HOLDER_YAW_WAVE */
/* #define HOLDER_VISION_WAVE */

//#define HOLDER_VISION_WAVE
#define HOLDER_DEBUG
#define HOLDER_PITCH_WAVE
//#define VISION_NO_CP
#ifdef DEBUG
//    #define HOLDER_DEBUG
//    #define HOLDER_YAW_WAVE
//    #define HOLDER_PITCH_WAVE
    //#define HOLDER_VISION_WAVE
#endif

/*****************************所有机器通用参数**********************************/
float rc_pitch_bias = 0.15f; /*< 遥控器Pitch目标角度缩小比例 */
float rc_yaw_bias = -0.32f;  /*< 遥控器Yaw目标角度缩小比例 */
float key_pitch_bias = 1.0f; /*< 鼠标Pitch目标角度缩小比例 */
float key_yaw_bias = 1.0f;   /*< 鼠标Pitch目标角度缩小比例 */
int32_t key_yaw_limit = 250; /*< 鼠标YAW速度限幅 */

/*****************************不同机器不同参数**********************************/
#ifdef INFANTRY_3 /*< 三号步兵*/
/* YAW轴 */
float Yaw_Angle_Tunning_Kp = 0.05f;  /*< 角度环参数P变化值（变结构比例） */
float Yaw_Angle_Min_Kp     = 0.10f; /*< 角度环参数P最小固定值（变结构比例） */
float Yaw_Angle_Tunning_Ki = 0.0f;  /*< 角度环参数I变化值（变结构积分） */
float Yaw_Angle_Min_Ki     = 0.0f;  /*< 角度环参数I最小固定值（变结构积分） */
float Yaw_Angle_Kd         = 0.0f;  /*< 角度环D参数（微分） */

float Yaw_Angle_Vision_Tunning_Kp = 0.08f;  /*< 角度环参数P变化值（变结构比例） */
float Yaw_Angle_Vision_Min_Kp     = 0.03f; /*< 角度环参数P最小固定值（变结构比例） */
float Yaw_Angle_Vision_Tunning_Ki = 0.0f;  /*< 角度环参数I变化值（变结构积分） */
float Yaw_Angle_Vision_Min_Ki     = 0.0f;  /*< 角度环参数I最小固定值（变结构积分） */
float Yaw_Angle_Vision_Kd         = 0.0f;  /*< 角度环D参数（微分） */

float Yaw_Speed_kp = 5.0f;           /*< 速度环参数P（比例） */
float Yaw_Speed_ki = 0.01f;         /*< 速度环参数I（积分） */
float Yaw_Speed_kd = 0.0f;           /*< 速度环参数D（微分） */

float Yaw_Speed_Vision_kp = 5.0f;           /*< 速度环参数P（比例） */
float Yaw_Speed_Vision_ki = 0.01f;         /*< 速度环参数I（积分） */
float Yaw_Speed_Vision_kd = 0.0f;           /*< 速度环参数D（微分） */

float Yaw_Angle_Lpf_Bias = 0.9f;    /*< 角度环低通滤波系数 */
float Yaw_Speed_Lpf_Bias = 0.9f;    /*< 速度环低通滤波系数 */

float Yaw_Angle_Kp_Limit = 99999.0f;  /*< 角度环P比例误差限幅 */
float Yaw_Angle_Ki_Limit = 999.0f;  /*< 角度环I积分误差限幅 */
float Yaw_Speed_Kp_Limit = 99999.0f;  /*< 速度环P比例误差限幅 */
float Yaw_Speed_Ki_Limit = 10000.0f;  /*< 速度环I积分误差限幅 */

float Yaw_Angle_Vision_Kp_Limit = 99999.0f;  /*< 角度环P比例误差限幅 */
float Yaw_Angle_Vision_Ki_Limit = 99999.0f;  /*< 角度环I积分误差限幅 */

float Yaw_Output_Limit   = 28000.0f;  /*< 输出限幅，6020最大接收30000 */

/* PITCH轴 */
float Pitch_Angle_Tunning_Kp = 0.010f; /*< 角度环参数P变化值（变结构比例） */
float Pitch_Angle_Min_Kp     = 0.008f; /*< 角度环参数P最小固定值（变结构比例） */
float Pitch_Angle_Tunning_Ki = 0.0f;    /*< 角度环参数I变化值（变结构积分） */
float Pitch_Angle_Min_Ki     = 0.0f;    /*< 角度环参数I最小固定值（变结构积分） */
float Pitch_Angle_Kd         = 0.0f;    /*< 角度环D参数（微分） */

float Pitch_Angle_Vision_Tunning_Kp = 0.001f; /*< 角度环参数P变化值（变结构比例） */
float Pitch_Angle_Vision_Min_Kp     = 0.00035f; /*< 角度环参数P最小固定值（变结构比例） */
float Pitch_Angle_Vision_Tunning_Ki = 0.0f;    /*< 角度环参数I变化值（变结构积分） */
float Pitch_Angle_Vision_Min_Ki     = 0.0f;    /*< 角度环参数I最小固定值（变结构积分） */
float Pitch_Angle_Vision_Kd         = 0.0f;    /*< 角度环D参数（微分） */

float Pitch_Speed_kp = 385.0f;           /*< 速度环参数P（比例） */
float Pitch_Speed_ki = 0.52f;            /*< 速度环参数I（积分） */
float Pitch_Speed_kd = 0.0f;            /*< 速度环参数D（微分） */

float Pitch_Speed_Vision_kp = 325.0f;           /*< 速度环参数P（比例） */
float Pitch_Speed_Vision_ki = 0.12f;            /*< 速度环参数I（积分） */
float Pitch_Speed_Vision_kd = 0.0f;            /*< 速度环参数D（微分） */

float Pitch_Angle_Lpf_Bias = 0.8f;    /*< 角度环低通滤波系数 */
float Pitch_Speed_Lpf_Bias = 0.8f;    /*< 速度环低通滤波系数 */

float Pitch_Angle_Kp_Limit = 99999.0f;  /*< 角度环P比例误差限幅 */
float Pitch_Angle_Ki_Limit = 99999.0f;  /*< 角度环I积分误差限幅 */
float Pitch_Speed_Kp_Limit = 99999.0f;  /*< 速度环P比例误差限幅 */
float Pitch_Speed_Ki_Limit = 9999.0f;  /*< 速度环I积分误差限幅 */

float Pitch_Angle_Vision_Kp_Limit = 99999.0f;  /*< 角度环P比例误差限幅 */
float Pitch_Angle_Vision_Ki_Limit = 99999.0f;  /*< 角度环I积分误差限幅 */

float Pitch_Output_Limit   = 28000.0f;  /*< 输出限幅，6020最大接收30000 */


/********************************大小风车 角度环参数重调，速度环不变***************************/
/* YAW */
float Pinwheel_Yaw_Angle_Tunning_Kp = 1.8f; /*< 角度环参数P变化值（变结构比例） */
float Pinwheel_Yaw_Angle_Min_Kp     = 0.0f; /*< 角度环参数P最小固定值（变结构比例） */
float Pinwheel_Yaw_Angle_Tunning_Ki = 0.0f;    /*< 角度环参数I变化值（变结构积分） */
float Pinwheel_Yaw_Angle_Min_Ki     = 0.001f;    /*< 角度环参数I最小固定值（变结构积分） */
float Pinwheel_Yaw_Angle_Kd         = 0.8f;    /*< 角度环D参数（微分） */

/* PITCH */
float Pinwheel_Pitch_Angle_Tunning_Kp = 0.08f; /*< 角度环参数P变化值（变结构比例） */
float Pinwheel_Pitch_Angle_Min_Kp     = 0.04f; /*< 角度环参数P最小固定值（变结构比例） */
float Pinwheel_Pitch_Angle_Tunning_Ki = 0.0f;    /*< 角度环参数I变化值（变结构积分） */
float Pinwheel_Pitch_Angle_Min_Ki     = 0.0f;    /*< 角度环参数I最小固定值（变结构积分） */
float Pinwheel_Pitch_Angle_Kd         = 0.02f;    /*< 角度环D参数（微分） */
#endif /*< ifdef INFANTRY_3*/

#ifdef INFANTRY_4
/* YAW轴 */
float Yaw_Angle_Tunning_Kp = 0.05f;  /*< 角度环参数P变化值（变结构比例） */
float Yaw_Angle_Min_Kp     = 0.10f; /*< 角度环参数P最小固定值（变结构比例） */
float Yaw_Angle_Tunning_Ki = 0.0f;  /*< 角度环参数I变化值（变结构积分） */
float Yaw_Angle_Min_Ki     = 0.0f;  /*< 角度环参数I最小固定值（变结构积分） */
float Yaw_Angle_Kd         = 0.0f;  /*< 角度环D参数（微分） */

float Yaw_Angle_Vision_Tunning_Kp = 0.05f;  /*< 角度环参数P变化值（变结构比例） */
float Yaw_Angle_Vision_Min_Kp     = 0.03f; /*< 角度环参数P最小固定值（变结构比例） */
float Yaw_Angle_Vision_Tunning_Ki = 0.0f;  /*< 角度环参数I变化值（变结构积分） */
float Yaw_Angle_Vision_Min_Ki     = 0.0f;  /*< 角度环参数I最小固定值（变结构积分） */
float Yaw_Angle_Vision_Kd         = 0.0f;  /*< 角度环D参数（微分） */

float Yaw_Speed_kp = 5.0f;           /*< 速度环参数P（比例） */
float Yaw_Speed_ki = 0.01f;         /*< 速度环参数I（积分） */
float Yaw_Speed_kd = 0.0f;           /*< 速度环参数D（微分） */

float Yaw_Speed_Vision_kp = 5.0f;           /*< 速度环参数P（比例） */
float Yaw_Speed_Vision_ki = 0.01f;         /*< 速度环参数I（积分） */
float Yaw_Speed_Vision_kd = 0.0f;           /*< 速度环参数D（微分） */

float Yaw_Angle_Lpf_Bias = 0.9f;    /*< 角度环低通滤波系数 */
float Yaw_Speed_Lpf_Bias = 0.9f;    /*< 速度环低通滤波系数 */

float Yaw_Angle_Kp_Limit = 99999.0f;  /*< 角度环P比例误差限幅 */
float Yaw_Angle_Ki_Limit = 999.0f;  /*< 角度环I积分误差限幅 */
float Yaw_Speed_Kp_Limit = 99999.0f;  /*< 速度环P比例误差限幅 */
float Yaw_Speed_Ki_Limit = 10000.0f;  /*< 速度环I积分误差限幅 */

float Yaw_Angle_Vision_Kp_Limit = 99999.0f;  /*< 角度环P比例误差限幅 */
float Yaw_Angle_Vision_Ki_Limit = 99999.0f;  /*< 角度环I积分误差限幅 */

float Yaw_Output_Limit   = 28000.0f;  /*< 输出限幅，6020最大接收30000 */

/* PITCH轴 */
float Pitch_Angle_Tunning_Kp = 0.0045f; /*< 角度环参数P变化值（变结构比例） */
float Pitch_Angle_Min_Kp     = 0.0025f; /*< 角度环参数P最小固定值（变结构比例） */
float Pitch_Angle_Tunning_Ki = 0.0f;    /*< 角度环参数I变化值（变结构积分） */
float Pitch_Angle_Min_Ki     = 0.0f;    /*< 角度环参数I最小固定值（变结构积分） */
float Pitch_Angle_Kd         = 0.0f;    /*< 角度环D参数（微分） */

float Pitch_Angle_Vision_Tunning_Kp = 0.001f; /*< 角度环参数P变化值（变结构比例） */
float Pitch_Angle_Vision_Min_Kp     = 0.00035f; /*< 角度环参数P最小固定值（变结构比例） */
float Pitch_Angle_Vision_Tunning_Ki = 0.0f;    /*< 角度环参数I变化值（变结构积分） */
float Pitch_Angle_Vision_Min_Ki     = 0.0f;    /*< 角度环参数I最小固定值（变结构积分） */
float Pitch_Angle_Vision_Kd         = 0.0f;    /*< 角度环D参数（微分） */

float Pitch_Speed_kp = 385.0f;           /*< 速度环参数P（比例） */
float Pitch_Speed_ki = 0.52f;            /*< 速度环参数I（积分） */
float Pitch_Speed_kd = 0.0f;            /*< 速度环参数D（微分） */

float Pitch_Speed_Vision_kp = 325.0f;           /*< 速度环参数P（比例） */
float Pitch_Speed_Vision_ki = 0.12f;            /*< 速度环参数I（积分） */
float Pitch_Speed_Vision_kd = 0.0f;            /*< 速度环参数D（微分） */

float Pitch_Angle_Lpf_Bias = 0.8f;    /*< 角度环低通滤波系数 */
float Pitch_Speed_Lpf_Bias = 0.8f;    /*< 速度环低通滤波系数 */

float Pitch_Angle_Kp_Limit = 99999.0f;  /*< 角度环P比例误差限幅 */
float Pitch_Angle_Ki_Limit = 99999.0f;  /*< 角度环I积分误差限幅 */
float Pitch_Speed_Kp_Limit = 99999.0f;  /*< 速度环P比例误差限幅 */
float Pitch_Speed_Ki_Limit = 9999.0f;  /*< 速度环I积分误差限幅 */

float Pitch_Angle_Vision_Kp_Limit = 99999.0f;  /*< 角度环P比例误差限幅 */
float Pitch_Angle_Vision_Ki_Limit = 99999.0f;  /*< 角度环I积分误差限幅 */

float Pitch_Output_Limit   = 28000.0f;  /*< 输出限幅，6020最大接收30000 */


/********************************大小风车 角度环参数重调，速度环不变***************************/
/* YAW */
float Pinwheel_Yaw_Angle_Tunning_Kp = 1.8f; /*< 角度环参数P变化值（变结构比例） */
float Pinwheel_Yaw_Angle_Min_Kp     = 0.0f; /*< 角度环参数P最小固定值（变结构比例） */
float Pinwheel_Yaw_Angle_Tunning_Ki = 0.0f;    /*< 角度环参数I变化值（变结构积分） */
float Pinwheel_Yaw_Angle_Min_Ki     = 0.001f;    /*< 角度环参数I最小固定值（变结构积分） */
float Pinwheel_Yaw_Angle_Kd         = 0.8f;    /*< 角度环D参数（微分） */

/* PITCH */
float Pinwheel_Pitch_Angle_Tunning_Kp = 0.08f; /*< 角度环参数P变化值（变结构比例） */
float Pinwheel_Pitch_Angle_Min_Kp     = 0.04f; /*< 角度环参数P最小固定值（变结构比例） */
float Pinwheel_Pitch_Angle_Tunning_Ki = 0.0f;    /*< 角度环参数I变化值（变结构积分） */
float Pinwheel_Pitch_Angle_Min_Ki     = 0.0f;    /*< 角度环参数I最小固定值（变结构积分） */
float Pinwheel_Pitch_Angle_Kd         = 0.02f;    /*< 角度环D参数（微分） */
#endif /*< ifdef INFANTRY_4*/

#ifdef INFANTRY_5
/* YAW轴 */
float Yaw_Angle_Tunning_Kp = 0.05f;  /*< 角度环参数P变化值（变结构比例） */
float Yaw_Angle_Min_Kp     = 0.10f; /*< 角度环参数P最小固定值（变结构比例） */
float Yaw_Angle_Tunning_Ki = 0.0f;  /*< 角度环参数I变化值（变结构积分） */
float Yaw_Angle_Min_Ki     = 0.0f;  /*< 角度环参数I最小固定值（变结构积分） */
float Yaw_Angle_Kd         = 0.0f;  /*< 角度环D参数（微分） */

float Yaw_Angle_Vision_Tunning_Kp = 0.10f;  /*< 角度环参数P变化值（变结构比例） */
float Yaw_Angle_Vision_Min_Kp     = 0.05f; /*< 角度环参数P最小固定值（变结构比例） */
float Yaw_Angle_Vision_Tunning_Ki = 0.0f;  /*< 角度环参数I变化值（变结构积分） */
float Yaw_Angle_Vision_Min_Ki     = 0.0f;  /*< 角度环参数I最小固定值（变结构积分） */
float Yaw_Angle_Vision_Kd         = 0.0f;  /*< 角度环D参数（微分） */

float Yaw_Speed_kp = 5.0f;           /*< 速度环参数P（比例） */
float Yaw_Speed_ki = 0.01f;         /*< 速度环参数I（积分） */
float Yaw_Speed_kd = 0.0f;           /*< 速度环参数D（微分） */

float Yaw_Speed_Vision_kp = 5.0f;           /*< 速度环参数P（比例） */
float Yaw_Speed_Vision_ki = 0.01f;         /*< 速度环参数I（积分） */
float Yaw_Speed_Vision_kd = 0.0f;           /*< 速度环参数D（微分） */

float Yaw_Angle_Lpf_Bias = 0.9f;    /*< 角度环低通滤波系数 */
float Yaw_Speed_Lpf_Bias = 0.9f;    /*< 速度环低通滤波系数 */

float Yaw_Angle_Kp_Limit = 99999.0f;  /*< 角度环P比例误差限幅 */
float Yaw_Angle_Ki_Limit = 999.0f;  /*< 角度环I积分误差限幅 */
float Yaw_Speed_Kp_Limit = 99999.0f;  /*< 速度环P比例误差限幅 */
float Yaw_Speed_Ki_Limit = 10000.0f;  /*< 速度环I积分误差限幅 */

float Yaw_Angle_Vision_Kp_Limit = 99999.0f;  /*< 角度环P比例误差限幅 */
float Yaw_Angle_Vision_Ki_Limit = 99999.0f;  /*< 角度环I积分误差限幅 */

float Yaw_Output_Limit   = 28000.0f;  /*< 输出限幅，6020最大接收30000 */

/* PITCH轴 */
float Pitch_Angle_Tunning_Kp = 0.008f; /*< 角度环参数P变化值（变结构比例） */
float Pitch_Angle_Min_Kp     = 0.0055f; /*< 角度环参数P最小固定值（变结构比例） */
float Pitch_Angle_Tunning_Ki = 0.0f;    /*< 角度环参数I变化值（变结构积分） */
float Pitch_Angle_Min_Ki     = 0.0f;    /*< 角度环参数I最小固定值（变结构积分） */
float Pitch_Angle_Kd         = 0.0f;    /*< 角度环D参数（微分） */

float Pitch_Angle_Vision_Tunning_Kp = 0.002f; /*< 角度环参数P变化值（变结构比例）0.001f */ 
float Pitch_Angle_Vision_Min_Kp     = 0.00055f; /*< 角度环参数P最小固定值（变结构比例）0.00035f */
float Pitch_Angle_Vision_Tunning_Ki = 0.0f;    /*< 角度环参数I变化值（变结构积分） */
float Pitch_Angle_Vision_Min_Ki     = 0.0f;    /*< 角度环参数I最小固定值（变结构积分） */
float Pitch_Angle_Vision_Kd         = 0.0f;    /*< 角度环D参数（微分） */

float Pitch_Speed_kp = 325.0f;           /*< 速度环参数P（比例） */
float Pitch_Speed_ki = 0.12f;            /*< 速度环参数I（积分） */
float Pitch_Speed_kd = 0.0f;            /*< 速度环参数D（微分） */

float Pitch_Speed_Vision_kp = 325.0f;           /*< 速度环参数P（比例） */
float Pitch_Speed_Vision_ki = 0.12f;            /*< 速度环参数I（积分） */
float Pitch_Speed_Vision_kd = 0.0f;            /*< 速度环参数D（微分） */


float Pitch_Angle_Lpf_Bias = 0.8f;    /*< 角度环低通滤波系数 */
float Pitch_Speed_Lpf_Bias = 0.8f;    /*< 速度环低通滤波系数 */

float Pitch_Angle_Kp_Limit = 99999.0f;  /*< 角度环P比例误差限幅 */
float Pitch_Angle_Ki_Limit = 99999.0f;  /*< 角度环I积分误差限幅 */
float Pitch_Speed_Kp_Limit = 99999.0f;  /*< 速度环P比例误差限幅 */
float Pitch_Speed_Ki_Limit = 9999.0f;  /*< 速度环I积分误差限幅 */

float Pitch_Angle_Vision_Kp_Limit = 99999.0f;  /*< 角度环P比例误差限幅 */
float Pitch_Angle_Vision_Ki_Limit = 99999.0f;  /*< 角度环I积分误差限幅 */

float Pitch_Output_Limit   = 28000.0f;  /*< 输出限幅，6020最大接收30000 */


/********************************大小风车 角度环参数重调，速度环不变***************************/
/* YAW */
float Pinwheel_Yaw_Angle_Tunning_Kp = 1.8f; /*< 角度环参数P变化值（变结构比例） */
float Pinwheel_Yaw_Angle_Min_Kp     = 0.0f; /*< 角度环参数P最小固定值（变结构比例） */
float Pinwheel_Yaw_Angle_Tunning_Ki = 0.0f;    /*< 角度环参数I变化值（变结构积分） */
float Pinwheel_Yaw_Angle_Min_Ki     = 0.001f;    /*< 角度环参数I最小固定值（变结构积分） */
float Pinwheel_Yaw_Angle_Kd         = 0.8f;    /*< 角度环D参数（微分） */

/* PITCH */
float Pinwheel_Pitch_Angle_Tunning_Kp = 0.08f; /*< 角度环参数P变化值（变结构比例） */
float Pinwheel_Pitch_Angle_Min_Kp     = 0.04f; /*< 角度环参数P最小固定值（变结构比例） */
float Pinwheel_Pitch_Angle_Tunning_Ki = 0.0f;    /*< 角度环参数I变化值（变结构积分） */
float Pinwheel_Pitch_Angle_Min_Ki     = 0.0f;    /*< 角度环参数I最小固定值（变结构积分） */
float Pinwheel_Pitch_Angle_Kd         = 0.02f;    /*< 角度环D参数（微分） */
#endif /*< ifdef INFANTRY_5*/

#ifdef INFANTRY_6
/* YAW轴 */
float Yaw_Angle_Tunning_Kp = 0.05f;  /*< 角度环参数P变化值（变结构比例） */
float Yaw_Angle_Min_Kp     = 0.10f; /*< 角度环参数P最小固定值（变结构比例） */
float Yaw_Angle_Tunning_Ki = 0.0f;  /*< 角度环参数I变化值（变结构积分） */
float Yaw_Angle_Min_Ki     = 0.0f;  /*< 角度环参数I最小固定值（变结构积分） */
float Yaw_Angle_Kd         = 0.0f;  /*< 角度环D参数（微分） */

float Yaw_Angle_Vision_Tunning_Kp = 0.05f;  /*< 角度环参数P变化值（变结构比例） */
float Yaw_Angle_Vision_Min_Kp     = 0.03f; /*< 角度环参数P最小固定值（变结构比例） */
float Yaw_Angle_Vision_Tunning_Ki = 0.0f;  /*< 角度环参数I变化值（变结构积分） */
float Yaw_Angle_Vision_Min_Ki     = 0.0f;  /*< 角度环参数I最小固定值（变结构积分） */
float Yaw_Angle_Vision_Kd         = 0.0f;  /*< 角度环D参数（微分） */

float Yaw_Speed_kp = 5.0f;           /*< 速度环参数P（比例） */
float Yaw_Speed_ki = 0.01f;         /*< 速度环参数I（积分） */
float Yaw_Speed_kd = 0.0f;           /*< 速度环参数D（微分） */

float Yaw_Speed_Vision_kp = 5.0f;           /*< 速度环参数P（比例） */
float Yaw_Speed_Vision_ki = 0.01f;         /*< 速度环参数I（积分） */
float Yaw_Speed_Vision_kd = 0.0f;           /*< 速度环参数D（微分） */

float Yaw_Angle_Lpf_Bias = 0.9f;    /*< 角度环低通滤波系数 */
float Yaw_Speed_Lpf_Bias = 0.9f;    /*< 速度环低通滤波系数 */

float Yaw_Angle_Kp_Limit = 99999.0f;  /*< 角度环P比例误差限幅 */
float Yaw_Angle_Ki_Limit = 999.0f;  /*< 角度环I积分误差限幅 */
float Yaw_Speed_Kp_Limit = 99999.0f;  /*< 速度环P比例误差限幅 */
float Yaw_Speed_Ki_Limit = 10000.0f;  /*< 速度环I积分误差限幅 */

float Yaw_Angle_Vision_Kp_Limit = 99999.0f;  /*< 角度环P比例误差限幅 */
float Yaw_Angle_Vision_Ki_Limit = 99999.0f;  /*< 角度环I积分误差限幅 */

float Yaw_Output_Limit   = 28000.0f;  /*< 输出限幅，6020最大接收30000 */

/* PITCH轴 */
float Pitch_Angle_Tunning_Kp = 0.035; /*< 角度环参数P变化值（变结构比例） */
float Pitch_Angle_Min_Kp     = 0.02f; /*< 角度环参数P最小固定值（变结构比例） */
float Pitch_Angle_Tunning_Ki = 0.0f;    /*< 角度环参数I变化值（变结构积分） */
float Pitch_Angle_Min_Ki     = 0.0f;    /*< 角度环参数I最小固定值（变结构积分） */
float Pitch_Angle_Kd         = 0.1f;    /*< 角度环D参数（微分） */

float Pitch_Angle_Vision_Tunning_Kp = 0.001f; /*< 角度环参数P变化值（变结构比例） */
float Pitch_Angle_Vision_Min_Kp     = 0.00035f; /*< 角度环参数P最小固定值（变结构比例） */
float Pitch_Angle_Vision_Tunning_Ki = 0.0f;    /*< 角度环参数I变化值（变结构积分） */
float Pitch_Angle_Vision_Min_Ki     = 0.0f;    /*< 角度环参数I最小固定值（变结构积分） */
float Pitch_Angle_Vision_Kd         = 0.0f;    /*< 角度环D参数（微分） */

float Pitch_Speed_kp = 75.0f;           /*< 速度环参数P（比例） */
float Pitch_Speed_ki = 0.0f;            /*< 速度环参数I（积分） */
float Pitch_Speed_kd = 25.0f;            /*< 速度环参数D（微分） */

float Pitch_Speed_Vision_kp = 325.0f;           /*< 速度环参数P（比例） */
float Pitch_Speed_Vision_ki = 0.12f;            /*< 速度环参数I（积分） */
float Pitch_Speed_Vision_kd = 0.0f;            /*< 速度环参数D（微分） */

float Pitch_Angle_Lpf_Bias = 0.8f;    /*< 角度环低通滤波系数 */
float Pitch_Speed_Lpf_Bias = 0.8f;    /*< 速度环低通滤波系数 */

float Pitch_Angle_Kp_Limit = 99999.0f;  /*< 角度环P比例误差限幅 */
float Pitch_Angle_Ki_Limit = 99999.0f;  /*< 角度环I积分误差限幅 */
float Pitch_Speed_Kp_Limit = 99999.0f;  /*< 速度环P比例误差限幅 */
float Pitch_Speed_Ki_Limit = 9999.0f;  /*< 速度环I积分误差限幅 */

float Pitch_Angle_Vision_Kp_Limit = 99999.0f;  /*< 角度环P比例误差限幅 */
float Pitch_Angle_Vision_Ki_Limit = 99999.0f;  /*< 角度环I积分误差限幅 */

float Pitch_Output_Limit   = 28000.0f;  /*< 输出限幅，6020最大接收30000 */

/********************************大小风车 角度环参数重调，速度环不变***************************/
/* YAW */
float Pinwheel_Yaw_Angle_Tunning_Kp = 1.8f; /*< 角度环参数P变化值（变结构比例） */
float Pinwheel_Yaw_Angle_Min_Kp     = 0.0f; /*< 角度环参数P最小固定值（变结构比例） */
float Pinwheel_Yaw_Angle_Tunning_Ki = 0.0f;    /*< 角度环参数I变化值（变结构积分） */
float Pinwheel_Yaw_Angle_Min_Ki     = 0.001f;    /*< 角度环参数I最小固定值（变结构积分） */
float Pinwheel_Yaw_Angle_Kd         = 0.8f;    /*< 角度环D参数（微分） */

/* PITCH */
float Pinwheel_Pitch_Angle_Tunning_Kp = 0.08f; /*< 角度环参数P变化值（变结构比例） */
float Pinwheel_Pitch_Angle_Min_Kp     = 0.04f; /*< 角度环参数P最小固定值（变结构比例） */
float Pinwheel_Pitch_Angle_Tunning_Ki = 0.0f;    /*< 角度环参数I变化值（变结构积分） */
float Pinwheel_Pitch_Angle_Min_Ki     = 0.0f;    /*< 角度环参数I最小固定值（变结构积分） */
float Pinwheel_Pitch_Angle_Kd         = 0.02f;    /*< 角度环D参数（微分） */
#endif /*< ifdef INFANTRY_6*/

#endif /*< ifdef HOLDER_CONFIG_H*/
