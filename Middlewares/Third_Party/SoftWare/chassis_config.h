#ifndef _CHASSIS_CONFIG_H
#define _CHASSIS_CONFIG_H


#include <stdint.h>
#include "config.h"

/******************************************************************************
  * @brief  底盘控制相关参数
  * @attention 双闭环控制 
  *     外环（速度环）采用增量式PID控制器
  *     内环（电流环）采用增量式PID控制器
  *     调试方法：
  *         1.定义 #define CHASSIS_DEBUG，可通过debug在线修改参数
  *         2.定义 #define CHASSIS_WAVE，可输出底盘相关波形
  *         具体实现方法在chassis.c中的void Chassis_Init(void)函数中
  *
  *         每次只能定义一个波形参数，调试完务必注释掉debug相关定义
  ****************************************************************************/

/* 进行调试前必须定义下面参数，其中波形参数只能定义一个 */
/* #define CHASSIS_DEBUG */
/* #define CHASSIS_WAVE */

//#define CHASSIS_DEBUG
//#define CHASSIS_WAVE

/* 速度环 */
float Chassis_Speed_Kp   = 12.0f; /*< 速度环P参数 */
float Chassis_Speed_Ki   = 0.15f; /*< 速度环I参数 */
float Chassis_Speed_Kd   = 0.0f; /*< 速度环D参数 */
float Chassis_Speed_Inc_Limit = 16000.0f; /*< 速度环增量限幅 */

/* 电流环 */
float Chassis_Current_Kp = 1.5f; /*< 电流环P参数 */
float Chassis_Current_Ki = 0.5f; /*< 速度环I参数 */
float Chassis_Current_Kd = 0.03f; /*< 速度环D参数 */
float Chassis_Current_Inc_Limit = 16000.0f; /*< 速度环增量限幅 */

float Chassis_Out_Limit = 16000.0f; /*< 输出限幅，3508最大接收16384*/


/* 动态小陀螺，移动越快，旋转越慢 */
float Move_Spin_Bias = 0.0001f; /*< 计算公式：旋转速度=旋转速度/（移速*bias+1） */
float Move_Spin_Limit = 0.1f;  /*< 旋转速度变化范围（正弦幅值），计算公式：最大旋转*limit */
  
#endif
