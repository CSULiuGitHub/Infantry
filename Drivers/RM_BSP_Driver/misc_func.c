/**
  ******************************************************************************
  * @file    
  * @author  qj
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
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/

/***************************************限幅***********************************/

/**
  * @brief  限幅
  * @param  目标变量，最小值，最大值
  * @retval int类型限幅值
  * @attention 限幅函数比较常用，大都在任务中循环执行
  */
int constrain(int amt, int low, int high)
{
    return amt<low? low:(amt>high? high:amt); /*< 问号表达式运行比if else快 */
}

float constrain_float(float amt, float low, float high)
{
	return amt<low? low:(amt>high? high:amt); /*< 问号表达式运行比if else快 */
}

int16_t constrain_int16_t(int16_t amt, int16_t low, int16_t high)
{
	return amt<low? low:(amt>high? high:amt); /*< 问号表达式运行比if else快 */
}

int32_t Constrain_Int32_t(int32_t amt, int32_t low, int32_t high)
{
	return amt<low? low:(amt>high? high:amt); /*< 问号表达式运行比if else快 */
}

uint16_t Constrain_Uint16_t(uint16_t amt, uint16_t low, uint16_t high)
{
	return amt<low? low:(amt>high? high:amt); /*< 问号表达式运行比if else快 */
}


/***************************************斜坡变化***********************************/

/**
  * @brief  斜坡函数,使目标输出值缓慢等于期望值
  * @param  期望最终输出,当前输出,变化速度(每次增加的值)
  * @retval 当前输出
  * @attention  
  */
float RAMP_float( float final, float now, float ramp )
{
    float buffer = 0;

    buffer = final - now;

    if (buffer > 0)
    {
        if (buffer > ramp)
        {  
            now += ramp;
        }   
        else
        {
            now += buffer;
        }
    }
    else
    {
        if (buffer < -ramp)
        {
            now += -ramp;
        }
        else
        {
            now += buffer;
        }
    }

    return now;
}


int32_t Misc_RAMP_Int32( int32_t final, int32_t now, int32_t ramp )
{
    int32_t buffer = 0;

    buffer = final - now;

    if (buffer > 0)
    {
        if (buffer > ramp)
        {  
            now += ramp;
        }   
        else
        {
            now += buffer;
        }
    }
    else
    {
        if (buffer < -ramp)
        {
            now += -ramp;
        }
        else
        {
            now += buffer;
        }
    }

    return now;
}


int16_t Misc_RAMP_Int16( int16_t final, int16_t now, int16_t ramp )
{
    int16_t buffer = 0;

    buffer = final - now;

    if (buffer > 0)
    {
        if (buffer > ramp)
        {  
            now += ramp;
        }   
        else
        {
            now += buffer;
        }
    }
    else
    {
        if (buffer < -ramp)
        {
            now += -ramp;
        }
        else
        {
            now += buffer;
        }
    }

    return now;
}

/**
  * @brief  斜坡函数,使目标输出值缓慢等于期望值
  * @param  期望最终输出,当前输出,变化速度(越大越快)
  * @retval 当前输出
  * @attention  
  */
uint16_t RAMP_Uint16( uint16_t final, uint16_t now, uint16_t ramp )
{
    int16_t buffer = 0;

    buffer = final - now;

    if (buffer > 0)
    {
        if (buffer > ramp)
        {  
            now += ramp;
        }   
        else
        {
            now += buffer;
        }
    }
    else
    {
        if (buffer < -ramp)
        {
            now += -ramp;
        }
        else
        {
            now += buffer;
        }
    }

    return now;
}



/**
  * @brief  斜坡函数,使目标输出值缓慢等于指针输入值
  * @param  要在当前输出量上累加的值,目标输出量,递增快慢
  * @retval 目标输出量
  * @attention  
  *             
*/
float RampInc_float( float *buffer, float now, float ramp )
{

    if (*buffer > 0)
    {
        if (*buffer > ramp)
        {  
            now     += ramp;
            *buffer -= ramp;
        }   
        else
        {
            now     += *buffer;
            *buffer  = 0;
        }
    }
    else
    {
        if (*buffer < -ramp)
        {
            now     += -ramp;
            *buffer -= -ramp;
        }
        else
        {
            now     += *buffer;
            *buffer  = 0;
        }
    }
    
    return now;
}


/***************************************数学***********************************/

/**
  * @brief  Quake-III Arena (雷神之锤3) 神秘代码
  * @param  x
  * @retval 根号x
  * @attention  大神写的快速平方根    
  */
float Quake_Sqrt(float x)
{
    if(x == 0) return 0; 
    float result = x; 
    float xhalf = 0.5f*result; 
    int i = *(int*)&result; 

    i = 0x5f3759df - (i>>1); 
    result = *(float*)&i; 
    result = result*(1.5f-xhalf*result*result);
    result = result*(1.5f-xhalf*result*result); 
    return 1.0f/result; 
}


/**
  * @brief  float取绝对值
  * @param  x
  * @retval |x|
  * @attention  
  */
float Misc_Fabsf(float x)
{
    float y;
    y = x>=0? x:-x;
    return y;
}

/**
  * @brief  float取绝对值
  * @param  x
  * @retval |x|
  * @attention  
  */
uint16_t Misc_U16absu16(uint16_t x)
{
    uint16_t y;
    y = x>0? x:-x;
    return y;
}


/***************************************信号处理***********************************/
/**
  * @brief  递推（滑动）平均滤波
  * @param  
  * @retval void
  * @attention 
  */
#define  RM_BUF_LEN                         4 /*递推数组的长度*/
int16_t RM_Filter_Buf[RM_BUF_LEN];            /*保存数据的递推数组*/

int16_t Misc_s16Recursive_Mean_Filter(int16_t get_data)
{
    int16_t sum = 0;
    uint8_t i;
    
    for(i = 0; i < RM_BUF_LEN - 1; i++) 
    {
		RM_Filter_Buf[i] = RM_Filter_Buf[i+1];  /*数组中所有数据左移一位 第一个数据扔掉*/
		sum += RM_Filter_Buf[i];                /*计算递推数组中处最后一个前面所有数据的和*/
	}
    
    RM_Filter_Buf[RM_BUF_LEN - 1] = get_data;
    
    sum += RM_Filter_Buf[RM_BUF_LEN - 1];                     /*计算递推数组所有数据的和*/
    sum = sum / RM_BUF_LEN;
    return sum;
}

int32_t Misc_s32Recursive_Mean4_Filter(int32_t get_data, int32_t RM4_Filter_Buf[4])
{
    int32_t sum = 0;
    uint8_t i;
    
    for(i = 0; i < 3; i++) 
    {
		RM4_Filter_Buf[i] = RM4_Filter_Buf[i+1];  /*数组中所有数据左移一位 第一个数据扔掉*/
		sum += RM4_Filter_Buf[i];                 /*计算递推数组中处最后一个前面所有数据的和*/
	}
    
    RM4_Filter_Buf[3] = get_data;
    
    sum =  RM4_Filter_Buf[0]*0.1 + RM4_Filter_Buf[1]*0.2 +RM4_Filter_Buf[2]*0.3 + RM4_Filter_Buf[3]*0.4;
    return sum;
}

float Misc_fRecursive_Mean4_Filter(float get_data, float RM4_Filter_Buf[4])
{
    float sum = 0;
    uint8_t i;
    
    for(i = 0; i < 3; i++) 
    {
		RM4_Filter_Buf[i] = RM4_Filter_Buf[i+1];  /*数组中所有数据左移一位 第一个数据扔掉*/
		sum += RM4_Filter_Buf[i];                 /*计算递推数组中处最后一个前面所有数据的和*/
	}
    
    RM4_Filter_Buf[3] = get_data;
    
    sum =  RM4_Filter_Buf[0]*0.1f + RM4_Filter_Buf[1]*0.2f +RM4_Filter_Buf[2]*0.3f + RM4_Filter_Buf[3]*0.4f;
    return sum;
}
