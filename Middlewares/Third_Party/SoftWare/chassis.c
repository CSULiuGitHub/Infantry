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
#include "chassis_config.h"
#include "chassis.h"
#include <string.h>
#include "holder.h"
#include "usart.h"
#include "tim.h"
#include "can.h"
#include "fast_tri_func.h"
#include "misc_func.h"
#include "judge.h"
#include "judge_tx.h"
#include "ui.h"
#include "FreeRTOS.h"
#include "task.h"
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
#define GAIN_I 0.1
#define GAIN_J 0.1
#define GAIN_K 0.2
#define FOLLOW 1

#define CHASSIS_FOLLOW (0)
#define CHASSIS_SPIN   (1)

/* variables -----------------------------------------------------------------*/
Chassis_t Chassis = {0};
int16_t front_temp,right_temp;

float fmax_speed_spin = 0; /*< 最大旋转速度 */
float fmax_wheel_speed = 0;

uint8_t turn_round_state = 0;
uint8_t chassis_state = CHASSIS_FOLLOW;

int16_t s16_max_front_speed = 320; /* 默认最大前后移动速度 */
int16_t s16_max_right_speed = 320; /* 默认最大左右移动速度 */

float imu_angle = 0.0f; /*< 归一化imu角度 */
float motor_angle = 0.0f; /*< 归一化电机角度 */
float chassis_angle = 0.0f; /*< 归一化底盘倾角 */

/*速度功率控制（需实测）    34      39      46      54      61       70       80       93 */
static float spin_speed[8]  = {650.0f, 700.0f, 800.0f, 1000.0f, 1050.0f, 1100.0f, 1150.0f, 1200.0f};
static float wheel_speed[8] = {6700.0f, 6800.0f, 6900.0f, 7000.0f, 7100.0f, 7200.0f, 7300.0f, 7400.0f};

float chassis_pitch_angle = 0.0f;

uint8_t speed_up_state = 0;

//chassis_angle = (|imu_angle-4704|)/987 - (|motor_angle-3416|)/1269

/* function ------------------------------------------------------------------*/


/**
  * @brief  底盘PID初始化
  * @param  void
  * @retval void
  * @attention 
  */
void Chassis_Init(void)
{
    uint8_t i;
	
    for(i=0;i<4;i++)
    {
        pid_init_increment(&Chassis.M3508[i].PidSpeed,Chassis_Speed_Kp,
                           Chassis_Speed_Ki,Chassis_Speed_Kd,Chassis_Speed_Inc_Limit);
    }
    for(i=0;i<4;i++)
    {
        pid_init_increment(&Chassis.M3508[i].PidCurrent,Chassis_Current_Kp,
                           Chassis_Current_Ki,Chassis_Current_Kd,Chassis_Current_Inc_Limit);
    }
}


/**
  * @brief  获取底盘控制数据
  * @param  遥控器消息结构体
  * @retval void
  * @attention 
  */
#define FRONT      (0)
#define RIGHT      (1)
#define CLOCK_WISE (2)
#define chassis_ramp_speed  (4)

int16_t chassis_now_speed[3];
int16_t chassis_aim_speed[3];

void Chassis_GetMoveData(RemoteData_t RDMsg)
{
    float DirAngle = Get_fDirAngle();
    uint8_t i;
    switch (RDMsg.S1)
    {
        case SUP:
        case SDOWN:
            Remote_Control_GetMoveData(RDMsg);
        break;
            
        case SMID:
            Keyboard_Control_GetMoveData(RDMsg);
        break;
        
        default:
        break;
    }
    
    
    /* 目标速度斜坡加速 稳定功率 */
    chassis_aim_speed[FRONT] = Misc_RAMP_Int16(front_temp, chassis_now_speed[FRONT], chassis_ramp_speed);
    chassis_aim_speed[RIGHT] = Misc_RAMP_Int16(right_temp, chassis_now_speed[RIGHT], chassis_ramp_speed);
    
    for(i=0; i<3; i++)
    {
        chassis_now_speed[i] = chassis_aim_speed[i];
    }
    
    Omni_Speed_Parse(DirAngle, chassis_aim_speed[FRONT], chassis_aim_speed[RIGHT]);
}

/**
  * @brief  电机速度控制
  * @param  void
  * @retval void
  * @attention 
  */
static void Chassis_Speed_Control(void)
{
    uint8_t i;
    float limit_speed = fmax_wheel_speed;
    float tmp_max_speed = 1;
    float tmp_min_speed = 1;
    float tmp_speed[4];
    float cmp_index = 1;
    
    
    #if defined(INFANTRY_3) || defined(INFANTRY_5)
        tmp_speed[0] = (-Chassis.MoveData.Right / GAIN_I 
                  -Chassis.MoveData.Front / GAIN_J 
                  +Chassis.MoveData.ClockWise / GAIN_K);

        tmp_speed[1] = (-Chassis.MoveData.Right / GAIN_I 
                      +Chassis.MoveData.Front / GAIN_J 
                      +Chassis.MoveData.ClockWise / GAIN_K);
        
        tmp_speed[2] = (Chassis.MoveData.Right / GAIN_I 
                      -Chassis.MoveData.Front / GAIN_J 
                      +Chassis.MoveData.ClockWise / GAIN_K);
        
        tmp_speed[3] = (Chassis.MoveData.Right / GAIN_I 
                      +Chassis.MoveData.Front / GAIN_J  
                      +Chassis.MoveData.ClockWise / GAIN_K);
                      
        if (chassis_angle > 0.1f)
        {
            tmp_speed[0] *= (chassis_angle*4.0f+1.0f);
            tmp_speed[1] *= (chassis_angle*4.0f+1.0f);
            
            tmp_speed[2] *= (chassis_angle*2+1.0f);
            tmp_speed[3] *= (chassis_angle*2+1.0f);
        }

    #endif
    
    
    #if  defined(INFANTRY_4) || defined(INFANTRY_6)
        tmp_speed[0] = (Chassis.MoveData.Right / GAIN_I 
                  +Chassis.MoveData.Front / GAIN_J 
                  +Chassis.MoveData.ClockWise / GAIN_K);

        tmp_speed[1] = (Chassis.MoveData.Right / GAIN_I 
                      -Chassis.MoveData.Front / GAIN_J 
                      +Chassis.MoveData.ClockWise / GAIN_K);
        
        tmp_speed[2] = (-Chassis.MoveData.Right / GAIN_I 
                      +Chassis.MoveData.Front / GAIN_J 
                      +Chassis.MoveData.ClockWise / GAIN_K);
        
        tmp_speed[3] = (-Chassis.MoveData.Right / GAIN_I 
                      -Chassis.MoveData.Front / GAIN_J  
                      +Chassis.MoveData.ClockWise / GAIN_K);
                      
                      /* 坡度大于0.1 加速并差速 */
        if (chassis_angle > 0.1f )
        {
            tmp_speed[2] *= (chassis_angle*4.0f+1.0f);
            tmp_speed[3] *= (chassis_angle*4.0f+1.0f);
            
            tmp_speed[0] *= (chassis_angle*2+1.0f);
            tmp_speed[1] *= (chassis_angle*2+1.0f);
        }

    #endif
    
    
    

    for(i = 0; i<4; i++)
    {
        /* 找最大值 */
        if (tmp_speed[i] > tmp_max_speed)
        {
            tmp_max_speed = tmp_speed[i];
        }
        
        /* 找最小值 */
        if (tmp_speed[i] < tmp_min_speed)
        {
            tmp_min_speed = tmp_speed[i];
        }
    }

    tmp_max_speed = Misc_Fabsf(tmp_max_speed);
    tmp_min_speed = Misc_Fabsf(tmp_min_speed);

    if (tmp_max_speed > limit_speed || tmp_min_speed > limit_speed)
    {
        if (tmp_max_speed < tmp_min_speed)
        {
            cmp_index = limit_speed/tmp_min_speed;
        }
        else if (tmp_max_speed > tmp_min_speed)
        {
            cmp_index = limit_speed/tmp_max_speed;
        }
        else
        {
            cmp_index = limit_speed/tmp_min_speed;
        }
    }
    else
    {
        cmp_index = 1;
    }

    for(i = 0; i<4; i++)
    {
        Chassis.M3508[i].TarSpeed = (int16_t)(tmp_speed[i]*cmp_index);
    }
}

/**
  * @brief  底盘模式选择（r键选择？）
  * @param  控制指令结构体
  * @retval void
  * @attention 
  */
void Chassis_ChooseMode(RemoteData_t RDMsg)
{
    int16_t spin_s16temp;
    static uint8_t delay_tick;
    
    float DirAngle = Get_fDirAngle();
    
    
    switch(RDMsg.S1)
    {
        case SUP:
            switch(RDMsg.S2)
            {
                case SMID:
                    spin_s16temp = s16Follow_Speed_Spin(Chassis.MoveData.Right); /*< 跟随速度 */
                break;
                
                case SUP:
                    spin_s16temp = s16Follow_Holder_Spin(DirAngle); /*< 跟随云台 */
                break;
                
                case SDOWN:
                    spin_s16temp = s16Holder_Follow_GetMoveData(RDMsg); /*< 云台跟随底盘*/
                break;
                
                default:
                    spin_s16temp = s16Follow_Holder_Spin(DirAngle); /*< 跟随云台 */
                break;
            }
        break; /*< end case SUP */
            
        case SDOWN:
            Dynamic_Spin();
        break; /*< end case SDOWN */
        
        case SMID: /*< 键鼠模式 */
            /* R键切换跟随和小陀螺模式 */
            
            if (delay_tick < 50)
            {
                delay_tick++;
            }
            
            if (RDMsg.KeyBoard.r == 1 && chassis_state == CHASSIS_FOLLOW && delay_tick == 50)
            {
                chassis_state = CHASSIS_SPIN;
                delay_tick = 0;
            }
            else if (RDMsg.KeyBoard.r == 1 && chassis_state == CHASSIS_SPIN && delay_tick == 50)
            {
                chassis_state = CHASSIS_FOLLOW;
                delay_tick = 0;
            }
            
            switch(chassis_state)
            {
                case CHASSIS_FOLLOW:
                    if (RDMsg.KeyBoard.c == 1)
                    {
                        spin_s16temp = s16Follow_Holder_Spin(DirAngle);
                    }
                    else
                    {
                        spin_s16temp = s16Follow_Speed_Spin(Chassis.MoveData.Right);
                    }
                break;
                    
                case CHASSIS_SPIN:
                    if (RDMsg.KeyBoard.c == 1)
                    {
                        spin_s16temp = s16Follow_Holder_Spin(DirAngle);
                    }
                    else 
                    {
                        Dynamic_Spin();
                    }
                break;
                    
                default:

                break;
            } /*< end switch chassis state */
            
        break; /*< end case SMID */
            
        default:
        break;        
    } /*< end switch S1 */

    Chassis.MoveData.ClockWise = Constrain_Int32_t(spin_s16temp, -fmax_speed_spin, fmax_speed_spin);
}

/**
  * @brief  底盘PID输出
  * @param  void
  * @retval void
  * @attention （通过速度环（外环）所得值再计算电流环，通过电流坏（内环）直接控制输出）（rx,lpf???）
  */
void Chassis_PidRun(void)
{
    uint8_t i;

    #ifdef CHASSIS_WAVE
        UART2_SendWave(5, 2, &Chassis.M3508[0].TarSpeed, &Chassis.M3508[0].Rx.Speed,
                             &Chassis.M3508[0].TarCurrent, &Chassis.M3508[0].Rx.Current,
                             &Chassis.M3508[0].LPf.Output);
    #endif

    #ifdef CHASSIS_DEBUG
        Chassis_Init();
    #endif

    for (i = 0; i < 4; i++)     //rx speed lpf
    {
        Chassis.M3508[i].LPf.Speed = 0.8 * Chassis.M3508[i].Rx.Speed + 0.2 * Chassis.M3508[i].LPf.Speed;
    }   
    for (i = 0; i < 4; i++)     //speed loop
    {
        Chassis.M3508[i].TarCurrent = pid_increment_update(Chassis.M3508[i].TarSpeed, Chassis.M3508[i].LPf.Speed, &Chassis.M3508[i].PidSpeed);
    }
    for (i = 0; i < 4; i++)     //tar current lpf 
    {
        Chassis.M3508[i].LPf.TarCurrent = 0.8 * Chassis.M3508[i].TarCurrent + 0.2 * Chassis.M3508[i].LPf.TarCurrent;
    }
    
    for (i = 0; i < 4; i++)     //rx current lpf 
    {
        Chassis.M3508[i].LPf.Current = 0.8 * Chassis.M3508[i].Rx.Current + 0.2 * Chassis.M3508[i].LPf.Current;
    }
    
    for (i = 0; i < 4; i++)     //current loop
    {
        Chassis.M3508[i].Output = pid_increment_update(Chassis.M3508[i].LPf.TarCurrent, Chassis.M3508[i].LPf.Current, &Chassis.M3508[i].PidCurrent);
    }
    
    for (i = 0; i < 4; i++)     //out lpf
    {
        Chassis.M3508[i].LPf.Output = 0.8 * Chassis.M3508[i].Output + 0.2 * Chassis.M3508[i].LPf.Output;
    }
}


/**
  * @brief  底盘电机CAN
  * @param  void
  * @retval void
  * @attention 放中断里面
  */
void Chassis_CanTransmit(void)
{
    uint8_t i;
    if(Observer.Tx.DR16_Rate>15) /*< 遥控器保护，数据量16时才开启控制 */
    {
        for(i=0;i<4;i++)
        {
            /* 输出限幅 */
            Chassis.M3508[i].LPf.Output = constrain_int16_t(Chassis.M3508[i].LPf.Output,
                                                    -Chassis_Out_Limit, Chassis_Out_Limit);
            
            /* CAN 赋值 */
            Chassis.CanData[2*i]=(uint8_t)(Chassis.M3508[i].LPf.Output>>8);
            Chassis.CanData[2*i+1]=(uint8_t)(Chassis.M3508[i].LPf.Output);
        }
    }
    else
    {
        Chassis_Speed_Reset(); /*< 关闭遥控器后，底盘目标速度一直保持当前状态（0） */
    }
    CAN2_Transmit(0x200,Chassis.CanData);
}


/**
  * @brief  超级功率CAN发送
  * @param  void
  * @retval void
  * @attention 
  */
void SuperCap_CanTransmit(void)
{
    Chassis.SuperCap.CanData[0] = Chassis.SuperCap.TargetPower >> 8;
    Chassis.SuperCap.CanData[1] = Chassis.SuperCap.TargetPower;
    CAN2_Transmit(0x210,Chassis.SuperCap.CanData);
}


/**
  * @brief  功率控制，向超级电容发送最大功率
  * @param  void
  * @retval void
  * @attention 超级电容根据最大功率控制电流输出，防止底盘超功率
  */
static void Chassis_Power_limit_F(void)
{
    static uint8_t tick = 0;
    int16_t max_move_speed;
    int16_t max_spin_speed;

    uint16_t power_lim = JUDGE_u16GetChassisPowerLimit();
    uint16_t power_buf = JUDGE_u16GetRemainEnergy();
    float speed_bias = 10*(float)(Chassis.SuperCap.CapVol)/(float)JUDGE_u16GetChassisVolt();

    tick++;/*< 延时*/
    if (tick == 20)
    {
        if (power_lim < 40)
        {
            power_lim = 40;
        }
        
        Chassis.SuperCap.TargetPower = power_lim*100;
        Chassis.SuperCap.TargetPower = Constrain_Uint16_t(Chassis.SuperCap.TargetPower, 3000, 15000);
        
        if(power_buf < 15)
        {
            Chassis.SuperCap.TargetPower = (power_lim-5)*100;
        }
        SuperCap_CanTransmit();
        tick = 0;
    }

    
    switch(power_lim)
    {
        case 40:
            fmax_wheel_speed = wheel_speed[0];
            fmax_speed_spin  = spin_speed[0];
            max_spin_speed = spin_speed[0];
            max_move_speed = 320;
        break;
        
        case 45:
            fmax_wheel_speed = wheel_speed[1];
            fmax_speed_spin  = spin_speed[1];
            max_spin_speed = spin_speed[1];
            max_move_speed = 330;
        break;
        
        case 50:
        case 55:
            fmax_wheel_speed = wheel_speed[2];
            fmax_speed_spin  = spin_speed[2];
            max_spin_speed = spin_speed[2];
            max_move_speed = 350;
        break;
        
        case 60:
            fmax_wheel_speed = wheel_speed[3];
            fmax_speed_spin  = spin_speed[3];
            max_spin_speed = spin_speed[3];
            max_move_speed = 380;
        break;
        
        case 70:
            fmax_wheel_speed = wheel_speed[4];
            fmax_speed_spin  = spin_speed[4];
            max_spin_speed = spin_speed[4];
            max_move_speed = 410;
        break;
        
        case 80:
            fmax_wheel_speed = wheel_speed[5];
            fmax_speed_spin  = spin_speed[5];
            max_spin_speed = spin_speed[5];
            max_move_speed = 500;
        break;
        
        case 100:
            fmax_wheel_speed = wheel_speed[6];
            fmax_speed_spin  = spin_speed[6];
            max_spin_speed = spin_speed[6];
            max_move_speed = 550;
        break;
        
        case 120:
            fmax_wheel_speed = wheel_speed[7];
            fmax_speed_spin  = spin_speed[7];
            max_spin_speed = spin_speed[7];
            max_move_speed = 650;
        break;
        
        default:
            fmax_wheel_speed = wheel_speed[0];
            fmax_speed_spin  = spin_speed[0];
            max_spin_speed = spin_speed[0];
            max_move_speed = 360;
        break;
    }
    
    s16_max_front_speed = max_move_speed*speed_bias;
    s16_max_right_speed = max_move_speed*speed_bias;
    fmax_speed_spin = max_spin_speed*speed_bias;
}


/**
  * @brief  底盘进程
  * @param  控制指令结构体
  * @retval void
* @attention 保护写在CAN发送前，（V）一键复位，PID误差和输出清零
  */
void Chassis_Process(RemoteData_t RDMsg)
{
    Chassis_Fly_Speed_Control();
    Chassis_Power_limit_F();
    Chassis_GetMoveData(RDMsg);
	Chassis_ChooseMode(RDMsg);
    Chassis_Protect(RDMsg);
    Turn_Round(RDMsg);    /*< 一键掉头*/
    Fast_Fix_Spin(RDMsg); /*< （E）键快速定点小陀螺 */
    Chassis_Speed_Control();
    Chassis_PidRun();
}

/*********************************底盘辅助函数*********************************/


/**
  * @brief  底盘速度初始化，PID输出清零
  * @param  void
  * @retval void
  * @attention 
  */
void Chassis_Speed_Reset(void)
{
    uint8_t i;
    
    Chassis.MoveData.Front = 0;
    Chassis.MoveData.ClockWise = 0;
    Chassis.MoveData.Right = 0;
    for(i = 0; i < 4; i++)
    {
        Chassis.M3508[i].TarSpeed            = 0;
        Chassis.M3508[i].PidSpeed.errNow     = 0;
        Chassis.M3508[i].PidSpeed.errOld1    = 0;
        Chassis.M3508[i].PidSpeed.errOld2    = 0;
        Chassis.M3508[i].PidSpeed.ctrOut     = 0;
        Chassis.M3508[i].PidSpeed.dCtrOut    = 0;
        Chassis.M3508[i].PidCurrent.errNow   = 0;
        Chassis.M3508[i].PidCurrent.errOld1  = 0;
        Chassis.M3508[i].PidCurrent.errOld2  = 0;
        Chassis.M3508[i].PidCurrent.dCtrOut  = 0;
        Chassis.M3508[i].PidCurrent.ctrOut   = 0;
    }
    memset(Chassis.CanData,0,sizeof(Chassis.CanData));
}


/**
  * @brief  底盘速度初始化，PID输出清零
  * @param  控制指令结构体
  * @retval void
  * @attention 
  */
void Chassis_Protect(RemoteData_t RDMsg)
{
    if (RDMsg.KeyBoard.v == 1)
    {
        Chassis_Speed_Reset();
    }
}


/**
  * @brief  快速定点小陀螺
  * @param  控制指令结构体
  * @retval void
  * @attention 
  */
void Fast_Fix_Spin(RemoteData_t RDMsg)
{
    if (RDMsg.KeyBoard.e == 1)
    {
        Chassis.MoveData.Front = 0;
        Chassis.MoveData.Right = 0;
        Chassis.MoveData.ClockWise = fmax_speed_spin*1.6f;
    }
}


/**
  * @brief  动态小陀螺(实测：限轮子速度就可以了，没必要动态改旋转速度)
  * @param  控制指令结构体
  * @retval void
  * @attention （front 或 right速度 与 clockwise速度 成反比关系：没用）
  */
void Dynamic_Spin(void)
{
    Chassis.MoveData.ClockWise = fmax_speed_spin;
    Chassis.MoveData.ClockWise = constrain_int16_t(Chassis.MoveData.ClockWise, -1310, 1310);
    JudgeUI_spin_state='l';
}


/**
  * @brief  (Q键)一键掉头，云台偏航+180度，底盘与云台差角+180度
  * @param  控制指令结构体
  * @retval void
  * @attention 
  */
void Turn_Round(RemoteData_t RDMsg)
{
    static uint8_t delay_tick = 0;
    static uint8_t state = 0;
   
    
    
    if (delay_tick < 100)
    {
        delay_tick++; /*< 延时，防止连续按两次*/
        
    }
    
    if (RDMsg.KeyBoard.q == 1 && delay_tick == 100 && state == 0)
    {
        Holder.Yaw.TarAngle +=TURN_ROUND_ANGLE;
        delay_tick = 0;
        state = 1;
    }
    else if (RDMsg.KeyBoard.q == 1 && delay_tick == 100 && state == 1)
    {
        Holder.Yaw.TarAngle +=TURN_ROUND_ANGLE;
        delay_tick = 0;
        state = 0;
    }
}


/**
  * @brief  底盘跟随云台旋转
  * @param  底盘和云台差角
  * @retval 旋转速度
  * @attention 
  */
int16_t s16Follow_Holder_Spin(float dir_angle)
{
    int8_t s8_sign;
    
    s8_sign = dir_angle>0? -1:1;
    
    JudgeUI_spin_state='f';
    
    return s8_sign*dir_angle*dir_angle;
}

/**
  * @brief  底盘跟随速度方向旋转
  * @param  横移速度
  * @retval 旋转速度
  * @attention 
  */
int16_t s16Follow_Speed_Spin(float right_speed)
{   
    JudgeUI_spin_state='s';
    
    return 2.5f*right_speed;
}

/**
  * @brief  全向底盘速度解算
  * @param  云台和底盘差角
  * @retval 
  * @attention 
  */
void Omni_Speed_Parse(float dir_angle, int16_t s16_speed_front, int16_t s16_speed_right)
{
    Chassis.MoveData.Front = (int16_t)(((float)s16_speed_front) * (fast_cos((int16_t)dir_angle))) 
                            +(int16_t)(((float)s16_speed_right) * (fast_sin((int16_t)dir_angle)));
    
    Chassis.MoveData.Right = -(int16_t)(((float)s16_speed_front) * (fast_sin((int16_t)dir_angle))) 
                             +(int16_t)(((float)s16_speed_right) * (fast_cos((int16_t)dir_angle)));
}

/**
  * @brief  键盘控制速度解算
  * @param  遥控器消息结构体
  * @retval 
  * @attention 
  */
void Keyboard_Control_GetMoveData(RemoteData_t RDMsg)
{
    if(RDMsg.KeyBoard.w)
        front_temp = s16_max_front_speed;
    else if((!RDMsg.KeyBoard.w)&&(RDMsg.KeyBoard.s))
        front_temp =-s16_max_front_speed;
    else
        front_temp = 0;
    
    if(RDMsg.KeyBoard.d)
        right_temp = s16_max_right_speed;
    else if((!RDMsg.KeyBoard.d)&&(RDMsg.KeyBoard.a))
        right_temp =-s16_max_right_speed;
    else
        right_temp = 0;
    
    if(RDMsg.KeyBoard.ctrl)
    {
        speed_up_state = 1;
        front_temp = front_temp*2;
        right_temp = right_temp*2;
    }else if((!RDMsg.KeyBoard.ctrl)&&(RDMsg.KeyBoard.shift))
    {
        front_temp = front_temp*0.7;
        right_temp = right_temp*0.7;
    }
}

/**
  * @brief  遥控器控制速度解算
  * @param  遥控器消息结构体
  * @retval 
  * @attention 
  */
void Remote_Control_GetMoveData(RemoteData_t RDMsg)
{
    right_temp = RDMsg.Ch2;
    front_temp = RDMsg.Ch3;
}

/**
  * @brief  云台跟随底盘
  * @param  遥控器消息结构体
  * @retval 
  * @attention 不进行无头模式解算
  */
int16_t s16Holder_Follow_GetMoveData(RemoteData_t RDMsg)
{
    Chassis.MoveData.Right = -RDMsg.Ch2;
    Chassis.MoveData.Front = -RDMsg.Ch3;
    return RDMsg.Ch0; /*< 返回旋转速度*/
}

/**
  * @brief  飞坡速度控制
  * @param  
  * @retval 
  * @attention
  */
void Chassis_Fly_Speed_Control(void)
{
    imu_angle = Misc_Fabsf(Holder.Pitch.ImuAngle - DOWN_PITCH_ANGLE_IMU)/Misc_Fabsf(UP_PITCH_ANGLE_IMU -DOWN_PITCH_ANGLE_IMU);
    motor_angle = Misc_Fabsf(Holder.Pitch.Angle - DOWN_PITCH_ANGLE_MOTOR)/Misc_Fabsf(UP_PITCH_ANGLE_MOTOR -DOWN_PITCH_ANGLE_MOTOR);
    chassis_angle = Misc_Fabsf(imu_angle - motor_angle);
}

/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
