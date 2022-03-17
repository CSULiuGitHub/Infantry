/**
  ******************************************************************************
  * @file    holder.c
  * @author  sy xl qj
  * @brief   云台任务
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
  * 云台注意事项：
  * 1.先开环测试是否存在机械问题
  * 2.计算函数尽量自定义，不要用math.h库
  ******************************************************************************
  */

/*includes ------------------------------------------------------------------*/
#include "holder_config.h"
#include <string.h>
#include "misc_func.h"
#include "vision.h"
#include "holder.h"
#include "motor.h"
#include "usart.h"
#include "judge.h"
#include "can.h"
#include "tim.h"
#include "fast_tri_func.h"
#include "kfilter.h"
#include "ADRC.h"

/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
#define VISION (1) /*< 视觉模式 */
#define MANUAL (0) /*< 手动模式 */

/* variables -----------------------------------------------------------------*/
Holder_t Holder;
static uint8_t holder_ctrl_state; /*< 云台控制模式 手动或视觉*/
int32_t x = 0; /*< 鼠标移动速度，注意是速度，积分后才是位移*/
int32_t y = 0;

int32_t vision_y_bias = 4000;
int32_t vision_x_bias = 0;
static int32_t y_bias = 0;
/* function ------------------------------------------------------------------*/

/**
  * @brief  云台PID初始化
  * @param  void
  * @retval void
  * @attention
  */
static void Holder_Adrc_Init(void)
{
    float ADRC_Unit_Angle[16]=
/*TD跟踪微分器   改进最速TD,h0=N*h      扩张状态观测器ESO           扰动补偿     非线性组合*/
/*  r     h      N                  beta_01   beta_02    beta_03     b0     beta_0  beta_1     beta_2     N1     C    alpha1  alpha2  zeta  b*/
 {300000 ,0.005 , 3,               300,      4000,      10000,     0.001,    0.002,   0.9,      0.00040,    5,    5,    0.85,   1.5,    50,    0};

 float ADRC_Unit_Speed[16]=
/*TD跟踪微分器   改进最速TD,h0=N*h      扩张状态观测器ESO           扰动补偿     非线性组合*/
/*  r     h      N                  beta_01   beta_02    beta_03     b0     beta_0  beta_1     beta_2     N1     C    alpha1  alpha2  zeta  b*/
 {300000 ,0.005 , 3,               300,      4000,      10000,     0.001,    0.002,   90.0,      0.040,    5,    5,    0.85,   1.5,    50,    0};

 
 ADRC_Init(&ADRC_Pitch_Controller, ADRC_Unit_Angle);
 ADRC_Init(&ADRC_Pitch_Speed_Controller, ADRC_Unit_Speed);
}

static void Holder_Adrc_Run(void)
{
//    int32_t err = Holder.Pitch.TarAngle - Holder.Pitch.Angle;
//    ADRC_Pitch_Controller.beta_2 = 0.002*(1 - 1/(1+0.001f*(err+err*err)));
    
    ADRC_Control(&ADRC_Pitch_Controller, Holder.Pitch.TarAngle, Holder.Pitch.Angle);
    Holder.Pitch.TarSpeed = ADRC_Pitch_Controller.u;
    
    ADRC_Control(&ADRC_Pitch_Speed_Controller, Holder.Pitch.TarSpeed, Holder.Pitch.Speed);
    Holder.Pitch.Output = ADRC_Pitch_Speed_Controller.u;
}


/**
  * @brief  云台PID初始化
  * @param  void
  * @retval void
  * @attention
  */
static void Holder_Pid_Init(void)
{
    /* Yaw 陀螺仪反馈*/
    pid_init_absolute(&Holder.Yaw.PidAngle, Yaw_Angle_Min_Kp, Yaw_Angle_Min_Ki, Yaw_Angle_Kd,
                      Yaw_Angle_Ki_Limit, Yaw_Angle_Kp_Limit);
    pid_init_absolute(&Holder.Yaw.PidSpeed, Yaw_Speed_kp, Yaw_Speed_ki, Yaw_Speed_kd,
                      Yaw_Speed_Ki_Limit, Yaw_Speed_Kp_Limit);
    
    /* Pitch 电机反馈*/
    pid_init_absolute(&Holder.Pitch.PidAngle, Pitch_Angle_Min_Kp, Pitch_Angle_Min_Ki, Pitch_Angle_Kd,
                      Pitch_Angle_Ki_Limit, Pitch_Angle_Kp_Limit);
    pid_init_absolute(&Holder.Pitch.PidSpeed, Pitch_Speed_kp, Pitch_Speed_ki, Pitch_Speed_kd,
                      Pitch_Speed_Ki_Limit, Pitch_Speed_Kp_Limit);
}



static void Holder_Vision_Pid_Init(void)
{
    /* Yaw 陀螺仪反馈*/
    pid_init_absolute(&Holder.Yaw.PidVisionAngle, Yaw_Angle_Vision_Min_Kp, Yaw_Angle_Vision_Min_Ki, Yaw_Angle_Vision_Kd,
                      Yaw_Angle_Vision_Ki_Limit, Yaw_Angle_Vision_Kp_Limit);
    pid_init_absolute(&Holder.Yaw.PidSpeed, Yaw_Speed_Vision_kp, Yaw_Speed_Vision_ki, Yaw_Speed_Vision_kd,
                      Yaw_Speed_Ki_Limit, Yaw_Speed_Kp_Limit);
    
    /* Pitch 电机反馈*/
    pid_init_absolute(&Holder.Pitch.PidVisionAngle, Pitch_Angle_Vision_Min_Kp, Pitch_Angle_Vision_Min_Ki, Pitch_Angle_Vision_Kd,
                      Pitch_Angle_Vision_Ki_Limit, Pitch_Angle_Vision_Kp_Limit);
    pid_init_absolute(&Holder.Pitch.PidSpeed, Pitch_Speed_Vision_kp, Pitch_Speed_Vision_ki, Pitch_Speed_Vision_kd,
                      Pitch_Speed_Ki_Limit, Pitch_Speed_Kp_Limit);
}

/**
  * @brief  云台PID控制，手动控制
  * @param  void
  * @retval void
  * @attention 电机失控：输出低通滤波调低，效果明显
  */
static void Holder_Pid_Manual(void)
{
    static uint8_t init_state = 0;

    if (init_state == 0) /*< 未初始化 */
    {
        Holder_Pid_Init();
        init_state = 1;
    }
    else if (init_state == 1)/*< 已初始化 */
    {
        Holder_Pid_Init();
        /* 变结构PI */
        Holder_Pi_Tunning_Absolute(Yaw_Angle_Tunning_Kp, Yaw_Angle_Min_Kp, 
                                   Yaw_Angle_Tunning_Ki, Yaw_Angle_Min_Ki,
                                   Yaw_Angle_Kd, &Holder.Yaw.PidAngle);
        
        Holder.Yaw.TarSpeedLpf = pid_absolute_update(Holder.Yaw.TarAngle,Holder.Yaw.Angle,&Holder.Yaw.PidAngle);
        Holder.Yaw.TarSpeed = Yaw_Angle_Lpf_Bias * Holder.Yaw.TarSpeedLpf + (1-Yaw_Angle_Lpf_Bias) * Holder.Yaw.TarSpeed;
        Holder.Yaw.OutputLpf = pid_absolute_update(Holder.Yaw.TarSpeed,Holder.Yaw.Speed,&Holder.Yaw.PidSpeed);
        Holder.Yaw.Output = Yaw_Speed_Lpf_Bias * Holder.Yaw.OutputLpf + (1-Yaw_Speed_Lpf_Bias) * Holder.Yaw.Output;
        Holder.Yaw.Output = Constrain_Int32_t(Holder.Yaw.Output, -Yaw_Output_Limit, Yaw_Output_Limit);

//        /* 变结构PI */
//        Holder_Pi_Tunning_Absolute(Pitch_Angle_Tunning_Kp, Pitch_Angle_Min_Kp,
//                                   Pitch_Angle_Tunning_Ki, Pitch_Angle_Min_Ki,
//                                   Pitch_Angle_Kd, &Holder.Pitch.PidAngle);
//        
//        Holder.Pitch.TarSpeedLpf = pid_absolute_update(Holder.Pitch.TarAngle,Holder.Pitch.Angle,&Holder.Pitch.PidAngle);
//        Holder.Pitch.TarSpeed = Pitch_Angle_Lpf_Bias * Holder.Pitch.TarSpeedLpf + (1-Pitch_Angle_Lpf_Bias) * Holder.Pitch.TarSpeed;
//        Holder.Pitch.OutputLpf = pid_absolute_update(Holder.Pitch.TarSpeed,Holder.Pitch.Speed,&Holder.Pitch.PidSpeed);
//        Holder.Pitch.Output = Pitch_Speed_Lpf_Bias * Holder.Pitch.OutputLpf + (1-Pitch_Speed_Lpf_Bias) * Holder.Pitch.Output;
//        Holder.Pitch.Output = Constrain_Int32_t(Holder.Pitch.Output, -Pitch_Output_Limit, Pitch_Output_Limit);
    }
}

/**
  * @brief  云台PID控制，视觉控制
  * @param  void
  * @retval void
  * @attention 变结构P减少超调，效果明显
  */

//射速15： 350 205   100 150  450 350  800 620   950 750 曲线拟合：http://www.qinms.com/webapp/curvefit/cf.aspx
static void Holder_Pid_Vision(void)
{
    static uint8_t init_state = 0;
    
    
//    vision_y_bias = (-321.3f + 3.921f*vData_z -0.00589*vData_z*vData_z + 0.00000393f*vData_z*vData_z*vData_z)*10;

//    if (vData_z < 60)
//    {
//        vision_bias = 0;
//    }
    if (init_state == 0) /*< 未初始化 */
    {
        Holder_Vision_Pid_Init();
        init_state = 1;
    }
    else if (init_state == 1)/*< 已初始化 */
    {
        Holder_Vision_Pid_Init();
        y_bias += -y;
        vision_x_bias = Constrain_Int32_t(vision_x_bias, -700*20, 700*20);
        vision_y_bias = Constrain_Int32_t(vision_y_bias, -900*10, 900*10);
        Holder.Yaw.TarAngle = Holder.Yaw.Angle + vData_x + vision_x_bias;
        Holder.Pitch.TarAngle = Holder.Pitch.Angle + vData_y + vision_y_bias + y_bias; 
        
        Holder_Pi_Tunning_Absolute(Yaw_Angle_Vision_Tunning_Kp, Yaw_Angle_Vision_Min_Kp, 
                                   Yaw_Angle_Vision_Tunning_Ki, Yaw_Angle_Vision_Min_Ki,
                                   Yaw_Angle_Vision_Kd, &Holder.Yaw.PidVisionAngle);
        
        Holder.Yaw.TarSpeedLpf = pid_absolute_update(Holder.Yaw.TarAngle,Holder.Yaw.Angle,&Holder.Yaw.PidVisionAngle);
        Holder.Yaw.TarSpeed = Yaw_Angle_Lpf_Bias * Holder.Yaw.TarSpeedLpf + (1-Yaw_Angle_Lpf_Bias) * Holder.Yaw.TarSpeed;
        Holder.Yaw.OutputLpf = pid_absolute_update(Holder.Yaw.TarSpeed,Holder.Yaw.Speed,&Holder.Yaw.PidSpeed);
        Holder.Yaw.Output = Yaw_Speed_Lpf_Bias * Holder.Yaw.OutputLpf + (1-Yaw_Speed_Lpf_Bias) * Holder.Yaw.Output;
        Holder.Yaw.Output = Constrain_Int32_t(Holder.Yaw.Output, -Yaw_Output_Limit, Yaw_Output_Limit);
//        
//        Holder_Pi_Tunning_Absolute(Pitch_Angle_Vision_Tunning_Kp, Pitch_Angle_Vision_Min_Kp,
//                                   Pitch_Angle_Vision_Tunning_Ki, Pitch_Angle_Vision_Min_Ki,
//                                   Pitch_Angle_Vision_Kd, &Holder.Pitch.PidVisionAngle);
//        
//        Holder.Pitch.TarSpeedLpf = pid_absolute_update(Holder.Pitch.TarAngle,Holder.Pitch.Angle,&Holder.Pitch.PidVisionAngle);
//        Holder.Pitch.TarSpeed = Pitch_Angle_Lpf_Bias * Holder.Pitch.TarSpeedLpf + (1-Pitch_Angle_Lpf_Bias) * Holder.Pitch.TarSpeed;
//        Holder.Pitch.OutputLpf = pid_absolute_update(Holder.Pitch.TarSpeed,Holder.Pitch.Speed,&Holder.Pitch.PidSpeed);
//        Holder.Pitch.Output = Pitch_Speed_Lpf_Bias * Holder.Pitch.OutputLpf + (1-Pitch_Speed_Lpf_Bias) * Holder.Pitch.Output;
//        Holder.Pitch.Output = Constrain_Int32_t(Holder.Pitch.Output, -Pitch_Output_Limit, Pitch_Output_Limit);
    }
}


/**
  * @brief  云台PID控制
  * @param  void
  * @retval void
  * @attention 多种控制器混控，效果不是特别好
  *            vision数据是目标与图像中心的偏差值
  */
static void Holder_PidRun(void)
{
	/* 自瞄 */
    #ifdef VISION_NO_CP
    holder_ctrl_state = VISION;
    #endif
    
    Holder_Vision_Prediction();
    
	if (holder_ctrl_state == VISION)
	{
        Holder_Pid_Vision();
	}
	/* 手动 */
	else if (holder_ctrl_state == MANUAL)
	{
        Holder_Pid_Manual();
	}
}


/**
  * @brief  获取云台控制数据
  * @param  控制消息结构体
  * @retval void
  * @attention 暂时只有键盘控制，后期加上
  */
static void Holder_GetMoveData(RemoteData_t RDMsg)
{
//    static uint8_t vision_init = 0;
    /* 遥控器控制 */
    if (RDMsg.S1 == 1 || RDMsg.S1 == 2)
    {
        /* 遥控器控制 */
        Holder.Pitch.TarAngle += rc_pitch_bias * RDMsg.Ch1;
        
        switch(RDMsg.S2)
        {
            case SUP:
            case SMID:
                Holder.Yaw.TarAngle   += rc_yaw_bias * RDMsg.Ch0;
            break;
            
            case SDOWN:
                Holder.Yaw.TarAngle -= Get_fDirAngle(); /*< 云台跟随底盘*/
            break;
            
            default:
                Holder.Yaw.TarAngle   -= rc_yaw_bias * RDMsg.Ch0;
            break;   
        }
    }
    else
    {
        static uint16_t delay_tick = 0;
        /* 按住鼠标右键开启视觉自瞄 */
        if ((RDMsg.MouseClick_right == 1) && (vData_z != -100)) /*<  && vData.Pos.z != -1 */
        {
            delay_tick = 0;
            holder_ctrl_state = VISION; /*< 开启自瞄状态*/
            y= RDMsg.Mouse_y;
            x= RDMsg.Mouse_x;
        }
        else
        {   
            
            delay_tick++; /*< 延时0.5s清除预测偏移 */
            
            /* 丢失视觉后 云台保持 */
            if (delay_tick < 2)
            {
                Holder.Pitch.TarAngle =  Holder.Pitch.Angle;
                Holder.Yaw.TarAngle = Holder.Yaw.Angle;
            }
            
            /* 延时1s后清除预测 */
            if (delay_tick >= 1000)
            {
                vision_x_bias = 0;
                vision_y_bias = 0;
                y_bias = 0;
                delay_tick = 1000;
            }
            
            holder_ctrl_state = MANUAL; /*< 开启手动状态*/
            
            x= RDMsg.Mouse_x;
            y= RDMsg.Mouse_y;
            y = Constrain_Int32_t(y, -key_yaw_limit, key_yaw_limit);
            Holder.Yaw.TarAngle -= key_yaw_bias*x;
            Holder.Pitch.TarAngle -=  key_pitch_bias*y;
        }
    }
	
	/* 角度保护 */
    Holder.Pitch.TarAngle = Constrain_Int32_t(Holder.Pitch.TarAngle,MIN_PITCH_ANGLE_MOTOR,MAX_PITCH_ANGLE_MOTOR);
}


/**
  * @brief  获取云台状态数据
  * @param  云台状态结构体
  * @retval void
  * @attention 俯仰Pitch角度范围 (-540, 540)
  */
static void Holder_MsgIn(HolderData_t HDMsg)
{   
    /* 陀螺仪反馈 */
    Holder.Yaw.Angle = HDMsg.Angle[2]*200.0f; 
    Holder.Yaw.Speed = HDMsg.Gyro[2];         

    Holder.Pitch.Angle = Holder.Pitch._0x209.Rx.Angle*10;
    #if defined(INFANTRY_6)
        if (Holder.Pitch.Angle > 20000)
        {
            Holder.Pitch.Angle = Holder.Pitch.Angle - 81920;
        }
    
    #endif
    
    #if defined(INFANTRY_5)
    if (Holder.Pitch.Angle < 10000)
    {
        Holder.Pitch.Angle = 81920 + Holder.Pitch.Angle;
    }
    #endif

    Holder.Pitch.Speed = Holder.Pitch._0x209.Rx.Speed;
    
    Holder.Pitch.ImuAngle = HDMsg.Angle[0]*100.0f;
}

/**
  * @brief  云台参数初始化
  * @param  云台状态结构体
  * @retval void
  * @attention
  */
void Holder_Init(HolderData_t HDMsg)
{
    Holder_MsgIn(HDMsg);
	Holder_Angle_Reset();
}

/**
  * @brief  云台电机can控制
  * @param  void
  * @retval void
  * @attention
  */
void Holder_CanTransmit(void)
{   
    if(Observer.Tx.DR16_Rate>15) /*< 遥控器保护，数据量16时才开启控制 */
    {
        Holder.CanData[0] = Holder.Pitch.Output>>8;
        Holder.CanData[1] = Holder.Pitch.Output;        
        Holder.CanData[2] = Holder.Yaw.Output>>8;
        Holder.CanData[3] = Holder.Yaw.Output;
    }
    else
    {
        Holder_Angle_Reset(); /*< 关闭遥控器后，云台目标角度一直保持当前状态 */
    }
    CAN1_Transmit(0x2FF,Holder.CanData);
}

/**
  * @brief  云台进程
  * @param  云台状态 云台控制
  * @retval void
  * @attention 目前视觉识别颜色需要改程序，后期改进
  */
void Holder_Process(HolderData_t HDMsg,RemoteData_t RDMsg)
{
    Holder_Adrc_Init();
    
    Holder_Debug();
	Holder_MsgIn(HDMsg);
    Holder_GetMoveData(RDMsg);
    Holder_Angle_Protect(RDMsg); 
    Holder_Adrc_Run();
    Holder_PidRun();
}

/************************************云台辅助函数*********************************/
/**
  * @brief  云台PID输出清零，角度保持（重置为当前角度）
  * @param  void
  * @retval void
  * @attention
  */
void Holder_Angle_Reset(void)
{
    Holder.Yaw.TarAngle   = Holder.Yaw.Angle;
	Holder.Pitch.TarAngle = Holder.Pitch.Angle;
    Holder.Pitch.PidAngle.ctrOut = 0;
    Holder.Pitch.PidAngle.errD   = 0;
    Holder.Pitch.PidAngle.errI   = 0;
    Holder.Pitch.PidSpeed.ctrOut = 0;
    Holder.Pitch.PidAngle.errD   = 0;
    Holder.Pitch.PidAngle.errI   = 0;
    Holder.Pitch.PidSpeed.errI   = 0;
    memset(Holder.CanData,0,sizeof(Holder.CanData)); /*< 清空CAN */
}


/**
  * @brief  云台角度保护，防止疯转
  * @param  void
  * @retval void
  * @attention
  */
void Holder_Angle_Protect(RemoteData_t RDMsg)
{ 
    /*（V）一键手动复位，底盘也复位（误差和输出清零） */
    if (RDMsg.KeyBoard.v == 1)
    {
        Holder_Angle_Reset();
    }
}

/**
  * @brief  自瞄预测值计算
  * @param  void
  * @retval void
  * @attention 云台自身绝对速度 - 目标相对云台速度 = 目标绝对速度（相对地面速度）
  */
int32_t vision_holder_speed = 0;
int32_t vision_armor_speed_to_holder = 0;
int32_t vision_armor_speed = 0;
int32_t vData_old = 0;
int32_t vData_now = 0;
void Holder_Vision_Prediction(void)
{
    vData_now = vData_x;
    vision_holder_speed = Holder.Yaw.Speed;
    vision_armor_speed_to_holder = vData_now - vData_old;
    vData_old = vData_now;
    
    vision_armor_speed = vision_holder_speed - vision_armor_speed_to_holder;
    
    vision_x_bias = vision_armor_speed;
}


void Holder_Debug()
{
    #ifdef HOLDER_DEBUG
        Holder_Pid_Init();
    #endif
    
    #ifdef HOLDER_PITCH_WAVE
        UART2_SendWave(5, 4, &Holder.Pitch.TarAngle, &Holder.Pitch.Angle, 
                            &Holder.Pitch.TarSpeed, &Holder.Pitch.Speed, &Holder.Pitch.Output);
    
//    int32_t y = ADRC_Pitch_Speed_Controller.x1;
//    UART2_SendWave(2, 4, &Holder.Pitch.TarSpeed, &y);
    #endif
    
    #ifdef HOLDER_YAW_WAVE
        UART2_SendWave(5, 4, &Holder.Yaw.TarAngle, &Holder.Yaw.Angle, 
                            &Holder.Yaw.TarSpeed, &Holder.Yaw.Speed, &Holder.Yaw.Output);
    #endif
    
    #ifdef HOLDER_VISION_WAVE
        UART2_SendWave(3, 2, &vData_x, &vData_y, &vData_z);
    #endif
}

/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
