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

float fmax_speed_spin = 0; /*< �����ת�ٶ� */
float fmax_wheel_speed = 0;

uint8_t turn_round_state = 0;
uint8_t chassis_state = CHASSIS_FOLLOW;

int16_t s16_max_front_speed = 320; /* Ĭ�����ǰ���ƶ��ٶ� */
int16_t s16_max_right_speed = 320; /* Ĭ����������ƶ��ٶ� */

float imu_angle = 0.0f; /*< ��һ��imu�Ƕ� */
float motor_angle = 0.0f; /*< ��һ������Ƕ� */
float chassis_angle = 0.0f; /*< ��һ��������� */

/*�ٶȹ��ʿ��ƣ���ʵ�⣩    34      39      46      54      61       70       80       93 */
static float spin_speed[8]  = {650.0f, 700.0f, 800.0f, 1000.0f, 1050.0f, 1100.0f, 1150.0f, 1200.0f};
static float wheel_speed[8] = {6700.0f, 6800.0f, 6900.0f, 7000.0f, 7100.0f, 7200.0f, 7300.0f, 7400.0f};

float chassis_pitch_angle = 0.0f;

uint8_t speed_up_state = 0;

//chassis_angle = (|imu_angle-4704|)/987 - (|motor_angle-3416|)/1269

/* function ------------------------------------------------------------------*/


/**
  * @brief  ����PID��ʼ��
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
  * @brief  ��ȡ���̿�������
  * @param  ң������Ϣ�ṹ��
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
    
    
    /* Ŀ���ٶ�б�¼��� �ȶ����� */
    chassis_aim_speed[FRONT] = Misc_RAMP_Int16(front_temp, chassis_now_speed[FRONT], chassis_ramp_speed);
    chassis_aim_speed[RIGHT] = Misc_RAMP_Int16(right_temp, chassis_now_speed[RIGHT], chassis_ramp_speed);
    
    for(i=0; i<3; i++)
    {
        chassis_now_speed[i] = chassis_aim_speed[i];
    }
    
    Omni_Speed_Parse(DirAngle, chassis_aim_speed[FRONT], chassis_aim_speed[RIGHT]);
}

/**
  * @brief  ����ٶȿ���
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
                      
                      /* �¶ȴ���0.1 ���ٲ����� */
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
        /* �����ֵ */
        if (tmp_speed[i] > tmp_max_speed)
        {
            tmp_max_speed = tmp_speed[i];
        }
        
        /* ����Сֵ */
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
  * @brief  ����ģʽѡ��r��ѡ�񣿣�
  * @param  ����ָ��ṹ��
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
                    spin_s16temp = s16Follow_Speed_Spin(Chassis.MoveData.Right); /*< �����ٶ� */
                break;
                
                case SUP:
                    spin_s16temp = s16Follow_Holder_Spin(DirAngle); /*< ������̨ */
                break;
                
                case SDOWN:
                    spin_s16temp = s16Holder_Follow_GetMoveData(RDMsg); /*< ��̨�������*/
                break;
                
                default:
                    spin_s16temp = s16Follow_Holder_Spin(DirAngle); /*< ������̨ */
                break;
            }
        break; /*< end case SUP */
            
        case SDOWN:
            Dynamic_Spin();
        break; /*< end case SDOWN */
        
        case SMID: /*< ����ģʽ */
            /* R���л������С����ģʽ */
            
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
  * @brief  ����PID���
  * @param  void
  * @retval void
  * @attention ��ͨ���ٶȻ����⻷������ֵ�ټ����������ͨ�����������ڻ���ֱ�ӿ����������rx,lpf???��
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
  * @brief  ���̵��CAN
  * @param  void
  * @retval void
  * @attention ���ж�����
  */
void Chassis_CanTransmit(void)
{
    uint8_t i;
    if(Observer.Tx.DR16_Rate>15) /*< ң����������������16ʱ�ſ������� */
    {
        for(i=0;i<4;i++)
        {
            /* ����޷� */
            Chassis.M3508[i].LPf.Output = constrain_int16_t(Chassis.M3508[i].LPf.Output,
                                                    -Chassis_Out_Limit, Chassis_Out_Limit);
            
            /* CAN ��ֵ */
            Chassis.CanData[2*i]=(uint8_t)(Chassis.M3508[i].LPf.Output>>8);
            Chassis.CanData[2*i+1]=(uint8_t)(Chassis.M3508[i].LPf.Output);
        }
    }
    else
    {
        Chassis_Speed_Reset(); /*< �ر�ң�����󣬵���Ŀ���ٶ�һֱ���ֵ�ǰ״̬��0�� */
    }
    CAN2_Transmit(0x200,Chassis.CanData);
}


/**
  * @brief  ��������CAN����
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
  * @brief  ���ʿ��ƣ��򳬼����ݷ��������
  * @param  void
  * @retval void
  * @attention �������ݸ�������ʿ��Ƶ����������ֹ���̳�����
  */
static void Chassis_Power_limit_F(void)
{
    static uint8_t tick = 0;
    int16_t max_move_speed;
    int16_t max_spin_speed;

    uint16_t power_lim = JUDGE_u16GetChassisPowerLimit();
    uint16_t power_buf = JUDGE_u16GetRemainEnergy();
    float speed_bias = 10*(float)(Chassis.SuperCap.CapVol)/(float)JUDGE_u16GetChassisVolt();

    tick++;/*< ��ʱ*/
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
  * @brief  ���̽���
  * @param  ����ָ��ṹ��
  * @retval void
* @attention ����д��CAN����ǰ����V��һ����λ��PID�����������
  */
void Chassis_Process(RemoteData_t RDMsg)
{
    Chassis_Fly_Speed_Control();
    Chassis_Power_limit_F();
    Chassis_GetMoveData(RDMsg);
	Chassis_ChooseMode(RDMsg);
    Chassis_Protect(RDMsg);
    Turn_Round(RDMsg);    /*< һ����ͷ*/
    Fast_Fix_Spin(RDMsg); /*< ��E�������ٶ���С���� */
    Chassis_Speed_Control();
    Chassis_PidRun();
}

/*********************************���̸�������*********************************/


/**
  * @brief  �����ٶȳ�ʼ����PID�������
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
  * @brief  �����ٶȳ�ʼ����PID�������
  * @param  ����ָ��ṹ��
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
  * @brief  ���ٶ���С����
  * @param  ����ָ��ṹ��
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
  * @brief  ��̬С����(ʵ�⣺�������ٶȾͿ����ˣ�û��Ҫ��̬����ת�ٶ�)
  * @param  ����ָ��ṹ��
  * @retval void
  * @attention ��front �� right�ٶ� �� clockwise�ٶ� �ɷ��ȹ�ϵ��û�ã�
  */
void Dynamic_Spin(void)
{
    Chassis.MoveData.ClockWise = fmax_speed_spin;
    Chassis.MoveData.ClockWise = constrain_int16_t(Chassis.MoveData.ClockWise, -1310, 1310);
    JudgeUI_spin_state='l';
}


/**
  * @brief  (Q��)һ����ͷ����̨ƫ��+180�ȣ���������̨���+180��
  * @param  ����ָ��ṹ��
  * @retval void
  * @attention 
  */
void Turn_Round(RemoteData_t RDMsg)
{
    static uint8_t delay_tick = 0;
    static uint8_t state = 0;
   
    
    
    if (delay_tick < 100)
    {
        delay_tick++; /*< ��ʱ����ֹ����������*/
        
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
  * @brief  ���̸�����̨��ת
  * @param  ���̺���̨���
  * @retval ��ת�ٶ�
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
  * @brief  ���̸����ٶȷ�����ת
  * @param  �����ٶ�
  * @retval ��ת�ٶ�
  * @attention 
  */
int16_t s16Follow_Speed_Spin(float right_speed)
{   
    JudgeUI_spin_state='s';
    
    return 2.5f*right_speed;
}

/**
  * @brief  ȫ������ٶȽ���
  * @param  ��̨�͵��̲��
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
  * @brief  ���̿����ٶȽ���
  * @param  ң������Ϣ�ṹ��
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
  * @brief  ң���������ٶȽ���
  * @param  ң������Ϣ�ṹ��
  * @retval 
  * @attention 
  */
void Remote_Control_GetMoveData(RemoteData_t RDMsg)
{
    right_temp = RDMsg.Ch2;
    front_temp = RDMsg.Ch3;
}

/**
  * @brief  ��̨�������
  * @param  ң������Ϣ�ṹ��
  * @retval 
  * @attention ��������ͷģʽ����
  */
int16_t s16Holder_Follow_GetMoveData(RemoteData_t RDMsg)
{
    Chassis.MoveData.Right = -RDMsg.Ch2;
    Chassis.MoveData.Front = -RDMsg.Ch3;
    return RDMsg.Ch0; /*< ������ת�ٶ�*/
}

/**
  * @brief  �����ٶȿ���
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
