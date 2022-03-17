#ifndef _HOLDER_CONFIG_H
#define _HOLDER_CONFIG_H


#include <stdint.h>
#include "config.h"

/******************************************************************************
  * @brief  ��̨������ز��� ��С�糵��ز���
  * @attention ˫�ջ����� 
  *     �⻷���ǶȻ������ñ�ṹPI����������С�������ӿ���Ӧ
  *     �ڻ����ٶȻ������þ���ʽPID������
  *     ���Է�����
  *         1.���� #define HOLDER_DEBUG����ͨ��debug�����޸Ĳ���
  *         2.���� #define HOLDER_PITCH_WAVE�������PITCH�Შ��
  *         3.���� #define HOLDER_YAW_WAVE, �����YAW�Შ��
  *         4.���� #define HOLDER_VISION_WAVE, ������Ӿ�����
  *         ����ʵ�ַ�����holder.c�е�static void Holder_Pid_Manual(void)������
  *
  *         ÿ��ֻ�ܶ���һ�����β��������������ע�͵�debug��ض���
  ****************************************************************************/

/* ���е���ǰ���붨��������������в��β���ֻ�ܶ���һ�� */
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

/*****************************���л���ͨ�ò���**********************************/
float rc_pitch_bias = 0.15f; /*< ң����PitchĿ��Ƕ���С���� */
float rc_yaw_bias = -0.32f;  /*< ң����YawĿ��Ƕ���С���� */
float key_pitch_bias = 1.0f; /*< ���PitchĿ��Ƕ���С���� */
float key_yaw_bias = 1.0f;   /*< ���PitchĿ��Ƕ���С���� */
int32_t key_yaw_limit = 250; /*< ���YAW�ٶ��޷� */

/*****************************��ͬ������ͬ����**********************************/
#ifdef INFANTRY_3 /*< ���Ų���*/
/* YAW�� */
float Yaw_Angle_Tunning_Kp = 0.05f;  /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float Yaw_Angle_Min_Kp     = 0.10f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float Yaw_Angle_Tunning_Ki = 0.0f;  /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float Yaw_Angle_Min_Ki     = 0.0f;  /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float Yaw_Angle_Kd         = 0.0f;  /*< �ǶȻ�D������΢�֣� */

float Yaw_Angle_Vision_Tunning_Kp = 0.08f;  /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float Yaw_Angle_Vision_Min_Kp     = 0.03f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float Yaw_Angle_Vision_Tunning_Ki = 0.0f;  /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float Yaw_Angle_Vision_Min_Ki     = 0.0f;  /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float Yaw_Angle_Vision_Kd         = 0.0f;  /*< �ǶȻ�D������΢�֣� */

float Yaw_Speed_kp = 5.0f;           /*< �ٶȻ�����P�������� */
float Yaw_Speed_ki = 0.01f;         /*< �ٶȻ�����I�����֣� */
float Yaw_Speed_kd = 0.0f;           /*< �ٶȻ�����D��΢�֣� */

float Yaw_Speed_Vision_kp = 5.0f;           /*< �ٶȻ�����P�������� */
float Yaw_Speed_Vision_ki = 0.01f;         /*< �ٶȻ�����I�����֣� */
float Yaw_Speed_Vision_kd = 0.0f;           /*< �ٶȻ�����D��΢�֣� */

float Yaw_Angle_Lpf_Bias = 0.9f;    /*< �ǶȻ���ͨ�˲�ϵ�� */
float Yaw_Speed_Lpf_Bias = 0.9f;    /*< �ٶȻ���ͨ�˲�ϵ�� */

float Yaw_Angle_Kp_Limit = 99999.0f;  /*< �ǶȻ�P��������޷� */
float Yaw_Angle_Ki_Limit = 999.0f;  /*< �ǶȻ�I��������޷� */
float Yaw_Speed_Kp_Limit = 99999.0f;  /*< �ٶȻ�P��������޷� */
float Yaw_Speed_Ki_Limit = 10000.0f;  /*< �ٶȻ�I��������޷� */

float Yaw_Angle_Vision_Kp_Limit = 99999.0f;  /*< �ǶȻ�P��������޷� */
float Yaw_Angle_Vision_Ki_Limit = 99999.0f;  /*< �ǶȻ�I��������޷� */

float Yaw_Output_Limit   = 28000.0f;  /*< ����޷���6020������30000 */

/* PITCH�� */
float Pitch_Angle_Tunning_Kp = 0.010f; /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float Pitch_Angle_Min_Kp     = 0.008f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float Pitch_Angle_Tunning_Ki = 0.0f;    /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float Pitch_Angle_Min_Ki     = 0.0f;    /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float Pitch_Angle_Kd         = 0.0f;    /*< �ǶȻ�D������΢�֣� */

float Pitch_Angle_Vision_Tunning_Kp = 0.001f; /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float Pitch_Angle_Vision_Min_Kp     = 0.00035f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float Pitch_Angle_Vision_Tunning_Ki = 0.0f;    /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float Pitch_Angle_Vision_Min_Ki     = 0.0f;    /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float Pitch_Angle_Vision_Kd         = 0.0f;    /*< �ǶȻ�D������΢�֣� */

float Pitch_Speed_kp = 385.0f;           /*< �ٶȻ�����P�������� */
float Pitch_Speed_ki = 0.52f;            /*< �ٶȻ�����I�����֣� */
float Pitch_Speed_kd = 0.0f;            /*< �ٶȻ�����D��΢�֣� */

float Pitch_Speed_Vision_kp = 325.0f;           /*< �ٶȻ�����P�������� */
float Pitch_Speed_Vision_ki = 0.12f;            /*< �ٶȻ�����I�����֣� */
float Pitch_Speed_Vision_kd = 0.0f;            /*< �ٶȻ�����D��΢�֣� */

float Pitch_Angle_Lpf_Bias = 0.8f;    /*< �ǶȻ���ͨ�˲�ϵ�� */
float Pitch_Speed_Lpf_Bias = 0.8f;    /*< �ٶȻ���ͨ�˲�ϵ�� */

float Pitch_Angle_Kp_Limit = 99999.0f;  /*< �ǶȻ�P��������޷� */
float Pitch_Angle_Ki_Limit = 99999.0f;  /*< �ǶȻ�I��������޷� */
float Pitch_Speed_Kp_Limit = 99999.0f;  /*< �ٶȻ�P��������޷� */
float Pitch_Speed_Ki_Limit = 9999.0f;  /*< �ٶȻ�I��������޷� */

float Pitch_Angle_Vision_Kp_Limit = 99999.0f;  /*< �ǶȻ�P��������޷� */
float Pitch_Angle_Vision_Ki_Limit = 99999.0f;  /*< �ǶȻ�I��������޷� */

float Pitch_Output_Limit   = 28000.0f;  /*< ����޷���6020������30000 */


/********************************��С�糵 �ǶȻ������ص����ٶȻ�����***************************/
/* YAW */
float Pinwheel_Yaw_Angle_Tunning_Kp = 1.8f; /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float Pinwheel_Yaw_Angle_Min_Kp     = 0.0f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float Pinwheel_Yaw_Angle_Tunning_Ki = 0.0f;    /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float Pinwheel_Yaw_Angle_Min_Ki     = 0.001f;    /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float Pinwheel_Yaw_Angle_Kd         = 0.8f;    /*< �ǶȻ�D������΢�֣� */

/* PITCH */
float Pinwheel_Pitch_Angle_Tunning_Kp = 0.08f; /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float Pinwheel_Pitch_Angle_Min_Kp     = 0.04f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float Pinwheel_Pitch_Angle_Tunning_Ki = 0.0f;    /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float Pinwheel_Pitch_Angle_Min_Ki     = 0.0f;    /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float Pinwheel_Pitch_Angle_Kd         = 0.02f;    /*< �ǶȻ�D������΢�֣� */
#endif /*< ifdef INFANTRY_3*/

#ifdef INFANTRY_4
/* YAW�� */
float Yaw_Angle_Tunning_Kp = 0.05f;  /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float Yaw_Angle_Min_Kp     = 0.10f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float Yaw_Angle_Tunning_Ki = 0.0f;  /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float Yaw_Angle_Min_Ki     = 0.0f;  /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float Yaw_Angle_Kd         = 0.0f;  /*< �ǶȻ�D������΢�֣� */

float Yaw_Angle_Vision_Tunning_Kp = 0.05f;  /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float Yaw_Angle_Vision_Min_Kp     = 0.03f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float Yaw_Angle_Vision_Tunning_Ki = 0.0f;  /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float Yaw_Angle_Vision_Min_Ki     = 0.0f;  /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float Yaw_Angle_Vision_Kd         = 0.0f;  /*< �ǶȻ�D������΢�֣� */

float Yaw_Speed_kp = 5.0f;           /*< �ٶȻ�����P�������� */
float Yaw_Speed_ki = 0.01f;         /*< �ٶȻ�����I�����֣� */
float Yaw_Speed_kd = 0.0f;           /*< �ٶȻ�����D��΢�֣� */

float Yaw_Speed_Vision_kp = 5.0f;           /*< �ٶȻ�����P�������� */
float Yaw_Speed_Vision_ki = 0.01f;         /*< �ٶȻ�����I�����֣� */
float Yaw_Speed_Vision_kd = 0.0f;           /*< �ٶȻ�����D��΢�֣� */

float Yaw_Angle_Lpf_Bias = 0.9f;    /*< �ǶȻ���ͨ�˲�ϵ�� */
float Yaw_Speed_Lpf_Bias = 0.9f;    /*< �ٶȻ���ͨ�˲�ϵ�� */

float Yaw_Angle_Kp_Limit = 99999.0f;  /*< �ǶȻ�P��������޷� */
float Yaw_Angle_Ki_Limit = 999.0f;  /*< �ǶȻ�I��������޷� */
float Yaw_Speed_Kp_Limit = 99999.0f;  /*< �ٶȻ�P��������޷� */
float Yaw_Speed_Ki_Limit = 10000.0f;  /*< �ٶȻ�I��������޷� */

float Yaw_Angle_Vision_Kp_Limit = 99999.0f;  /*< �ǶȻ�P��������޷� */
float Yaw_Angle_Vision_Ki_Limit = 99999.0f;  /*< �ǶȻ�I��������޷� */

float Yaw_Output_Limit   = 28000.0f;  /*< ����޷���6020������30000 */

/* PITCH�� */
float Pitch_Angle_Tunning_Kp = 0.0045f; /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float Pitch_Angle_Min_Kp     = 0.0025f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float Pitch_Angle_Tunning_Ki = 0.0f;    /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float Pitch_Angle_Min_Ki     = 0.0f;    /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float Pitch_Angle_Kd         = 0.0f;    /*< �ǶȻ�D������΢�֣� */

float Pitch_Angle_Vision_Tunning_Kp = 0.001f; /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float Pitch_Angle_Vision_Min_Kp     = 0.00035f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float Pitch_Angle_Vision_Tunning_Ki = 0.0f;    /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float Pitch_Angle_Vision_Min_Ki     = 0.0f;    /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float Pitch_Angle_Vision_Kd         = 0.0f;    /*< �ǶȻ�D������΢�֣� */

float Pitch_Speed_kp = 385.0f;           /*< �ٶȻ�����P�������� */
float Pitch_Speed_ki = 0.52f;            /*< �ٶȻ�����I�����֣� */
float Pitch_Speed_kd = 0.0f;            /*< �ٶȻ�����D��΢�֣� */

float Pitch_Speed_Vision_kp = 325.0f;           /*< �ٶȻ�����P�������� */
float Pitch_Speed_Vision_ki = 0.12f;            /*< �ٶȻ�����I�����֣� */
float Pitch_Speed_Vision_kd = 0.0f;            /*< �ٶȻ�����D��΢�֣� */

float Pitch_Angle_Lpf_Bias = 0.8f;    /*< �ǶȻ���ͨ�˲�ϵ�� */
float Pitch_Speed_Lpf_Bias = 0.8f;    /*< �ٶȻ���ͨ�˲�ϵ�� */

float Pitch_Angle_Kp_Limit = 99999.0f;  /*< �ǶȻ�P��������޷� */
float Pitch_Angle_Ki_Limit = 99999.0f;  /*< �ǶȻ�I��������޷� */
float Pitch_Speed_Kp_Limit = 99999.0f;  /*< �ٶȻ�P��������޷� */
float Pitch_Speed_Ki_Limit = 9999.0f;  /*< �ٶȻ�I��������޷� */

float Pitch_Angle_Vision_Kp_Limit = 99999.0f;  /*< �ǶȻ�P��������޷� */
float Pitch_Angle_Vision_Ki_Limit = 99999.0f;  /*< �ǶȻ�I��������޷� */

float Pitch_Output_Limit   = 28000.0f;  /*< ����޷���6020������30000 */


/********************************��С�糵 �ǶȻ������ص����ٶȻ�����***************************/
/* YAW */
float Pinwheel_Yaw_Angle_Tunning_Kp = 1.8f; /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float Pinwheel_Yaw_Angle_Min_Kp     = 0.0f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float Pinwheel_Yaw_Angle_Tunning_Ki = 0.0f;    /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float Pinwheel_Yaw_Angle_Min_Ki     = 0.001f;    /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float Pinwheel_Yaw_Angle_Kd         = 0.8f;    /*< �ǶȻ�D������΢�֣� */

/* PITCH */
float Pinwheel_Pitch_Angle_Tunning_Kp = 0.08f; /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float Pinwheel_Pitch_Angle_Min_Kp     = 0.04f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float Pinwheel_Pitch_Angle_Tunning_Ki = 0.0f;    /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float Pinwheel_Pitch_Angle_Min_Ki     = 0.0f;    /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float Pinwheel_Pitch_Angle_Kd         = 0.02f;    /*< �ǶȻ�D������΢�֣� */
#endif /*< ifdef INFANTRY_4*/

#ifdef INFANTRY_5
/* YAW�� */
float Yaw_Angle_Tunning_Kp = 0.05f;  /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float Yaw_Angle_Min_Kp     = 0.10f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float Yaw_Angle_Tunning_Ki = 0.0f;  /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float Yaw_Angle_Min_Ki     = 0.0f;  /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float Yaw_Angle_Kd         = 0.0f;  /*< �ǶȻ�D������΢�֣� */

float Yaw_Angle_Vision_Tunning_Kp = 0.10f;  /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float Yaw_Angle_Vision_Min_Kp     = 0.05f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float Yaw_Angle_Vision_Tunning_Ki = 0.0f;  /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float Yaw_Angle_Vision_Min_Ki     = 0.0f;  /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float Yaw_Angle_Vision_Kd         = 0.0f;  /*< �ǶȻ�D������΢�֣� */

float Yaw_Speed_kp = 5.0f;           /*< �ٶȻ�����P�������� */
float Yaw_Speed_ki = 0.01f;         /*< �ٶȻ�����I�����֣� */
float Yaw_Speed_kd = 0.0f;           /*< �ٶȻ�����D��΢�֣� */

float Yaw_Speed_Vision_kp = 5.0f;           /*< �ٶȻ�����P�������� */
float Yaw_Speed_Vision_ki = 0.01f;         /*< �ٶȻ�����I�����֣� */
float Yaw_Speed_Vision_kd = 0.0f;           /*< �ٶȻ�����D��΢�֣� */

float Yaw_Angle_Lpf_Bias = 0.9f;    /*< �ǶȻ���ͨ�˲�ϵ�� */
float Yaw_Speed_Lpf_Bias = 0.9f;    /*< �ٶȻ���ͨ�˲�ϵ�� */

float Yaw_Angle_Kp_Limit = 99999.0f;  /*< �ǶȻ�P��������޷� */
float Yaw_Angle_Ki_Limit = 999.0f;  /*< �ǶȻ�I��������޷� */
float Yaw_Speed_Kp_Limit = 99999.0f;  /*< �ٶȻ�P��������޷� */
float Yaw_Speed_Ki_Limit = 10000.0f;  /*< �ٶȻ�I��������޷� */

float Yaw_Angle_Vision_Kp_Limit = 99999.0f;  /*< �ǶȻ�P��������޷� */
float Yaw_Angle_Vision_Ki_Limit = 99999.0f;  /*< �ǶȻ�I��������޷� */

float Yaw_Output_Limit   = 28000.0f;  /*< ����޷���6020������30000 */

/* PITCH�� */
float Pitch_Angle_Tunning_Kp = 0.008f; /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float Pitch_Angle_Min_Kp     = 0.0055f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float Pitch_Angle_Tunning_Ki = 0.0f;    /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float Pitch_Angle_Min_Ki     = 0.0f;    /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float Pitch_Angle_Kd         = 0.0f;    /*< �ǶȻ�D������΢�֣� */

float Pitch_Angle_Vision_Tunning_Kp = 0.002f; /*< �ǶȻ�����P�仯ֵ����ṹ������0.001f */ 
float Pitch_Angle_Vision_Min_Kp     = 0.00055f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������0.00035f */
float Pitch_Angle_Vision_Tunning_Ki = 0.0f;    /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float Pitch_Angle_Vision_Min_Ki     = 0.0f;    /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float Pitch_Angle_Vision_Kd         = 0.0f;    /*< �ǶȻ�D������΢�֣� */

float Pitch_Speed_kp = 325.0f;           /*< �ٶȻ�����P�������� */
float Pitch_Speed_ki = 0.12f;            /*< �ٶȻ�����I�����֣� */
float Pitch_Speed_kd = 0.0f;            /*< �ٶȻ�����D��΢�֣� */

float Pitch_Speed_Vision_kp = 325.0f;           /*< �ٶȻ�����P�������� */
float Pitch_Speed_Vision_ki = 0.12f;            /*< �ٶȻ�����I�����֣� */
float Pitch_Speed_Vision_kd = 0.0f;            /*< �ٶȻ�����D��΢�֣� */


float Pitch_Angle_Lpf_Bias = 0.8f;    /*< �ǶȻ���ͨ�˲�ϵ�� */
float Pitch_Speed_Lpf_Bias = 0.8f;    /*< �ٶȻ���ͨ�˲�ϵ�� */

float Pitch_Angle_Kp_Limit = 99999.0f;  /*< �ǶȻ�P��������޷� */
float Pitch_Angle_Ki_Limit = 99999.0f;  /*< �ǶȻ�I��������޷� */
float Pitch_Speed_Kp_Limit = 99999.0f;  /*< �ٶȻ�P��������޷� */
float Pitch_Speed_Ki_Limit = 9999.0f;  /*< �ٶȻ�I��������޷� */

float Pitch_Angle_Vision_Kp_Limit = 99999.0f;  /*< �ǶȻ�P��������޷� */
float Pitch_Angle_Vision_Ki_Limit = 99999.0f;  /*< �ǶȻ�I��������޷� */

float Pitch_Output_Limit   = 28000.0f;  /*< ����޷���6020������30000 */


/********************************��С�糵 �ǶȻ������ص����ٶȻ�����***************************/
/* YAW */
float Pinwheel_Yaw_Angle_Tunning_Kp = 1.8f; /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float Pinwheel_Yaw_Angle_Min_Kp     = 0.0f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float Pinwheel_Yaw_Angle_Tunning_Ki = 0.0f;    /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float Pinwheel_Yaw_Angle_Min_Ki     = 0.001f;    /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float Pinwheel_Yaw_Angle_Kd         = 0.8f;    /*< �ǶȻ�D������΢�֣� */

/* PITCH */
float Pinwheel_Pitch_Angle_Tunning_Kp = 0.08f; /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float Pinwheel_Pitch_Angle_Min_Kp     = 0.04f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float Pinwheel_Pitch_Angle_Tunning_Ki = 0.0f;    /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float Pinwheel_Pitch_Angle_Min_Ki     = 0.0f;    /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float Pinwheel_Pitch_Angle_Kd         = 0.02f;    /*< �ǶȻ�D������΢�֣� */
#endif /*< ifdef INFANTRY_5*/

#ifdef INFANTRY_6
/* YAW�� */
float Yaw_Angle_Tunning_Kp = 0.05f;  /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float Yaw_Angle_Min_Kp     = 0.10f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float Yaw_Angle_Tunning_Ki = 0.0f;  /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float Yaw_Angle_Min_Ki     = 0.0f;  /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float Yaw_Angle_Kd         = 0.0f;  /*< �ǶȻ�D������΢�֣� */

float Yaw_Angle_Vision_Tunning_Kp = 0.05f;  /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float Yaw_Angle_Vision_Min_Kp     = 0.03f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float Yaw_Angle_Vision_Tunning_Ki = 0.0f;  /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float Yaw_Angle_Vision_Min_Ki     = 0.0f;  /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float Yaw_Angle_Vision_Kd         = 0.0f;  /*< �ǶȻ�D������΢�֣� */

float Yaw_Speed_kp = 5.0f;           /*< �ٶȻ�����P�������� */
float Yaw_Speed_ki = 0.01f;         /*< �ٶȻ�����I�����֣� */
float Yaw_Speed_kd = 0.0f;           /*< �ٶȻ�����D��΢�֣� */

float Yaw_Speed_Vision_kp = 5.0f;           /*< �ٶȻ�����P�������� */
float Yaw_Speed_Vision_ki = 0.01f;         /*< �ٶȻ�����I�����֣� */
float Yaw_Speed_Vision_kd = 0.0f;           /*< �ٶȻ�����D��΢�֣� */

float Yaw_Angle_Lpf_Bias = 0.9f;    /*< �ǶȻ���ͨ�˲�ϵ�� */
float Yaw_Speed_Lpf_Bias = 0.9f;    /*< �ٶȻ���ͨ�˲�ϵ�� */

float Yaw_Angle_Kp_Limit = 99999.0f;  /*< �ǶȻ�P��������޷� */
float Yaw_Angle_Ki_Limit = 999.0f;  /*< �ǶȻ�I��������޷� */
float Yaw_Speed_Kp_Limit = 99999.0f;  /*< �ٶȻ�P��������޷� */
float Yaw_Speed_Ki_Limit = 10000.0f;  /*< �ٶȻ�I��������޷� */

float Yaw_Angle_Vision_Kp_Limit = 99999.0f;  /*< �ǶȻ�P��������޷� */
float Yaw_Angle_Vision_Ki_Limit = 99999.0f;  /*< �ǶȻ�I��������޷� */

float Yaw_Output_Limit   = 28000.0f;  /*< ����޷���6020������30000 */

/* PITCH�� */
float Pitch_Angle_Tunning_Kp = 0.035; /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float Pitch_Angle_Min_Kp     = 0.02f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float Pitch_Angle_Tunning_Ki = 0.0f;    /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float Pitch_Angle_Min_Ki     = 0.0f;    /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float Pitch_Angle_Kd         = 0.1f;    /*< �ǶȻ�D������΢�֣� */

float Pitch_Angle_Vision_Tunning_Kp = 0.001f; /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float Pitch_Angle_Vision_Min_Kp     = 0.00035f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float Pitch_Angle_Vision_Tunning_Ki = 0.0f;    /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float Pitch_Angle_Vision_Min_Ki     = 0.0f;    /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float Pitch_Angle_Vision_Kd         = 0.0f;    /*< �ǶȻ�D������΢�֣� */

float Pitch_Speed_kp = 75.0f;           /*< �ٶȻ�����P�������� */
float Pitch_Speed_ki = 0.0f;            /*< �ٶȻ�����I�����֣� */
float Pitch_Speed_kd = 25.0f;            /*< �ٶȻ�����D��΢�֣� */

float Pitch_Speed_Vision_kp = 325.0f;           /*< �ٶȻ�����P�������� */
float Pitch_Speed_Vision_ki = 0.12f;            /*< �ٶȻ�����I�����֣� */
float Pitch_Speed_Vision_kd = 0.0f;            /*< �ٶȻ�����D��΢�֣� */

float Pitch_Angle_Lpf_Bias = 0.8f;    /*< �ǶȻ���ͨ�˲�ϵ�� */
float Pitch_Speed_Lpf_Bias = 0.8f;    /*< �ٶȻ���ͨ�˲�ϵ�� */

float Pitch_Angle_Kp_Limit = 99999.0f;  /*< �ǶȻ�P��������޷� */
float Pitch_Angle_Ki_Limit = 99999.0f;  /*< �ǶȻ�I��������޷� */
float Pitch_Speed_Kp_Limit = 99999.0f;  /*< �ٶȻ�P��������޷� */
float Pitch_Speed_Ki_Limit = 9999.0f;  /*< �ٶȻ�I��������޷� */

float Pitch_Angle_Vision_Kp_Limit = 99999.0f;  /*< �ǶȻ�P��������޷� */
float Pitch_Angle_Vision_Ki_Limit = 99999.0f;  /*< �ǶȻ�I��������޷� */

float Pitch_Output_Limit   = 28000.0f;  /*< ����޷���6020������30000 */

/********************************��С�糵 �ǶȻ������ص����ٶȻ�����***************************/
/* YAW */
float Pinwheel_Yaw_Angle_Tunning_Kp = 1.8f; /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float Pinwheel_Yaw_Angle_Min_Kp     = 0.0f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float Pinwheel_Yaw_Angle_Tunning_Ki = 0.0f;    /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float Pinwheel_Yaw_Angle_Min_Ki     = 0.001f;    /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float Pinwheel_Yaw_Angle_Kd         = 0.8f;    /*< �ǶȻ�D������΢�֣� */

/* PITCH */
float Pinwheel_Pitch_Angle_Tunning_Kp = 0.08f; /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float Pinwheel_Pitch_Angle_Min_Kp     = 0.04f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float Pinwheel_Pitch_Angle_Tunning_Ki = 0.0f;    /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float Pinwheel_Pitch_Angle_Min_Ki     = 0.0f;    /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float Pinwheel_Pitch_Angle_Kd         = 0.02f;    /*< �ǶȻ�D������΢�֣� */
#endif /*< ifdef INFANTRY_6*/

#endif /*< ifdef HOLDER_CONFIG_H*/
