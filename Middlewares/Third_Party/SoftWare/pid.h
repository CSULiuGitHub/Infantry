#ifndef __PID_H
#define __PID_H
#include <stdio.h>
#include <stdint.h>
typedef struct
{
    /*PID�㷨�ӿڱ��������ڸ��û���ȡ���޸�PID�㷨������*/
    float kp; //����ϵ��
    float ki; //����ϵ��
    float kd; //΢��ϵ��

    float errNow;  //��ǰ�����
    float dCtrOut; //�����������
    float ctrOut;  //�������

    float IncLim; //�����޷�
    /*PID�㷨�ڲ���������ֵ�����޸�*/
    float errOld1;
    float errOld2;
} PID_IncrementType;    /* ����ʽpid */

typedef struct
{
    float kp;
    float ki;
    float kd;
    float errILim_up;   //��������
    float errILim_down; //��������
    float errPLim; //��������
    float errLim;
    float errNow;
    float errOld;
    float errP;
    float errI;
    float errD;
    float ctrOut;
} PID_AbsoluteType;         /* ����ʽpid */

typedef struct
{
    float tempkp[3];
    float err[3];
    float kp;
    float ki;
    float kd;
    float errIUp;   //��������
    float errIDown; //��������
    float errLim;
    float errNow;
    float errOld;
    float errP;
    float errI;
    float errD;
    float ctrOut;
    float errMax;
    float outMax;
    float accErr;
} PID_SectionType;     /* �ֶ�ʽpid */

/* ΢������by����ҫ */
typedef struct  
{
    float kp;               //����ϵ��
    float ki;               //����ϵ��
    float kd;               //΢��ϵ��
    float lasterror;        //ǰһ��ƫ��
    float preerror;         //ǰ����ƫ��
    float deadband;         //����
    float ctrOut;           //���ֵ
    float deltadiff;        /*΢������*/
    float integralValue;    /*�����ۼ���*/
    float gama;             /*΢�������˲�ϵ��*/
    float lastPv;           /*��һ�ĵĹ��̲���ֵ*/
    float lastDeltaPv;      /*��һ�ĵĹ��̲���ֵ����*/
    float outlim;		    /*����޷�*/
}PID_PreDifferentialType;   /* ΢������ */

typedef struct
{
    float kp;
    float ki;
    float kd;
    float errNow;
    float errOld;
    float errP;
    float errI;
    float errD;
    float errSat;
    float ctrOut;
    float outMax;
    float outMin;
} PID_AntiIntegralType; /* �����ֱ���pid */



typedef struct
{
    float tempkp[3];
    float err[3];
    float kp;
    float ki;
    float kd;
    float errILim_up;//��������
    float errILim_down;//��������
    float errLim;
    float errNow;
    float errOld;
    float errP;
    float errI;
    float errD;
    float ctrOut;
    float ERRMAX;
    float OUTMAX;
    float AccErr;

} PID_AbsoluteType_section;

/* ��ṹPI������ */
void Holder_Pi_Tunning(float p_max, float p_min, float i_max, float i_min, float kd, PID_AntiIntegralType* PID);
void Holder_Pi_Tunning_Absolute(float p_max, float p_min, float i_max, float i_min, float kd, PID_AbsoluteType* PID);

int32_t Constrain_Int32_t(int32_t amt, int32_t low, int32_t high); /*< �޷�*/
/* ����Pʽ������ */
void	holder_pid_init_absolute_section(PID_AbsoluteType_section* PID,\
        float kp1,float kp2,float kp3, float ki, float kd, float errlimit,\
        float err0,float err1,float err2,float AccErr,float ERRMAX,float OUTMAX);
float PID_Update_Yaw_index(float Target,float Current,PID_AbsoluteType_section* PID);


void    pid_init_increment(PID_IncrementType *PID, float kp, float ki, float kd, float IncLim);
float   pid_increment_update(float Target, float Current, PID_IncrementType *PID);

void    pid_init_absolute(PID_AbsoluteType *PID, float kp, float ki, float kd, float errlimit, float errPLim);
float   pid_absolute_update(float Target, float Current, PID_AbsoluteType *PID);

void    pid_init_predifferential(PID_PreDifferentialType *PID,float kp, float ki, float kd, float gama, float outlim);
float   pid_predifferential_update(float Target,float Current,PID_PreDifferentialType *PID);

void	pid_init_section(PID_SectionType* PID,float kp1,float kp2,float kp3, float ki, float kd, float errlimit,float err1,float err2,float err3,float accerr,float errmax,float outmax);
float   pid_section_update(float Target,float Current,PID_SectionType* PID);

void    pid_init_antiintegral(PID_AntiIntegralType *PID, float kp, float ki, float kd, float outMax,float outMin);
float   pid_antiintegral_update(float Target,float Current,PID_AntiIntegralType* PID);
		
float Vision_Update(float err,PID_AntiIntegralType* PID);


#endif
