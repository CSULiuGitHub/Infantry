#ifndef __PID_H
#define __PID_H
#include <stdio.h>
#include <stdint.h>
typedef struct
{
    /*PID算法接口变量，用于给用户获取或修改PID算法的特性*/
    float kp; //比例系数
    float ki; //积分系数
    float kd; //微分系数

    float errNow;  //当前的误差
    float dCtrOut; //控制增量输出
    float ctrOut;  //控制输出

    float IncLim; //增量限幅
    /*PID算法内部变量，其值不能修改*/
    float errOld1;
    float errOld2;
} PID_IncrementType;    /* 增量式pid */

typedef struct
{
    float kp;
    float ki;
    float kd;
    float errILim_up;   //积分上限
    float errILim_down; //积分上限
    float errPLim; //比例上限
    float errLim;
    float errNow;
    float errOld;
    float errP;
    float errI;
    float errD;
    float ctrOut;
} PID_AbsoluteType;         /* 绝对式pid */

typedef struct
{
    float tempkp[3];
    float err[3];
    float kp;
    float ki;
    float kd;
    float errIUp;   //积分上限
    float errIDown; //积分上限
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
} PID_SectionType;     /* 分段式pid */

/* 微分先行by刘光耀 */
typedef struct  
{
    float kp;               //比例系数
    float ki;               //积分系数
    float kd;               //微分系数
    float lasterror;        //前一拍偏差
    float preerror;         //前两拍偏差
    float deadband;         //死区
    float ctrOut;           //输出值
    float deltadiff;        /*微分增量*/
    float integralValue;    /*积分累计量*/
    float gama;             /*微分先行滤波系数*/
    float lastPv;           /*上一拍的过程测量值*/
    float lastDeltaPv;      /*上一拍的过程测量值增量*/
    float outlim;		    /*输出限幅*/
}PID_PreDifferentialType;   /* 微分先行 */

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
} PID_AntiIntegralType; /* 抗积分饱和pid */



typedef struct
{
    float tempkp[3];
    float err[3];
    float kp;
    float ki;
    float kd;
    float errILim_up;//积分上限
    float errILim_down;//积分上限
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

/* 变结构PI控制器 */
void Holder_Pi_Tunning(float p_max, float p_min, float i_max, float i_min, float kd, PID_AntiIntegralType* PID);
void Holder_Pi_Tunning_Absolute(float p_max, float p_min, float i_max, float i_min, float kd, PID_AbsoluteType* PID);

int32_t Constrain_Int32_t(int32_t amt, int32_t low, int32_t high); /*< 限幅*/
/* 三段P式控制器 */
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
