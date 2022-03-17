#include "pid.h"
/**
  * @brief  ����ʽPID��ʼ��.
  * @param  PID�ṹ���ַ��P,I,D,�����޷��������޷�.
  * @note   .
  * @retval none.
  */
void pid_init_increment(PID_IncrementType *PID, float kp, float ki, float kd, float IncLim) //PID��ʼ��ϵ��
{
    PID->kp = kp;
    PID->ki = ki;
    PID->kd = kd;
    PID->IncLim = IncLim;
}

/**
  * @brief  ��ͨ������ʽPID�����Խ��������޷���.
  * @param  Ŀ��ֵ������ֵ��PID�ṹ���ַ.
  * @note   ���صĲ�������������ֱ������Ҫ�����ֵ.
  * @retval ��Ҫ�������.
  */
float pid_increment_update(float Target, float Current, PID_IncrementType *PID)
{
    float dErrP, dErrI, dErrD;

    PID->errNow = Target - Current; //�������Ŀ��-Ŀǰ���������β���ֵȡ��֣�

    dErrP = PID->errNow - PID->errOld1;					   //�����������----΢�֣������-��һ�����
    dErrI = PID->errNow;								   //������ַ��������������������
    dErrD = PID->errNow - 2 * PID->errOld1 + PID->errOld2; //����΢�ַ��������������������-2*һ�����΢��+�������΢��

    /*����ʽPID����*/
    PID->dCtrOut = PID->kp * dErrP + PID->ki * dErrI + PID->kd * dErrD; //PID�ϳ������

    PID->errOld2 = PID->errOld1; //�������΢��
    PID->errOld1 = PID->errNow;  //һ�����΢��
    
    if (PID->dCtrOut < -PID->IncLim)
        PID->dCtrOut = -PID->IncLim;
    else if (PID->dCtrOut > PID->IncLim)
        PID->dCtrOut = PID->IncLim;

    PID->ctrOut += PID->dCtrOut;

    return PID->ctrOut;
}

/**
  * @brief  ����ʽPID��ʼ��.
  * @param  PID�ṹ���ַ��P,I,D,�����޷�.
  * @note   .
  * @retval none.
  */
void pid_init_absolute(PID_AbsoluteType *PID, float kp, float ki, float kd, float errlimit, float errPLim)
{
    PID->kp = kp;
    PID->ki = ki;
    PID->kd = kd;
    PID->errLim = errlimit;
    PID->errPLim = errPLim;
}


/**
  * @brief  ��ͨ����ʽPID.
  * @param  Ŀ��ֵ��ʵ��ֵ��PID�ṹ���ַ.
  * @note   .
  * @retval ��Ҫ�����ֵ.
  */

float pid_absolute_update(float Target, float Current, PID_AbsoluteType *PID)
{
    PID->errNow = Target - Current;
    PID->errP = PID->errNow; //��ȡ���ڵ�������kp����
    PID->errI += PID->errNow; //�����֣�����ki����
    if (PID->errLim != 0) //΢�����޺�����
    {
        if (PID->errI > PID->errLim)
            PID->errI = PID->errLim;
        else if (PID->errI < -PID->errLim)
            PID->errI = -PID->errLim;
    }
    
    /* ����������޺����� */
    if (PID->errPLim != 0)
    {
        if (PID->errP > PID->errPLim)
        {
            PID->errP = PID->errPLim;
        }
        else if (PID->errP < -PID->errPLim)
        {
            PID->errP = -PID->errPLim;
        }
    }
    
    
//    if (PID->errNow < 5)
//    {
//        PID->errI = 0;
//    }

    PID->errD = PID->errNow - PID->errOld; //���΢�֣�����kd����
    PID->errOld = PID->errNow; //�������ڵ����
    PID->ctrOut = PID->kp * PID->errP + PID->ki * PID->errI + PID->kd * PID->errD; //�������ʽPID���
    return PID->ctrOut;
}

void pid_init_predifferential(PID_PreDifferentialType *PID,float kp, float ki, float kd, float gama, float outlim)
{
	PID-> kp = kp;
	PID-> ki = ki;
	PID-> kd = kd;
	PID-> gama = gama;
	PID-> outlim = outlim;
}


float pid_predifferential_update(float Target,float Current,PID_PreDifferentialType *PID)
{
    float thisError;
    float increment;
    float pError,iError;
    float c1,c2,c3,temp;
    float deltaPv;
    
    temp= PID-> gama * PID-> kd + PID-> kp;
    c3= PID-> kd/temp;
    c2=( PID-> kd+ PID-> kp)/temp;
    c1= PID-> gama*c3;
    
    deltaPv= Current- PID-> lastDeltaPv;
    PID-> deltadiff =c1* PID-> deltadiff +c2* deltaPv +c3* PID-> lastDeltaPv;
    
    thisError=Target-Current; //�õ�ƫ��ֵ
    pError=thisError-PID->lasterror;
    iError=thisError;
    increment=PID->kp*pError+PID->ki*iError+ PID-> deltadiff;   //��������
    
    PID->preerror=PID->lasterror;  //���ƫ�������´�����
    PID->lastDeltaPv=deltaPv;
    PID->lastPv= Current;
    PID->lasterror=thisError;
    PID->ctrOut+=increment;
    
    if(PID->outlim!=0)
    {
        if (PID->ctrOut > PID->outlim)
            PID->ctrOut = PID->outlim;
        else if (PID->ctrOut < -PID->outlim)
            PID->ctrOut = -PID->outlim;        
    }
    return PID->ctrOut;
}


void pid_init_antiintegral(PID_AntiIntegralType *PID, float kp, float ki, float kd, float outMax,float outMin)
{
    PID->kp = kp;
    PID->ki = ki;
    PID->kd = kd;
    PID->outMax = outMax;
    PID->outMin = outMin;
}


/**
  * @brief  ��ṹPI������.
  * @param  p i������Χ��d������ PID�ṹ��ָ��.
  * @note   .
  * @retval none.
  */
void Holder_Pi_Tunning(float p_max, float p_min, float i_max, float i_min, float kd, PID_AntiIntegralType* PID)
{
	float err = PID->errNow;
	
	err = err<0? -err : err; /*< ȡ����ֵ*/
	
	PID->kp = p_min + p_max*(1 - 1/(1+0.05f*(err+err*err))); /*< ����P�󣬷�Ӧ�� */
	
	PID->ki = i_min + i_max*(1/(1+0.5f*err*err*err*err)); /*< ���С��I�� �ȶ���ǿ */
	PID->kd = kd;
}


/**
  * @brief  ��ṹPI������.
  * @param  p i������Χ��d������ PID�ṹ��ָ��.
  * @note   .
  * @retval none.
  */
void Holder_Pi_Tunning_Absolute(float p_max, float p_min, float i_max, float i_min, float kd, PID_AbsoluteType* PID)
{
	float err = PID->errNow;
	
	err = err<0? -err : err; /*< ȡ����ֵ*/
	
	PID->kp = p_min + p_max*(1 - 1/(1+0.001f*(err+err*err))); /*< ����P�󣬷�Ӧ�� */
	
	PID->ki = i_min + i_max*(1/(1+0.001f*(err+err*err))); /*< ���С��I�� �ȶ���ǿ */
	
    if (err < 20)
    {
        PID->ki = 0;
    }
    PID->kd = kd;
}

///**
//  * @brief  P����޷�
//  * @param  p i������Χ��d������ PID�ṹ��ָ��.
//  * @note   .
//  * @retval none.
//  */
//void Perr_Limit(float err1, float err2, float err3)
//{
//    
//}


float pid_antiintegral_update(float Target,float Current,PID_AntiIntegralType* PID)
{
    float OutPreSat;
	

    
    PID->errNow = Target - Current;
    
    PID->errP = PID->kp * PID->errNow; //��ȡ���ڵ�������kp����
    PID->errI = PID->errI + PID->ki * PID->errNow + PID->ki * PID->errSat; //�����֣�����ki���� 
    PID->errD = PID->kd * (PID->errNow - PID->errOld); //���΢�֣�����kd����
    
    OutPreSat = PID->errP + PID->errI + PID->errD;
    
    if (OutPreSat > PID->outMax)
        PID->ctrOut = PID->outMax;
    else if (OutPreSat < PID->outMin)
        PID->ctrOut = PID->outMin;  
    else 
        PID->ctrOut = OutPreSat;
    PID->errSat = PID->ctrOut - OutPreSat;
    PID->errOld = PID->errNow; //�������ڵ����
    return PID->ctrOut;
}








/**
  * @brief  ����ʽPID��ʼ��.
  * @param  PID�ṹ���ַ��P1,P2,P3,I,D,�����޷������ֶε�0��1���ɽ��ܵ���������ERRMAX��������ֱֵ�������ֵOUTMAX��.
  * @note   .
  * @retval none.
  */
void	holder_pid_init_absolute_section(PID_AbsoluteType_section* PID,\
        float kp1,float kp2,float kp3, float ki, float kd, float errlimit,\
        float err0,float err1,float err2,float AccErr,float ERRMAX,float OUTMAX)
{
    PID->tempkp[0]		= kp1;
    PID->tempkp[1]		= kp2;
    PID->tempkp[2]		= kp3;
    PID->err[0]   =  err0;
    PID->err[1]   =  err1;
    PID->err[1]   =  err2;
    PID->ki		= ki;
    PID->kd		= kd;
    PID->errLim 	= errlimit;
    PID->errNow= 0;
    PID->errP= 0;
    PID->errI= 0;
    PID->errD= 0;
    PID->errOld= 0;
    PID->ctrOut= 0;
    PID->AccErr = AccErr;
    PID->ERRMAX = ERRMAX;
    PID->OUTMAX = OUTMAX;
}

/**
  * @brief  3��P����ʽPID
  * @param  Ŀ��ֵ������ֵ��PID�ṹ���ַ
  * @note   ����һ����Χ��ֱ�������ֵ����δʹ�û��ַ���.
  * @retval ���ؼ���������ֵ.
  */
float PID_Update_Yaw_index(float Target,float Current,PID_AbsoluteType_section* PID)
{
    PID->errNow = Target - Current;

    if(PID->errNow > -PID->err[0] && PID->errNow < PID->err[0])
        PID->kp = PID ->tempkp[2];
    else if(PID->errNow > -PID->err[1] && PID->errNow < PID->err[1])
        PID->kp = PID->tempkp[1];
    else
        PID->kp = PID->tempkp[0];

    PID->errP = PID->errNow;  //��ȡ���ڵ�������kp����

    PID->errI += PID->errNow; //�����֣�����ki����

    if(PID->errLim != 0)	   //΢�����޺�����
    {
        if(PID->errI >  PID->errLim)
            PID->errI =  PID->errLim;
        else if(PID->errI <  -PID->errLim)
            PID->errI = -PID->errLim;
    }

    PID->errD = PID->errNow - PID->errOld;//���΢�֣�����kd����

    PID->errOld = PID->errNow;	//�������ڵ����

    PID->ctrOut = PID->kp * PID->errP + PID->ki * PID->errI + PID->kd * PID->errD;//�������ʽPID���

    if(PID->errNow < -PID->ERRMAX)
        return -PID->OUTMAX;
    else if(PID->errNow > PID->ERRMAX)
        return PID->OUTMAX;
    else
    {
        if(PID->ctrOut < -PID->OUTMAX)
            return -PID->OUTMAX;
        else if(PID->ctrOut > PID->OUTMAX)
            return PID->OUTMAX;
        else
            return PID->ctrOut;
    }
}


/* ��pid�Ŀ���ģ�����Ӿ�ģ�� -by ryh */
float Vision_Update(float err,PID_AntiIntegralType* PID)
{
    float OutPreSat;
    
    PID->errNow = err;
    PID->errP = PID->kp * PID->errNow; //��ȡ���ڵ�������kp����
    PID->errI = PID->errI + PID->ki * PID->errNow + PID->ki * PID->errSat; //�����֣�����ki���� 
    PID->errD = PID->kd * (PID->errNow - PID->errOld); //���΢�֣�����kd����
    
    OutPreSat = PID->errP + PID->errI + PID->errD;
    
    if (OutPreSat > PID->outMax)
        PID->ctrOut = PID->outMax;
    else if (OutPreSat < PID->outMin)
        PID->ctrOut = PID->outMin;  
    else 
        PID->ctrOut = OutPreSat;
    PID->errSat = PID->ctrOut - OutPreSat;
    PID->errOld = PID->errNow; //�������ڵ����
    return PID->ctrOut;
}
