#include "pid.h"
/**
  * @brief  增量式PID初始化.
  * @param  PID结构体地址，P,I,D,积分限幅，增量限幅.
  * @note   .
  * @retval none.
  */
void pid_init_increment(PID_IncrementType *PID, float kp, float ki, float kd, float IncLim) //PID初始化系数
{
    PID->kp = kp;
    PID->ki = ki;
    PID->kd = kd;
    PID->IncLim = IncLim;
}

/**
  * @brief  普通的增量式PID（可以进行增量限幅）.
  * @param  目标值，反馈值，PID结构体地址.
  * @note   返回的并非是增量，而直接是需要输出的值.
  * @retval 需要输出的量.
  */
float pid_increment_update(float Target, float Current, PID_IncrementType *PID)
{
    float dErrP, dErrI, dErrD;

    PID->errNow = Target - Current; //误差量：目标-目前（相邻两次测量值取差分）

    dErrP = PID->errNow - PID->errOld1;					   //计算比例分量----微分：现误差-上一个误差
    dErrI = PID->errNow;								   //计算积分分量——————现误差
    dErrD = PID->errNow - 2 * PID->errOld1 + PID->errOld2; //计算微分分量——————现误差-2*一阶误差微分+二阶误差微分

    /*增量式PID计算*/
    PID->dCtrOut = PID->kp * dErrP + PID->ki * dErrI + PID->kd * dErrD; //PID合成输出量

    PID->errOld2 = PID->errOld1; //二阶误差微分
    PID->errOld1 = PID->errNow;  //一阶误差微分
    
    if (PID->dCtrOut < -PID->IncLim)
        PID->dCtrOut = -PID->IncLim;
    else if (PID->dCtrOut > PID->IncLim)
        PID->dCtrOut = PID->IncLim;

    PID->ctrOut += PID->dCtrOut;

    return PID->ctrOut;
}

/**
  * @brief  绝对式PID初始化.
  * @param  PID结构体地址，P,I,D,积分限幅.
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
  * @brief  普通绝对式PID.
  * @param  目标值，实际值，PID结构体地址.
  * @note   .
  * @retval 需要输出的值.
  */

float pid_absolute_update(float Target, float Current, PID_AbsoluteType *PID)
{
    PID->errNow = Target - Current;
    PID->errP = PID->errNow; //读取现在的误差，用于kp控制
    PID->errI += PID->errNow; //误差积分，用于ki控制
    if (PID->errLim != 0) //微分上限和下限
    {
        if (PID->errI > PID->errLim)
            PID->errI = PID->errLim;
        else if (PID->errI < -PID->errLim)
            PID->errI = -PID->errLim;
    }
    
    /* 比例误差上限和下限 */
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

    PID->errD = PID->errNow - PID->errOld; //误差微分，用于kd控制
    PID->errOld = PID->errNow; //保存现在的误差
    PID->ctrOut = PID->kp * PID->errP + PID->ki * PID->errI + PID->kd * PID->errD; //计算绝对式PID输出
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
    
    thisError=Target-Current; //得到偏差值
    pError=thisError-PID->lasterror;
    iError=thisError;
    increment=PID->kp*pError+PID->ki*iError+ PID-> deltadiff;   //增量计算
    
    PID->preerror=PID->lasterror;  //存放偏差用于下次运算
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
  * @brief  变结构PI控制器.
  * @param  p i参数范围，d参数， PID结构体指针.
  * @note   .
  * @retval none.
  */
void Holder_Pi_Tunning(float p_max, float p_min, float i_max, float i_min, float kd, PID_AntiIntegralType* PID)
{
	float err = PID->errNow;
	
	err = err<0? -err : err; /*< 取绝对值*/
	
	PID->kp = p_min + p_max*(1 - 1/(1+0.05f*(err+err*err))); /*< 误差大，P大，反应快 */
	
	PID->ki = i_min + i_max*(1/(1+0.5f*err*err*err*err)); /*< 误差小，I大， 稳定性强 */
	PID->kd = kd;
}


/**
  * @brief  变结构PI控制器.
  * @param  p i参数范围，d参数， PID结构体指针.
  * @note   .
  * @retval none.
  */
void Holder_Pi_Tunning_Absolute(float p_max, float p_min, float i_max, float i_min, float kd, PID_AbsoluteType* PID)
{
	float err = PID->errNow;
	
	err = err<0? -err : err; /*< 取绝对值*/
	
	PID->kp = p_min + p_max*(1 - 1/(1+0.001f*(err+err*err))); /*< 误差大，P大，反应快 */
	
	PID->ki = i_min + i_max*(1/(1+0.001f*(err+err*err))); /*< 误差小，I大， 稳定性强 */
	
    if (err < 20)
    {
        PID->ki = 0;
    }
    PID->kd = kd;
}

///**
//  * @brief  P误差限幅
//  * @param  p i参数范围，d参数， PID结构体指针.
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
    
    PID->errP = PID->kp * PID->errNow; //读取现在的误差，用于kp控制
    PID->errI = PID->errI + PID->ki * PID->errNow + PID->ki * PID->errSat; //误差积分，用于ki控制 
    PID->errD = PID->kd * (PID->errNow - PID->errOld); //误差微分，用于kd控制
    
    OutPreSat = PID->errP + PID->errI + PID->errD;
    
    if (OutPreSat > PID->outMax)
        PID->ctrOut = PID->outMax;
    else if (OutPreSat < PID->outMin)
        PID->ctrOut = PID->outMin;  
    else 
        PID->ctrOut = OutPreSat;
    PID->errSat = PID->ctrOut - OutPreSat;
    PID->errOld = PID->errNow; //保存现在的误差
    return PID->ctrOut;
}








/**
  * @brief  三段式PID初始化.
  * @param  PID结构体地址，P1,P2,P3,I,D,积分限幅，误差分段点0，1，可接受的误差，最大误差ERRMAX（超过此值直接输出恒值OUTMAX）.
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
  * @brief  3段P绝对式PID
  * @param  目标值，反馈值，PID结构体地址
  * @note   误差超过一定范围即直接输出恒值，暂未使用积分分离.
  * @retval 返回计算出的输出值.
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

    PID->errP = PID->errNow;  //读取现在的误差，用于kp控制

    PID->errI += PID->errNow; //误差积分，用于ki控制

    if(PID->errLim != 0)	   //微分上限和下限
    {
        if(PID->errI >  PID->errLim)
            PID->errI =  PID->errLim;
        else if(PID->errI <  -PID->errLim)
            PID->errI = -PID->errLim;
    }

    PID->errD = PID->errNow - PID->errOld;//误差微分，用于kd控制

    PID->errOld = PID->errNow;	//保存现在的误差

    PID->ctrOut = PID->kp * PID->errP + PID->ki * PID->errI + PID->kd * PID->errD;//计算绝对式PID输出

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


/* 用pid的控制模型做视觉模型 -by ryh */
float Vision_Update(float err,PID_AntiIntegralType* PID)
{
    float OutPreSat;
    
    PID->errNow = err;
    PID->errP = PID->kp * PID->errNow; //读取现在的误差，用于kp控制
    PID->errI = PID->errI + PID->ki * PID->errNow + PID->ki * PID->errSat; //误差积分，用于ki控制 
    PID->errD = PID->kd * (PID->errNow - PID->errOld); //误差微分，用于kd控制
    
    OutPreSat = PID->errP + PID->errI + PID->errD;
    
    if (OutPreSat > PID->outMax)
        PID->ctrOut = PID->outMax;
    else if (OutPreSat < PID->outMin)
        PID->ctrOut = PID->outMin;  
    else 
        PID->ctrOut = OutPreSat;
    PID->errSat = PID->ctrOut - OutPreSat;
    PID->errOld = PID->errNow; //保存现在的误差
    return PID->ctrOut;
}
