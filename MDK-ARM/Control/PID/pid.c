/**
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 * @file       pid.c
 * @brief     
 * @note
 * @Version    V1.0.0
 * @Date       2021.5
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 */
#include "pid.h"
#include "math.h"
#include "bsp_can.h"
#include "rc_control.h"

float pi=3.1415926535898;
float angle0=0,yaw0=0;
extern uint8_t spin_flag;
extern float InputPower;
float chasisspd;
extern uint8_t SpinStartFlag,IsSpinFlag;

PID_TypeDef PID_GM6020[6];	// 定义GM6020 PID结构体
PID_TypeDef PID_GM6020_speed[2];
PID_TypeDef PID_M2006[3];		// 定义M2006 PID结构体
PID_TypeDef PID_M2006_ANGLE[3];// 定义M2006 角度PID结构体
PID_TypeDef PID_M3508[8];	// 定义M3508 PID结构体 前四个为底盘电机 后两个为摩擦轮电机
PID_TypeDef PID_M3508_Follow;// 定义M3508底盘 PID结构体
PID_TypeDef PID_M3508_Follow1;// 定义M3508底盘 PID结构体
PID_TypeDef PID_Superpower;
PID_TypeDef PID_HEAT_PWM;// 定义陀螺仪加热 PID结构体
void abs_limit(float *a, float ABS_MAX) {
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}

void PID_Reset(PID_TypeDef	*pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}
/**
  * @brief  初始化PID结构体
  * @param  PID结构体指针
    @param  比例系数
		@param  积分系数
		@param  微分系数
		@param  积分最大值
		@param  总输出最大值
  * @retval None
  */
void PID_Init(
    PID_TypeDef*	pid,
    uint32_t 			mode,
    uint32_t 			maxout,
    uint32_t 			intergral_limit,
    float 				kp,
    float 				ki,
    float 				kd
			)
{
    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->pid_mode = mode;

    pid->target[0]=0;
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

}

float PID_Calculate(PID_TypeDef *pid, float target, float feedback)
{
    pid->feedback[NOW] = feedback;
    pid->target[NOW] = target;
    pid->err[NOW] = target - feedback;

    if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err  )
       return 0;
    if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
        return 0;

    if(pid->pid_mode == POSITION_PID)					 //位置式PID
    {
        pid->pout = pid->Kp * pid->err[NOW];
        pid->iout += pid->Ki * pid->err[NOW];
        pid->dout = pid->Kd * (pid->err[NOW] - pid->err[LAST] );

//				if(pid==&PID_GM6020[0])//
//				{
//					if(fabs(pid->err[NOW])>35)
//					{
//							pid->pout = 1.35f*pid->pout;
//							pid->iout = 0;
//							pid->dout = 1.75f*pid->dout;
//					}
//					if(fabs(pid->err[NOW])>10&&fabs(pid->err[NOW])<=35)
//					{
//							pid->pout = 1.2f*pid->pout;
//							pid->iout = 0;
//							pid->dout = 0.5f*pid->dout;
//					}
//					
//					if(fabs(pid->err[NOW])>=5&&fabs(pid->err[NOW])<=10)
//					{
//							pid->pout = pid->pout;//0.75
//							pid->iout = 0.5f*pid->iout;
//							pid->dout = 1.5*pid->dout;
//					}
//					if(fabs(pid->err[NOW])>=0&&fabs(pid->err[NOW])<=5)
//					{
//							pid->pout = 0.75f*pid->pout;
//							pid->iout = pid->iout;
//							pid->dout = 2*pid->dout;
//					}
//				}
				if(pid==&PID_GM6020[1])//
				{
					if(fabs(pid->err[NOW])>35)
					{
							pid->pout = 1.35f*pid->pout;
							pid->iout = 0;
							pid->dout = 1.75f*pid->dout;
					}
					if(fabs(pid->err[NOW])>10&&fabs(pid->err[NOW])<=35)
					{
							pid->pout = 1.2f*pid->pout;
							pid->iout = 0;
							pid->dout = 0.5f*pid->dout;
					}
					
					if(fabs(pid->err[NOW])>=5&&fabs(pid->err[NOW])<=10)
					{
							pid->pout = pid->pout;//0.75
							pid->iout = 0.5f*pid->iout;
							pid->dout = 1.5f*pid->dout;
					}
					if(fabs(pid->err[NOW])>=0&&fabs(pid->err[NOW])<=5)
					{
							pid->pout = 0.75f*pid->pout;
							pid->iout = pid->iout;
							pid->dout = 2*pid->dout;
					}
				}
				
				if(pid==&PID_GM6020[2])//
				{
					if(fabs(pid->err[NOW])>35)
					{
							pid->pout = 1.35f*pid->pout;
							pid->iout = 0;
							pid->dout = 1.75f*pid->dout;
					}
					if(fabs(pid->err[NOW])>15&&fabs(pid->err[NOW])<=35)
					{
							pid->pout = pid->pout;
							pid->iout = 0;
							pid->dout = pid->dout;
					}
					
					if(fabs(pid->err[NOW])>=5&&fabs(pid->err[NOW])<=15)
					{
							pid->pout = 0.85f*pid->pout;//0.75
							pid->iout = 2*pid->iout;
							pid->dout = 2*pid->dout;
					}
					if(fabs(pid->err[NOW])>=0&&fabs(pid->err[NOW])<=5)
					{
							pid->pout = 0.75f*pid->pout;
							pid->iout = 2*pid->iout;
							pid->dout = 5*pid->dout;
					}
				}
					
			if(pid==&PID_GM6020[3])//
				{
					if(fabs(pid->err[NOW])>35)
					{
							pid->pout = 1.35f*pid->pout;
							pid->iout = 0;
							pid->dout = 1.75f*pid->dout;
					}
					if(fabs(pid->err[NOW])>15&&fabs(pid->err[NOW])<=35)
					{
							pid->pout = pid->pout;
							pid->iout = 0;
							pid->dout = pid->dout;
					}
					
					if(fabs(pid->err[NOW])>=5&&fabs(pid->err[NOW])<=15)
					{
							pid->pout = 0.85f*pid->pout;//0.75
							pid->iout = 3*pid->iout;;
							pid->dout = 2*pid->dout;
					}
					if(fabs(pid->err[NOW])>=0&&fabs(pid->err[NOW])<=5)
					{
							pid->pout = 0.75f*pid->pout;
							pid->iout = 5*pid->iout;
							pid->dout = 5*pid->dout;
					}
				}
//					if(pid==&PID_M3508_Follow)
//				{
//					if(fabs(pid->err[NOW])>120)
//					{
//							pid->pout = 1.35f*pid->pout;
//							pid->iout = 0;
//							pid->dout = 1*pid->dout;
//					}
//					if(fabs(pid->err[NOW])>20&&fabs(pid->err[NOW])<=120)
//					{
//							pid->pout = pid->pout;
//							pid->iout = 0;
//							pid->dout = pid->dout;
//					}
//					
//					if(fabs(pid->err[NOW])>=0&&fabs(pid->err[NOW])<=20)
//					{
//							pid->pout = 0.1f*pid->pout;
//							pid->iout = pid->iout;
//							pid->dout = 0.1f*pid->dout;
//					}
//					
//					if(fabs(PID_GM6020[0].err[NOW]) <2)
//					{
//							pid->pout = 0*pid->pout;
//							pid->iout = 0*pid->iout;
//							pid->dout = 0*pid->dout;
//					}
//				}
				
				if(pid==&PID_M2006_ANGLE[2])
				{
					if(fabs(pid->err[NOW])>90)
					{
							pid->pout = 1.35f*pid->pout;
							pid->iout = 0;
							pid->dout = 1*pid->dout;
					}
					if(fabs(pid->err[NOW])>20&&fabs(pid->err[NOW])<=35)
					{
							pid->pout = pid->pout;
							pid->iout = 0;
							pid->dout = pid->dout;
					}
					if(fabs(pid->err[NOW])>10&&fabs(pid->err[NOW])<=20)
					{
							pid->pout = 0.8f*pid->pout;
							pid->iout = 0;
							pid->dout = pid->dout;
					}
					
					if(fabs(pid->err[NOW])>=0&&fabs(pid->err[NOW])<=10)
					{
							pid->pout = 0.5f*pid->pout;
							pid->iout = 1.5f*pid->iout;
							pid->dout = 0.1f*pid->dout;
					}
					
				}
        abs_limit(&(pid->iout), pid->IntegralLimit);				//限制积分输出
        pid->pos_out = pid->pout + pid->iout + pid->dout;		// 计算总输出
        abs_limit(&(pid->pos_out), pid->MaxOutput);					// 限制总输出
        pid->last_pos_out = pid->pos_out;										//更新上一次总输出
    }
    else if(pid->pid_mode == DELTA_PID)					//增量式PID
    {
        pid->pout = pid->Kp * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->Ki * pid->err[NOW];
        pid->dout = pid->Kd * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);
	
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->delta_u = pid->pout + pid->iout + pid->dout;
        pid->delta_out = pid->last_delta_out + pid->delta_u;
        abs_limit(&(pid->delta_out), pid->MaxOutput);
        pid->last_delta_out = pid->delta_out;	//update last time
    }

    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->feedback[LLAST] = pid->feedback[LAST];
    pid->feedback[LAST] = pid->feedback[NOW];
    pid->target[LLAST] = pid->target[LAST];
    pid->target[LAST] = pid->target[NOW];

    return pid->pid_mode==POSITION_PID ? pid->pos_out : pid->delta_out;
}

void GM6020_angle_sum(void)
{
		CAN_GM6020[0].cur_angle=CAN_GM6020[0].angle-CAN_GM6020[0].last_angle;
		CAN_GM6020[0].last_angle=CAN_GM6020[0].angle;
		if(CAN_GM6020[0].cur_angle>4095) CAN_GM6020[0].cur_angle-=8191;
		if(CAN_GM6020[0].cur_angle<-4095) CAN_GM6020[0].cur_angle+=8191;
		CAN_GM6020[0].angle_sum+=CAN_GM6020[0].cur_angle;
}

void M3508_follow(PID_TypeDef *pid_6020,float target,int v[])//底盘跟随
{
    float n=0;
		PID_Calculate(pid_6020,target,CAN_GM6020[0].total_angle);
		 n=pid_6020->pos_out;
//	if(fabs(CAN_GM6020[0].total_angle - target) < 20)
//		n = 0;
		//n = PID_Calculate(&PID_M3508_Follow,target,(int)CAN_GM6020[0].total_angle%8191);

	if(fabs(CAN_GM6020[0].total_angle - target) >200)
	{
		CAN_M3508[0].set_current=PID_Calculate(&PID_M3508[0],n+v[0],CAN_M3508[0].speed);
		CAN_M3508[1].set_current=PID_Calculate(&PID_M3508[1],n+v[1],CAN_M3508[1].speed);
		CAN_M3508[2].set_current=PID_Calculate(&PID_M3508[2],n+v[2],CAN_M3508[2].speed);
		CAN_M3508[3].set_current=PID_Calculate(&PID_M3508[3],n+v[3],CAN_M3508[3].speed);
	}
	else
	{
		CAN_M3508[0].set_current=PID_Calculate(&PID_M3508[0],v[0],CAN_M3508[0].speed);
		CAN_M3508[1].set_current=PID_Calculate(&PID_M3508[1],v[1],CAN_M3508[1].speed);
		CAN_M3508[2].set_current=PID_Calculate(&PID_M3508[2],v[2],CAN_M3508[2].speed);
		CAN_M3508[3].set_current=PID_Calculate(&PID_M3508[3],v[3],CAN_M3508[3].speed);
	}
}
void M3508_follow_2(PID_TypeDef *pid_6020,float target,float angle,float speed,float target2)
{
	  float n=0;
		PID_Calculate(pid_6020,target,CAN_GM6020[0].total_angle);
		n=pid_6020->pos_out;
	  speed=-n+speed;
	  angle=(float)(CAN_GM6020[0].angle-target)/8191*2*pi-angle;
	  if(!IsFollowFlag) n=0;
		if(IsSpinFlag==0)//ctrl//if(!rc.key[5])//ctrl
		{
			
			CAN_M3508[0].set_current=PID_Calculate(&PID_M3508[0],n+cos(angle+pi/4)*speed,CAN_M3508[0].speed);
			CAN_M3508[1].set_current=PID_Calculate(&PID_M3508[1],n-sin(angle+pi/4)*speed,CAN_M3508[1].speed);
			CAN_M3508[2].set_current=PID_Calculate(&PID_M3508[2],n+sin(angle+pi/4)*speed,CAN_M3508[2].speed);
			CAN_M3508[3].set_current=PID_Calculate(&PID_M3508[3],n-cos(angle+pi/4)*speed,CAN_M3508[3].speed);
		}
		else
		{
		  CAN_GM6020[0].total_angle=CAN_GM6020[0].angle;
			CAN_M3508[0].set_current=PID_Calculate(&PID_M3508[0],target2+cos(angle+pi/4)*speed,CAN_M3508[0].speed);
			CAN_M3508[1].set_current=PID_Calculate(&PID_M3508[1],target2-sin(angle+pi/4)*speed,CAN_M3508[1].speed);
			CAN_M3508[2].set_current=PID_Calculate(&PID_M3508[2],target2+sin(angle+pi/4)*speed,CAN_M3508[2].speed);
			CAN_M3508[3].set_current=PID_Calculate(&PID_M3508[3],target2-cos(angle+pi/4)*speed,CAN_M3508[3].speed);
		}
}
void Gyro_mobile_init()//小陀螺移动初始化
{
//  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 10000);
//	HAL_Delay(0);
    angle0=(float)CAN_GM6020[0].angle/8191*2*pi;
    yaw0=imu.yaw;

}
void Gyro_mobile(float angle,float speed,float target,int v[])//小陀螺移动
{
//   float angle1;
//	angle=angle/180*pi;
//    angle1=	angle0;//+imu.yaw-yaw0;
    angle=(float)(CAN_GM6020[0].angle)/8191*2*pi-angle;
    CAN_M3508[0].set_current=PID_Calculate(&PID_M3508[0],cos(angle+pi/4)*speed+target,CAN_M3508[0].speed);
    CAN_M3508[1].set_current=PID_Calculate(&PID_M3508[1],-sin(angle+pi/4)*speed+target,CAN_M3508[1].speed);
    CAN_M3508[2].set_current=PID_Calculate(&PID_M3508[2],sin(angle+pi/4)*speed+target,CAN_M3508[2].speed);
    CAN_M3508[3].set_current=PID_Calculate(&PID_M3508[3],-cos(angle+pi/4)*speed+target,CAN_M3508[3].speed);

//    CAN_M3508_Chassis_SendCurrent();
}

float Control_Power_of_Chasis(float targetpower, float inputpower)
{
	float CHASISSPDMAX = 800;
	float CHASISSPDMIN = -800 ;
	chasisspd = PID_Calculate(&PID_Superpower,targetpower,inputpower);
	if(chasisspd>=CHASISSPDMAX)
		chasisspd = CHASISSPDMAX;
	else if(chasisspd<=CHASISSPDMIN)
		chasisspd = CHASISSPDMIN;
	return chasisspd;
}
