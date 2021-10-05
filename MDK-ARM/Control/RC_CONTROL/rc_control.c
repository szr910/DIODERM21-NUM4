/**
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 * @file       rc_control.c
 * @brief
 * @note
 * @Version    V3.0.0
 * @Date       2021.5
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 */
#include "rc_control.h"
#include "shoot.h"
#include "math.h"
#include "pid.h"

#define RACEFLAG 0 //����ģʽ
#define SPINFLAG 1 //���̸���ģʽ
#define pitch_angle_min	118 
#define pitch_angle_max	227

extern int 	setSpeed[4];
extern VisionData datadata;
extern float pi;
extern float GM6020speed;
extern float imukal_speed;
float Vision_aiming_pitch=0,Vision_aiming_yaw=-5;
uint8_t rc_flag=0;	// ң�������ӱ�־
uint16_t frispd=4350;
uint16_t frispdmax = 4500;
uint16_t Compensatefrispd;
uint8_t  SlowDownflag;//
uint8_t  SpdUpflag;//
int CompensateCounter;


// ң�������Ƶ���
uint8_t rc_f=0,rc_n=1,shootflag=0,shootflag2=1,spin_flag1=0;     //�����жϱ�־

shoot_control_t shoot_control;          //�������
uint8_t Storeflag=0;
uint16_t StoreCounter=0;

uint8_t KeyCounter,KeyCounterTrue,KeyCounterFalse,Key_FLAG;
int M2006_cur=0;

uint8_t M2006flag=1;
float STORETOTALANGLE=0;

float p_flag=0.15;
float p=0,y=0;
float temp_speed;
double imu_last_anglesum;
double imu_angle_bias;
double imu_agbias_sum;
float incpl = 0,incpl_last = 0;

uint16_t FollowInitAngle=7671; //4216
uint8_t QuickReveseFlag=0;
uint16_t QuickReveseTarget=180;
uint16_t QuickReveseCounter=0;
uint8_t Switch_Pid_Flag=0;

float lowpassy;
float lowpassy_last = 0;
float lowpassp;
float lowpassp_last = 0;
extern uint16_t dialspd;
uint16_t tempx;
uint16_t tempy;
extern float vis_pitch,vis_yaw;
uint8_t SpeedUpFlag,IsSpeedUpFlag;//���ٱ�־
uint8_t SpinStartFlag,IsSpinFlag;//С���ݱ�־
uint8_t ShotSwitchflag,ShotFlag;
uint32_t ShotSwitchCounter=1;//�л�ǹ��
uint8_t FollowSwitchFlag,IsFollowFlag=1,LastIsFollowFlag=1;
extern float pit_speed;
uint8_t LevelUpflag,LevelUpCounter=1;


int x0,y0;

void angle_init(void)
{
	imu.angle_sum = 0;
	imu.cur_angle = 0;
	imu.last_angle = 0;
}

void angle_sum(void)
{
	//����������ϴεĲ�ֵ
	imu.cur_angle=imu.yaw - imu.last_angle;
	//����εĽǶ���Ϊ��һ�β�������һ�νǶ�
	imu.last_angle=imu.yaw;
	//��ת����180�㴫���������䵽-180�㣬�������ж���������
	if(imu.cur_angle>180)  
	{
			imu.cur_angle-=360;
	}
	if(imu.cur_angle<-180) 
	{
			imu.cur_angle+=360;
	}
	//���нǶ��ۼӣ�ʹ�Ƕ�����
	imu.angle_sum-=imu.cur_angle;
}

void M2006_Total_Angle(void)
{
	//����������ϴεĲ�ֵ
	 M2006_cur=CAN_M2006[2].angle - CAN_M2006[2].last_angle;
	//����εĽǶ���Ϊ��һ�β�������һ�νǶ�
	CAN_M2006[2].last_angle=CAN_M2006[2].angle ;
	if(M2006flag)
	{
		 M2006flag = 0;
		 M2006_cur = 0;
	}
	//��ת����180�㴫���������䵽-180�㣬�������ж���������
	if(M2006_cur>5000)  
	{
			M2006_cur-=891;
	}
	if(M2006_cur<-5000) 
	{
			M2006_cur+=8191;
	}
	//���нǶ��ۼӣ�ʹ�Ƕ�����
	CAN_M2006[2].total_angle+=M2006_cur;
	STORETOTALANGLE=CAN_M2006[2].total_angle/8191;
}

void M3508_Speed_Angle(void)
{
		CAN_M3508[0].cur_angle = CAN_M3508[0].angle - CAN_M3508[0].last_angle;
		CAN_M3508[0].last_angle = CAN_M3508[0].angle;
		if(CAN_M3508[0].cur_angle>8191)
		{
				CAN_M3508[0].cur_angle-=8191;
		}
		if(CAN_M3508[0].cur_angle<8191)
		{
				CAN_M3508[0].cur_angle+=8191;
		}
}

void RC_PC(float classis_speed,float spin_speed,float dial_speed,float shoot_speed)
{	
  if(rc.sw2==1) {
		        CAN_M3508[4].set_current=0;
            CAN_M3508[5].set_current=0;
						CAN_M3508[6].set_current=0;
            CAN_M3508[7].set_current=0;
            CAN_GM6020[0].set_voltage=0;
            CAN_GM6020[1].set_voltage=0;
						CAN_M2006[0].set_current=0;
						CAN_M2006[1].set_current=0;
						CAN_M2006[2].set_current=0;
		        spin_angle_set=imu.angle_sum;
		        pitch_angle_set=(float)CAN_GM6020[1].angle/8191*360;
						CAN_GM6020[0].total_angle=CAN_GM6020[0].angle;
            velocity(rc.ch2,rc.ch3);
            for(int i=0; i<4; i++) 	//?M3508???
            {
                setSpeed[i] = map(setSpeed[i]+rc.ch0*0.6,-1320,1320,-10000,10000);
                CAN_M3508[i].set_current =  PID_Calculate(&PID_M3508[i], setSpeed[i], CAN_M3508[i].speed);
            }
		}
		else
  {
	imu_angle_bias = imu.angle_sum-imu_last_anglesum;
	if(imu_angle_bias <= 0.1 && imu_angle_bias>=-0.1) imu_agbias_sum-=imu_angle_bias;
	
		pitch_angle_set += map(rc.ch1,-660,660,-2,2);
		if(pitch_angle_set>=pitch_angle_max) pitch_angle_set= pitch_angle_max;
		else if(pitch_angle_set<=pitch_angle_min) pitch_angle_set= pitch_angle_min;
		spin_angle_set -= map(rc.ch0,-660,660,-2,2);
		angle_sum();
		
	imu_angle_bias = imu.angle_sum-imu_last_anglesum;
	if(imu_angle_bias <= 0.1 && imu_angle_bias>=-0.1) imu_agbias_sum+=imu_angle_bias;
	lowpassy = 0.5f*PID_Calculate(&PID_GM6020_speed[0],PID_Calculate(&PID_GM6020[0],spin_angle_set,imu.angle_sum+imu_agbias_sum),imukal_speed)+0.5f*lowpassy_last;
	lowpassy_last = lowpassy;
	lowpassp = 0.5f*PID_Calculate(&PID_GM6020_speed[1],PID_Calculate(&PID_GM6020[1],pitch_angle_set,(float)CAN_GM6020[1].angle/8191*360),(float)pit_speed/8191*360)+0.5f*lowpassp_last;
	lowpassp_last = lowpassp;
  CAN_GM6020[0].set_voltage = lowpassy;
	CAN_GM6020[1].set_voltage = lowpassp;

  	/*************************��Q��С����*******************************/
	if(rc.sw2==2)IsSpinFlag=1;
	else IsSpinFlag=0;
	/*************************�����˶�*******************************/
	float x=map(rc.ch3,-660,660,-3,3);
	float y=map(rc.ch2,-660,660,3,-3);
	if(fabs(x)<0.5)
	tempx = x;
	tempy = y;
	float pix=0;
	if(x>0&&y<0)	pix=pi/4;
	else if(x==0&&y<0)	pix=pi/2;
	else if(x<0&&y<0)	pix=pi*3/4;
	else if(x<0&&y==0)pix=pi;
	else if(x<0&&y>0)pix=pi*5/4;
	else if(x==0&&y>0)pix=pi*3/2;
	else if(x>0&&y>0)pix=pi*7/4;
  M3508_follow_2(&PID_M3508_Follow,FollowInitAngle,pix,sqrt(x*x+y*y)*classis_speed,spin_speed*2);

# if RACEFLAG
		CAN_M3508[4].set_current =  PID_Calculate(&PID_M3508[4], shoot_speed, CAN_M3508[4].speed);
    CAN_M3508[5].set_current =  PID_Calculate(&PID_M3508[5], -shoot_speed, CAN_M3508[5].speed);
		CAN_M3508[6].set_current =  PID_Calculate(&PID_M3508[6], shoot_speed, CAN_M3508[6].speed);
    CAN_M3508[7].set_current =  PID_Calculate(&PID_M3508[7], -shoot_speed,CAN_M3508[7].speed);
# else

	if(rc.sw1!=1){shootflag2=0;}//��CTRL��Ħ����
	else{shootflag2=1;}//��G�ر�Ħ����
	
	if(shootflag2)
	{
		CAN_M3508[4].set_current =  PID_Calculate(&PID_M3508[4], 0, CAN_M3508[4].speed);
    CAN_M3508[5].set_current =  PID_Calculate(&PID_M3508[5], 0, CAN_M3508[5].speed);
		CAN_M3508[6].set_current =  PID_Calculate(&PID_M3508[6], 0, CAN_M3508[6].speed);
    CAN_M3508[7].set_current =  PID_Calculate(&PID_M3508[7], 0, CAN_M3508[7].speed);
	}
	
	else
	{
		CAN_M3508[4].set_current =  PID_Calculate(&PID_M3508[4], shoot_speed, CAN_M3508[4].speed);
    CAN_M3508[5].set_current =  PID_Calculate(&PID_M3508[5], -shoot_speed, CAN_M3508[5].speed);
		CAN_M3508[6].set_current =  PID_Calculate(&PID_M3508[6], shoot_speed, CAN_M3508[6].speed);
    CAN_M3508[7].set_current =  PID_Calculate(&PID_M3508[7], -shoot_speed,CAN_M3508[7].speed);
	}
#endif
		/*************************��f�л�����*******************************/

		ShotFlag = 0;
//	ShotFlag = 2;
	if(rc.sw1==2){
		if(ShotFlag==0)
		{
		CAN_M2006[0].set_current = PID_Calculate(&PID_M2006[0],-dial_speed, CAN_M2006[0].speed);
		CAN_M2006[1].set_current = PID_Calculate(&PID_M2006[1],0, CAN_M2006[1].speed);
		HAL_GPIO_WritePin(PWR24V2_GPIO_Port,PWR24V2_Pin,GPIO_PIN_SET);
		//HAL_GPIO_WritePin(PWR24V3_GPIO_Port,PWR24V3_Pin,GPIO_PIN_SET);
		}
		else if(ShotFlag==1)
		{
		CAN_M2006[0].set_current = PID_Calculate(&PID_M2006[0],0, CAN_M2006[0].speed);
		CAN_M2006[1].set_current = PID_Calculate(&PID_M2006[1],dial_speed, CAN_M2006[1].speed);
		//HAL_GPIO_WritePin(PWR24V2_GPIO_Port,PWR24V2_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(PWR24V3_GPIO_Port,PWR24V3_Pin,GPIO_PIN_SET);
		}
		else
		{
		CAN_M2006[0].set_current = PID_Calculate(&PID_M2006[0],-dial_speed, CAN_M2006[0].speed);
		CAN_M2006[1].set_current = PID_Calculate(&PID_M2006[1],dial_speed, CAN_M2006[1].speed);
		HAL_GPIO_WritePin(PWR24V2_GPIO_Port,PWR24V2_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(PWR24V3_GPIO_Port,PWR24V3_Pin,GPIO_PIN_SET);
		}
	}
	else{
		CAN_M2006[0].set_current = PID_Calculate(&PID_M2006[0],0, CAN_M2006[0].speed);
		CAN_M2006[1].set_current = PID_Calculate(&PID_M2006[1],0, CAN_M2006[1].speed);
		HAL_GPIO_WritePin(PWR24V2_GPIO_Port,PWR24V2_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PWR24V3_GPIO_Port,PWR24V3_Pin,GPIO_PIN_RESET);
	}}
	CAN_Shoot_SendCurrent();
	CAN_Chassis_SendCurrent();
}

void RC_Chassis(void)
{
	
	pitch_angle_set = map(rc.ch1,-660,660,-2,2);
	if(pitch_angle_set>=pitch_angle_max)pitch_angle_set= pitch_angle_max;
  	else if(pitch_angle_set<=pitch_angle_min)pitch_angle_set= pitch_angle_min;
    spin_angle_set -= map(rc.ch0,-660,660,-2,2);
	imu_angle_bias = imu.angle_sum-imu_last_anglesum;
	if(imu_angle_bias <= 0.1 && imu_angle_bias>=-0.1)
	imu_agbias_sum+=imu_angle_bias;
	if(rc.ch0!=0)
		CAN_GM6020[0].set_voltage=PID_Calculate(&PID_GM6020[0],spin_angle_set,imu.angle_sum-imu_agbias_sum);//4.3
	else
		CAN_GM6020[0].set_voltage=PID_Calculate(&PID_GM6020[2],spin_angle_set,imu.angle_sum-imu_agbias_sum);//4.3
	
	CAN_GM6020[1].set_voltage = PID_Calculate(&PID_GM6020[1],pitch_angle_set,(float)CAN_GM6020[1].angle/8191*360);
 
	switch(rc.sw2)
    {
    case 1:
		{
		Gyro_mobile(2,map(rc.ch3,-660,660,-4000,4000),2000,setSpeed);
		spin_flag1 = 1;
		}
        break;
    case 3:
		{
			if(spin_flag1)
			{
				spin_flag1=0;
				CAN_GM6020[0].total_angle = CAN_GM6020[0].angle;
			}
		if(rc.ch2!=0||rc.ch3!=0) 
		{
			velocity(rc.ch2,rc.ch3);
			
			for(int i=0; i<4; i++) 	//��M3508�Ĳ���
			{
				setSpeed[i] = map(setSpeed[i]+rc.ch0*p_flag,-660*(p_flag+1),660*(p_flag+1),-20000,20000);
			//	setSpeed[i] = map(0*setSpeed[i]+rc.ch0,-660,660,-20000,20000);
				CAN_M3508[i].set_current =  PID_Calculate(&PID_M3508[i], setSpeed[i], CAN_M3508[i].speed);
			}
		}
		else	
		{
				for(int i=0; i<4; i++) 	//��M3508�Ĳ���
			{
				setSpeed[i] = 0;
				CAN_M3508[i].set_current =  PID_Calculate(&PID_M3508[i], setSpeed[i], CAN_M3508[i].speed);
			}
		}
		//M3508_follow(&PID_M3508_Follow,FollowInitAngle,setSpeed);
		
		}
      break;
	
    case 2:
		{
				for(int i=0; i<4; i++) 	//��M3508�Ĳ���
				{
						CAN_M3508[i].set_current =  PID_Calculate(&PID_M3508[i],0, CAN_M3508[i].speed);
				}
		}
        break;
    }
		CAN_Chassis_SendCurrent();
    CAN_Shoot_SendCurrent();

}
/*
SW1 = 1	: ������
SW1 = 2 ���������̺�Ħ����
SW1 = 3 ��ֻ����Ħ����
*/
void RC_Shoot(float fri_speed,float dial_speed)
{
	// �������ʱ��С�����趨��ʱ��,�����ת��
		if( shoot_control.block_time < BLOCK_TIME)
    {
        dial_speed = dial_speed;
    }
// �������ʱ����ڵ������趨��ʱ�䣬�����ת��
    else
    {
        dial_speed = -dial_speed;
    }
//����ң�����򿪺�ң���������򿪵�����£���
//	���2006���ٶ�С���趨���ٶȣ���������������
//	ͬʱ������ʱ��С���ҹ涨��ʱ�䣬
//	�򿨵���ʱ������������ӣ���ת��ʱ�����㡣
    if((abs(CAN_M2006[0].speed) < BLOCK_TRIGGER_SPEED ||abs(CAN_M2006[1].speed) < BLOCK_TRIGGER_SPEED )
		&& shoot_control.block_time < BLOCK_TIME&&(rc_flag!=0&&rc.sw1==2))
    {
        shoot_control.block_time++;
        shoot_control.reverse_time = 0;
    }
// ���������ʱ������ҹ涨��ʱ�䣬ͬʱ�����ת��ʱ��С�����趨�ķ�תʱ�䣬��ת��ʱ������������ӡ�
    else if(shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
    {
        shoot_control.reverse_time++;
    }
// ���ң�����رջ�ң���������رգ��򿨵�ʱ��������㡣
		else if(rc_flag==0||rc.sw1==1)//ֹͣ
		{
				shoot_control.block_time = 0;
		}
// ��������¿���ʱ�����Ϊ�㡣
    else
    {
        shoot_control.block_time = 0;
    }
    // ����SW1���÷�������������
    switch(rc.sw1)
    {
//	if(CAN_M2006[0].speed<dial_speed/10)rc.sw1=1;
    case 1:
			  CAN_M3508[4].set_current = PID_Calculate(&PID_M3508[4], 0, CAN_M3508[4].speed);
        CAN_M3508[5].set_current = PID_Calculate(&PID_M3508[5], 0, CAN_M3508[5].speed);
        CAN_M3508[6].set_current = PID_Calculate(&PID_M3508[6], 0, CAN_M3508[6].speed);
				CAN_M3508[7].set_current = PID_Calculate(&PID_M3508[7], 0, CAN_M3508[7].speed);
        CAN_M2006[0].set_current = PID_Calculate(&PID_M2006[0], 0, CAN_M2006[0].speed);
				CAN_M2006[1].set_current = PID_Calculate(&PID_M2006[1], 0, CAN_M2006[1].speed);
        break;
    case 3:
				CAN_M3508[4].set_current = PID_Calculate(&PID_M3508[4], -fri_speed, CAN_M3508[4].speed);
				CAN_M3508[5].set_current = PID_Calculate(&PID_M3508[5], fri_speed, CAN_M3508[5].speed);
        CAN_M3508[6].set_current = PID_Calculate(&PID_M3508[6], -fri_speed, CAN_M3508[6].speed);
				CAN_M3508[7].set_current = PID_Calculate(&PID_M3508[7], fri_speed, CAN_M3508[7].speed);
        CAN_M2006[0].set_current = PID_Calculate(&PID_M2006[0],0, CAN_M2006[0].speed);
				CAN_M2006[1].set_current = PID_Calculate(&PID_M2006[1],0, CAN_M2006[1].speed);

        break;
    case 2:
				CAN_M3508[4].set_current = PID_Calculate(&PID_M3508[4], -fri_speed, CAN_M3508[4].speed);
				CAN_M3508[5].set_current = PID_Calculate(&PID_M3508[5], fri_speed, CAN_M3508[5].speed);
        CAN_M3508[6].set_current = PID_Calculate(&PID_M3508[6], -fri_speed, CAN_M3508[6].speed);
				CAN_M3508[7].set_current = PID_Calculate(&PID_M3508[7], fri_speed, CAN_M3508[7].speed);
        CAN_M2006[0].set_current = PID_Calculate(&PID_M2006[0], -dial_speed, CAN_M2006[0].speed);
				CAN_M2006[1].set_current = PID_Calculate(&PID_M2006[1], -dial_speed, CAN_M2006[1].speed);

        break;
    }

    // ���͵���ֵ�����
    CAN_Shoot_SendCurrent();
}
void RC_Singleshot(float fri_speed,uint8_t pattern)//patternΪ6ʱΪС���裬4ʱΪ����
{

	switch(rc.sw1)
    {
//	if(CAN_M2006.speed<dial_speed/10)rc.sw1=1;
    case 1:
        CAN_M3508[4].set_current = PID_Calculate(&PID_M3508[4], 0, CAN_M3508[4].speed);
        CAN_M3508[5].set_current = PID_Calculate(&PID_M3508[5], 0, CAN_M3508[5].speed);
        CAN_M3508[6].set_current = PID_Calculate(&PID_M3508[6], 0, CAN_M3508[6].speed);
				CAN_M3508[7].set_current = PID_Calculate(&PID_M3508[7], 0, CAN_M3508[7].speed);
        CAN_M2006[0].set_current = PID_Calculate(&PID_M2006[0], 0, CAN_M2006[0].speed);
				CAN_M2006[1].set_current = PID_Calculate(&PID_M2006[1], 0, CAN_M2006[1].speed);
				LASER_Off();
        break;
    case 3:
        CAN_M3508[4].set_current = PID_Calculate(&PID_M3508[4], -fri_speed, CAN_M3508[4].speed);
				CAN_M3508[5].set_current = PID_Calculate(&PID_M3508[5], fri_speed, CAN_M3508[5].speed);
        CAN_M3508[6].set_current = PID_Calculate(&PID_M3508[6], -fri_speed, CAN_M3508[6].speed);
				CAN_M3508[7].set_current = PID_Calculate(&PID_M3508[7], fri_speed, CAN_M3508[7].speed);
        CAN_M2006[0].set_current = PID_Calculate(&PID_M2006[0],0, CAN_M2006[0].speed);
				CAN_M2006[1].set_current = PID_Calculate(&PID_M2006[1],0, CAN_M2006[1].speed);
				LASER_On();
//        if(rc_f==1){Pluck_angle(360/pattern*rc_n);rc_n++;rc_f=0;}
//				else{Pluck_angle(360/pattern*rc_n);}
        break;
    case 2:
        CAN_M3508[4].set_current = PID_Calculate(&PID_M3508[4], -fri_speed, CAN_M3508[4].speed);
				CAN_M3508[5].set_current = PID_Calculate(&PID_M3508[5], fri_speed, CAN_M3508[5].speed);
        CAN_M3508[6].set_current = PID_Calculate(&PID_M3508[6], -fri_speed, CAN_M3508[6].speed);
				CAN_M3508[7].set_current = PID_Calculate(&PID_M3508[7], fri_speed, CAN_M3508[7].speed);
				LASER_On();
//      if(rc_f==0){Pluck_angle(360/pattern*rc_n);rc_n++;rc_f=1;}
//			else{Pluck_angle(360/pattern*rc_n);}
        break;
    }
    // ���͵���ֵ�����
    CAN_Shoot_SendCurrent();
}
void RC_Spin(void)
{
		IMU_Get();
	
		pitch_angle_set += map(rc.ch1,-660,660,-1,1);
		if(pitch_angle_set>=pitch_angle_max)pitch_angle_set= pitch_angle_max;
		else if(pitch_angle_set<=pitch_angle_min)pitch_angle_set= pitch_angle_min;
	
    spin_angle_set -= map(rc.ch0,-660,660,-1,1);
    CAN_GM6020[0].set_voltage = PID_Calculate(&PID_GM6020[0],spin_angle_set,imu.angle_sum);
		if(rc.sw1==1);
//		Gyro_mobile(0,600,2000);
		else
		{
			if(rc.ch1!=0)
			{
				pitch_angle_set = -map(rc.ch1,-660,660,-4000,4000);
				for(int i=0; i<4; i++) 	//��M3508�Ĳ���
				{
					CAN_M3508[i].set_current =  PID_Calculate(&PID_M3508[i], pitch_angle_set, CAN_M3508[i].speed);
				}
			}
			else if(rc.ch2!=0||rc.ch3!=0) {
	//        velocity(rc.ch2,rc.ch3);
			velocity(0,-rc.ch3);
					for(int i=0; i<4; i++) 	//��M3508�Ĳ���
					{
							setSpeed[i] = map(setSpeed[i],-660,660,-20000,20000);
							CAN_M3508[i].set_current =  PID_Calculate(&PID_M3508[i], setSpeed[i], CAN_M3508[i].speed);
					}
			}
		else{	if(1);
//			if(rc.sw1==2)M3508_follow(&PID_M3508_Follow,0);
			else
			{
				CAN_M3508[0].set_current = 0;
				CAN_M3508[1].set_current = 0;
				CAN_M3508[2].set_current = 0;
				CAN_M3508[3].set_current = 0;
			}
		}
	}
		CAN_Chassis_SendCurrent();
	//	M3508_follow(&PID_M3508_Follow,60);
}

void Level_Up_System(void)
{
	switch(GameRobotState.robot_level){
		case 1:
			CAN_Superpower((uint16_t)(80)*100);
			dialspd = 2300;
			Compensatefrispd = frispd + CompensateCounter*50;
			if(Compensatefrispd>=frispdmax) Compensatefrispd = frispdmax;
			if(powerData[1]>18){
					if(IsSpeedUpFlag)
							RC_PC(3500,3500,dialspd,Compensatefrispd);
          else if(powerData[1]>=20.5f)RC_PC(2500,2500,dialspd,Compensatefrispd);
					else RC_PC(2000,2000,dialspd,Compensatefrispd);
			}
			else if(powerData[1]<=18 && powerData[1]>=16)
				IsSpeedUpFlag = 0,RC_PC(1500,500,dialspd,Compensatefrispd);
			else RC_PC(700,500,dialspd,Compensatefrispd);
		  break;
		case 2:
		  break;
		case 3:
		  break;
		default:
			CAN_Superpower((uint16_t)(80)*100);
			dialspd = 2300;
			Compensatefrispd = frispd + CompensateCounter*50;
		
# if Super-Capacitor
			if(Compensatefrispd>=frispdmax) Compensatefrispd = frispdmax;
			if(powerData[1]>18){
				if(IsSpeedUpFlag)
					RC_PC(3500,3500,dialspd,Compensatefrispd);
        else if(powerData[1]>=20.5f)RC_PC(2500,2500,dialspd,Compensatefrispd);
				else RC_PC(2000,2000,dialspd,Compensatefrispd);
			}
			else if(powerData[1]<=18 && powerData[1]>=16)
				IsSpeedUpFlag = 0,RC_PC(1500,500,dialspd,Compensatefrispd);
			else RC_PC(700,500,dialspd,Compensatefrispd);
			}
#else
			RC_PC(2500,2500,dialspd,Compensatefrispd);
#endif
	}
}
	
