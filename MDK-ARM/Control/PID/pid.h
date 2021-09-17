#ifndef _PID_H
#define _PID_H
#include "main.h"

#define ABS(x)		(((x)>0)? (x): -(x))
enum {
    LLAST	= 0,
    LAST 	= 1,
    NOW 	= 2,

    POSITION_PID,
    DELTA_PID,
};
typedef struct _PID_Typedef
{
    float Kp;
    float Ki;
    float Kd;

    float target[3];					//Ŀ��ֵ,����NOW�� LAST�� LLAST���ϴ�
    float feedback[3];					//����ֵ
    float err[3];							//���

    float pout;								//p���
    float iout;								//i���
    float dout;								//d���

    float pos_out;						//����λ��ʽ���
    float last_pos_out;					//�ϴ�λ��ʽ���
    float pos_out0;						//λ��ʽ�����Сֵ

		float lastdev;            //ǰһ��ʱ��΢����ֵ
		float alpha;							//����ȫ΢��ϵ��
		float lastdeltadev;      //ǰһ��ʱ��΢��������
	
    float delta_u;						//��������ֵ
    float last_delta_out;				//�ϴ�����ʽ���
    float delta_out;					//��������ʽ��� = last_delta_out + delta_u

    float max_err;
    float deadband;						//err < deadband return

    uint32_t pid_mode;
    int MaxOutput;				//����޷�
    uint32_t IntegralLimit;		    //�����޷�
} PID_TypeDef;

extern PID_TypeDef PID_GM6020[6];	// ����GM6020 PID�ṹ��
extern PID_TypeDef PID_GM6020_speed[2];
extern PID_TypeDef PID_M2006[3];		// ����M2006 PID�ṹ��
extern PID_TypeDef PID_M2006_ANGLE[3];	// ����M2006 �Ƕ�PID�ṹ��
extern PID_TypeDef PID_M3508[8];	// ����M3508 PID�ṹ��
extern PID_TypeDef PID_M3508_Follow;
extern PID_TypeDef PID_M3508_Follow1;
extern PID_TypeDef PID_HEAT_PWM;
extern PID_TypeDef PID_Superpower;

void PID_Init(
    PID_TypeDef*	pid,
    uint32_t 			mode,
    uint32_t 			maxout,
    uint32_t 			intergral_limit,
    float 				kp,
    float 				ki,
    float 				kd
);
void PID_Reset(PID_TypeDef	*pid, float kp, float ki, float kd);

float PID_Calculate(PID_TypeDef *pid, float target, float feedback);
void M3508_follow(PID_TypeDef *pid_6020,float target,int v[]);
void Gyro_mobile_init(void);
void Gyro_mobile(float angle,float speed,float target,int v[]);
void GM6020_angle_sum(void);
void M3508_follow_2(PID_TypeDef *pid_6020,float target,float angle,float speed,float target2);
float Control_Power_of_Chasis(float targetpower, float inputpower);
#endif



