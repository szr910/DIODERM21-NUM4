/**
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 * @file       gimbal.c
 * @brief     
 * @note
 * @Version    V1.0.0
 * @Date       2021.5
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 */
#include "gimbal.h"
#include "kalman.h"
float spin_angle_set = 0.1,pitch_angle_set=55,gyro_angle_set=0;

uint8_t Vision_receiving = 0;
uint8_t Vision_buffer[10] = {0}; 
uint8_t Vision_buffercnt = 0;
uint8_t tmp_vision;
VisionData datadata;
extern PID_TypeDef PID_GM6020[];

extKalman_t vis_data_pit;
extKalman_t vis_data_yaw;
float vis_pitch,vis_yaw;


void VisionUartRxCpltCallback()
{
	if(Vision_receiving){
		Vision_buffer[Vision_buffercnt] = tmp_vision;
		Vision_buffercnt++;
		if(tmp_vision == 0x65){
			if(Vision_buffer[1]>>7)
				datadata.yaw_angle=	-(100-(((Vision_buffer[1]<<8)| Vision_buffer[2])&0x7fff)*0.003051850947599);
			else
				datadata.yaw_angle=	(((Vision_buffer[1]<<8)| Vision_buffer[2])&0x7fff)*0.003051850947599;
			if(Vision_buffer[3]>>7)
				datadata.pitch_angle=  	-(100-(((Vision_buffer[3]<<8)| Vision_buffer[4])&0x7fff)*0.003051850947599);
			else
				datadata.pitch_angle=		(((Vision_buffer[3]<<8)| Vision_buffer[4])&0x7fff)*0.003051850947599;
			if(Vision_buffer[5]>>7)
			datadata.dis=	   			-(100-(((Vision_buffer[5]<<8)| Vision_buffer[6])&0x7fff)*0.003051850947599);
			else
				datadata.dis=	   		(((Vision_buffer[5]<<8)| Vision_buffer[6])&0x7fff)*0.003051850947599;
			Vision_buffercnt = 0,Vision_receiving=0;
			
//			printf("%f %f %f \n",datadata.pitch_angle,datadata.yaw_angle,datadata.dis);
			
		}
	}
	else{
		if(tmp_vision == 0x73){

			Vision_receiving = 1;
			Vision_buffercnt = 0;
			Vision_buffer[0] = tmp_vision;
			Vision_buffercnt++;
		}
	}
	//	HAL_UART_Receive_IT(&huart6, &tmp_vision, 1);
	if(HAL_UART_Receive_DMA(&huart6, &tmp_vision, 1) != HAL_OK){
		Error_Handler();
	}
}

void Pluck_angle(int num,float angle)
{
		float n;
    n=PID_Calculate(&PID_M2006_ANGLE[0],-angle,CAN_M2006[num].total_angle);
		CAN_M2006[num].set_current = PID_Calculate(&PID_M2006[0],n,CAN_M2006[num].speed);
//	CAN_M2006.total_angle=0;
}
