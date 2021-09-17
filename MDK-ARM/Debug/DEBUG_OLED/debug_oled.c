/**
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 * @file       debug_oled.c
 * @brief      
 * @note       
 * @Version    V1.0.0
 * @Date       2021.5      
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 */
#include "debug_oled.h"

extern CAN_GM6020_TypeDef 	CAN_GM6020[];	// 引用GM6020结构体
extern CAN_M2006_TypeDef	CAN_M2006[];		// 引用M2006结构体
extern CAN_M3508_TypeDef	CAN_M3508[];	// 引用M3508结构体

extern PID_TypeDef PID_GM6020[];			// 引用GM6020 PID结构体
extern PID_TypeDef PID_M2006[];				// 引用M2006 PID结构体
extern PID_TypeDef PID_M3508[];				// 引用M3508 PID结构体

extern rc_info_t	rc;						// 遥控器接收
extern imu_t 		imu;					// 引用陀螺仪
extern int 			setSpeed[4];			// 引用底盘四个电机速度
extern float 		powerData[4];			// 引用超级电容的数据

//void OLED_ShowM2006(){
//    OLED_Printf(0,1,"%d",CAN_M2006.current);
//    OLED_Printf(1,1,"%d",CAN_M2006.speed);
//    OLED_Printf(2,1,"%d",CAN_M2006.angle);
//    OLED_RefreshGram();
//}

void OLED_ShowImu(){
    IMU_Get();
    OLED_ShowFloat(0,1,imu.rol,0,5,3);
    OLED_ShowFloat(1,1,imu.pit,0,5,3);
    OLED_ShowFloat(2,1,imu.yaw,0,5,3);
    OLED_RefreshGram();
}

void OLED_ShowRC(){
	OLED_Printf(0,1,"ch1:%d",rc.ch0);
	OLED_Printf(1,1,"ch2:%d",rc.ch1);
	OLED_Printf(2,1,"ch3:%d",rc.ch2);
	OLED_Printf(3,1,"ch4:%d",rc.ch3);
	
    OLED_RefreshGram();
}

void OLED_ShowSpeed(){
	OLED_Clear(Pen_Write);
	OLED_Printf(0,1,"speed1:%d",setSpeed[0]);
	OLED_Printf(1,1,"speed2:%d",setSpeed[1]);
	OLED_Printf(2,1,"speed3:%d",setSpeed[2]);
	OLED_Printf(3,1,"speed4:%d",setSpeed[3]);
	
    OLED_RefreshGram();
}

void OLED_ShowChassisSetCurrent(){
	OLED_Clear(Pen_Write);
	OLED_Printf(0,1,"current1:%d",CAN_M3508[0].set_current);
	OLED_Printf(1,1,"current2:%d",CAN_M3508[1].set_current);
	OLED_Printf(2,1,"current3:%d",CAN_M3508[2].set_current);
	OLED_Printf(3,1,"current4:%d",CAN_M3508[3].set_current);
	
    OLED_RefreshGram();
}

void OLED_ShowChassisFwCurrent(){
	OLED_Clear(Pen_Write);
	OLED_Printf(0,1,"FwCurrent1:%d",CAN_M3508[0].current);
	OLED_Printf(1,1,"FwCurrent2:%d",CAN_M3508[1].current);
	OLED_Printf(2,1,"FwCurrent3:%d",CAN_M3508[2].current);
	OLED_Printf(3,1,"FwCurrent4:%d",CAN_M3508[3].current);
	
    OLED_RefreshGram();
}

void  OLED_ShowSuperCapacity(){
	OLED_Clear (Pen_Write);
	OLED_Printf(0,1,"In Vol:%f",powerData[0]);
	OLED_Printf(1,1,"Cap Vol:%f",powerData[1]);
	OLED_Printf(2,1,"In Cur:%f",powerData[2]);
	OLED_Printf(3,1,"Set Pow:%f",powerData[3]);
	
	OLED_RefreshGram();
}

