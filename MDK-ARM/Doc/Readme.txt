工程名字：	
			Infantry_Num4
			
工程版本：	
			V4.0
			
工程时间：	2021.5.16
			
工程作者：	
			DIODE战队 控制组
				
工程版本：   
			V4.0:
					1、重新命名工程
			V2.1:
					1、这是第一次步 兵车的规范代码，之后所有的结构都按照这份代码的标准来，比如PID结构体的定义，电机结构体的定义等
					2、整合目前底盘遥控和小陀螺、底盘跟随、旋转移动的功能。但是未完全整合在一起，待考试之后将它们完美整合
					3、添加USMART调试组件
					4、添加虚拟示波器调试组件
					
			v1.4:
					1、【验证RC】遥控器没有问题
					2、【验证底盘电机】可以用遥控器遥控
					3、【beep】增加控制的蜂鸣器函数，在程序初始化之后让蜂鸣器鸣叫500ms
						
			v1.3:
					1、【shoot.c】增加摩擦轮控制函数
							Friction_SetSpeed(speed_l,speed_s);		// 设置摩擦轮的速度
					2、【debug.c】增加OLED调试函数
							OLED_ShowM2006(void);
							OLED_ShowImu(void);
					3、这是底盘和云台电调理论版本，预计在1.4版本中，将本次版本做的改变实现
					
			v1.2:
					1、增加GM6020电机和舵机
					2、增加CAN配置，增加bus
					3、使用tim1
			v1.1:
					1、增加OLED_ShowFloat(uint8_t row, uint8_t col, float num, uint8_t mode, uint8_t len1,uint8_t len2);
						支持显示浮点数
					2、增加陀螺仪配置
					3、增加CAN配置，由C620电调手册给出，can的波特率为1Mbps
						实现电调电流的设置，电调参数的读取，
						（但是发现给定电流后，速度在一段时间之后会持续增大，所以后续版本需要速度闭环）
					4、删除USB虚拟串口，因为发现USB设备会与CAN发生冲突，两者不能同时使用，考虑到可以用OLED查看数据，就不用USB虚拟串口了
			
			V1.0：
					1、OLED显示
					2、五轴按键响应
					3、ADC任意通道单次采样
					4、freeRTOS
							default任务完成调试工作
							LEDTwinkle任务实现指示灯的变化，反应任务进行的状态
					5、USB虚拟串口
			