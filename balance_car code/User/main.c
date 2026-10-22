#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "LED.h"
#include "Timer.h"
#include "Key.h"
#include "MPU6050.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include "BlueSerial.h"
#include <math.h>
#include "PID.h"
#include <string.h>
#include <stdlib.h>//字符串转整型：atoi，转浮点型：atof

void BT_Parse(void);//蓝牙串口数据解析

/*定义全局变量*/
int16_t AX, AY, AZ, GX, GY, GZ;		//读取MPU6050的原始数据
uint8_t TimerErrorFlag;	//定时器错误标志位，如果定时中断函数执行时间超过了定时时间，则此标志位置1
uint16_t TimerCount;	//定时器计数值，此值可用于计算定时中断函数具体的执行时间

float AngleAcc;			//由加速度计得到的角度值
float AngleGyro;		//由陀螺仪得到的角度值，执行互补滤波后，此值基本与Angle相等
float GyroY_Actual; //陀螺仪Y轴角速度
float Angle;			//互补滤波后的角度值，准确且无漂移

uint8_t KeyNum,RunFlag;

int16_t LeftPWM,RightPWM;
int16_t AvePWM,DifPWM;

float LeftSpeed,RightSpeed;
float AveSpeed,DifSpeed;

PID_t AnglePID={
    .Kp=4.42,   //4.42  0.08 3.25
	  .Ki=0.08,
	  .Kd=3.25,
	
	  .OutMax=100,
	  .OutMin=-100,
	
	.OutOffset=0.0,//输出偏移
	.ErrorIntMax=600,.ErrorIntMin=-600,//积分限幅
};

/*速度环变量*/
PID_t SpeedPID=
{
	.Kp=2.8,.Ki=0.1,.Kd=0.0,//PID值
	.OutMax=20,.OutMin=-20,//输出限幅
	.ErrorIntMax=150,.ErrorIntMin=-150,//积分限幅
};

/*转向环变量*/
PID_t TurnPID=
{
	.Kp=0.0,.Ki=2.6,.Kd=0.0,//PID值
	.OutMax=50,.OutMin=-50,//输出限幅
	.ErrorIntMax=20,.ErrorIntMin=-20,//积分限幅
};

/*角速度环变量*/
PID_t GyroPID=
{
	.Kp=0.1,.Ki=0.0,.Kd=0.1,//PID值  0.1 0 0.1
	.OutMax=5,.OutMin=-5,//输出限幅
};

int main(void)
{
	/*模块初始化*/
	OLED_Init();		//OLED初始化
	MPU6050_Init();		//MPU6050初始化
	BlueSerial_Init();	//蓝牙串口初始化
	LED_Init();
	Key_Init();
	Motor_Init();
	Encoder_Init();
	Serial_Init();
	Timer_Init();		//定时器初始化，定时中断时间1ms
	
	while (1)
	{
		if(RunFlag)
    {LED_ON();}
		else
		{LED_OFF();}
		
		KeyNum = Key_GetNum();		//获取键码
		if (KeyNum == 1)			//如果K1按下
		{
			/*RunFlag变量取非*/
			if (RunFlag == 0)		//如果RunFlag为0
			{
				PID_Init(&AnglePID);//启动程序时给PID初始化，清除之前可能遗留的参数
				PID_Init(&SpeedPID);
				PID_Init(&TurnPID);
			  PID_Init(&GyroPID);//角速度环PID初始化
				
				RunFlag = 1;		//则RunFlag置1
			}
			else					//否则，RunFlag非0
			{
				RunFlag = 0;		//则RunFlag置0
			}
		}
		
		/*OLED显示*/
		OLED_Clear();
		OLED_Printf(0, 0, OLED_6X8, "  Angle");						//OLED左侧显示角度环参数
		OLED_Printf(0, 8, OLED_6X8, "P:%05.2f", AnglePID.Kp);		//角度环Kp
		OLED_Printf(0, 16, OLED_6X8, "I:%05.2f", AnglePID.Ki);		//角度环Ki
		OLED_Printf(0, 24, OLED_6X8, "D:%05.2f", AnglePID.Kd);		//角度环Kd
		OLED_Printf(0, 32, OLED_6X8, "T:%+05.1f", AnglePID.Target);	//角度环目标值
		OLED_Printf(0, 40, OLED_6X8, "A:%+05.1f", Angle);			//角度环实际值
		OLED_Printf(0, 48, OLED_6X8, "O:%+05.0f", AnglePID.Out);	//角度环输出值
		OLED_Printf(0, 56, OLED_6X8, "GY:%+05d", GY);				//显示GY，便于校准陀螺仪零漂
		OLED_Update();				//OLED更新
		
		/*蓝牙串口打印波形，需配合蓝牙串口小程序实现波形绘制*/
		//BlueSerial_Printf("[plot,%f,%f,%f]", AngleAcc, AngleGyro, Angle);	//绘制AngleAcc、AngleGyro和Angle的波形
		// 原来：BlueSerial_Printf(...);
     Serial_Printf("%.3f,%.3f,%.3f,%d,%d,%d\r\n",SpeedPID.Target,AveSpeed,Angle,GX,GY,GZ);
	}
}

void TIM1_UP_IRQHandler(void)
{
	static uint16_t Count0,Count1;
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		/*定时中断函数1ms自动执行一次*/
		
		/*进入中断函数后，立刻清标志位*/
		/*如果中断函数退出前，标志位又置1了，说明中断函数执行时间超过了定时时间（1ms）*/
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		
		Key_Tick();
		
		Count0 ++;
		if(Count0 >=10)
		{
		   Count0 = 0;
			
		
		
		/*在中断里读取MPU6050，可以保证读取间隔严格为1ms*/
		/*但要保证MPU6050_GetData执行时间不超过1ms*/
		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
		
		/*校准陀螺仪X轴零漂*/
		/*此值需实测确定，不同的设备零漂一般不同*/
		/*实测方法是，在完全静止时，观察OLED显示的GX值，即为零漂值*/
		/*然后在此处将零漂值减去，使得完全静止时，GY值为0*/
		GY -=3;		
		/*由加速度计计算得到角度值*/
		/*atan2计算反正切，得到角度（弧度制）， / 3.14159 * 180可将弧度制转为角度值*/
		AngleAcc = -atan2(AX, AZ) / 3.14159 * 180;
			
		AngleAcc-=5.0; /*减去零漂值*/
		
		/*由陀螺仪积分得到角度值*/
		/*互补滤波下，角度积分要在上次滤波后的Angle上进行*/
		/*公式中32768是int16_t变量的最大值，2000是陀螺仪配置的满量程2000度每秒，0.001是定时时间1ms*/
		AngleGyro = Angle + GY / 32768.0 * 2000 * 0.01;
		
		/*Y轴角速度：角速度计*/
			GyroY_Actual=GY/32768.0*2000;
		
		/*执行互补滤波*/
		float Alpha = 0.01;		//互补滤波系数，值越大，越偏向于加速度计，值越小，越偏向于陀螺仪
		Angle = Alpha * AngleAcc + (1 - Alpha) * AngleGyro;		//互补滤波计算，得到稳定的角度值
		
		if(Angle>50||Angle <-50)
		{
			RunFlag=0;
		}
		if(RunFlag)
		{
			/*角度环调控*/
			AnglePID.Actual=Angle;
			PID_Update(&AnglePID);
			
			/*角速度环调控*/
			GyroPID.Target=AnglePID.Out;
			GyroPID.Actual=GyroY_Actual;
			PID_Update(&GyroPID);
			
			AvePWM=-(AnglePID.Out+GyroPID.Out);
			
			LeftPWM=AvePWM+DifPWM/2;
			RightPWM=AvePWM-DifPWM/2;
			
			if (LeftPWM > 100) {LeftPWM = 100;} else if (LeftPWM < -100) {LeftPWM = -100;}
				if (RightPWM > 100) {RightPWM = 100;} else if (RightPWM < -100) {RightPWM = -100;}
				
				Motor_SetPWM(1, LeftPWM);		//LeftPWM输出给左轮电机
				Motor_SetPWM(2, RightPWM);		//RightPWM输出给右轮电机
		}
		else 
		{
			Motor_SetPWM(1,0);
			Motor_SetPWM(2,0);
		}
	  }
		Count1++;
		if(Count1>=50)
		{
			Count1 = 0;
			
			LeftSpeed=Encoder_Get(1)/44.0/0.05/9.27666;
			RightSpeed=Encoder_Get(2)/44.0/0.05/9.27666;
			
			AveSpeed=(LeftSpeed+RightSpeed)/2.0;
			DifSpeed=LeftSpeed-RightSpeed;
			
			if (RunFlag)
			{
				SpeedPID.Actual=AveSpeed;
				PID_Update(&SpeedPID);
				AnglePID.Target=SpeedPID.Out;
				
				
				TurnPID.Actual=DifSpeed;
				PID_Update(&TurnPID);
				TurnPID.Target=TurnPID.Out;
		}
	}
		
		/*中断函数退出前，再次检查标志位*/
		if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
		{
			/*标志位又置1了，说明中断函数执行时间超过了定时时间（1ms）*/
			/*置TimerErrorFlag为1，表示定时中断错误*/
			TimerErrorFlag = 1;
			
			/*清标志位，避免中断连续触发，导致主函数完全无法执行*/
			TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		}
		
		/*中断函数退出前，读取计数器的值，此值可用于测量中断函数的具体执行时间*/
		TimerCount = TIM_GetCounter(TIM1);
	}
}

/*蓝牙串口数据解析函数*/
void BT_Parse()
{
	// 判断串口是否收到一包新数据（接收完成标志置1）
	if (Serial_RxFlag==1)
	{
		/*字符串分割:第一次调用strtok，以逗号作为分隔符拆分接收缓冲区字符串*/
		char *string1=strtok (Serial_RxPacket,",");
		
		/*字符串判断+拆分：按键功能（自定义协议格式）*/
		if (strcmp (string1,"key")==0) // 判断数据包类型为按键数据包
		{
			char *string2=strtok (NULL,","); // 继续取下一段分割字符串
			char *string3=strtok (NULL,","); // 取下第三段分割字符串
			if (strcmp (string2,"1")==0) // 判断目标按键是按键1
			{
				if (strcmp (string3,"ON/OFF")==0) // 功能：平衡车启停控制
				{
					if (RunFlag==0) // 当前为停止状态，准备启动平衡车
					{
						PID_Init(&AnglePID);  // 初始化角度环PID，清空积分等状态
						PID_Init(&SpeedPID);  // 初始化速度环PID
						PID_Init(&TurnPID);   // 初始化转向环PID
						RunFlag=1;            // 置运行标志，平衡车开始进入控制
					}
					else // 当前已经在运行，按下则停止
					{
						RunFlag=0;            // 清除运行标志，平衡车关闭控制
					}
				}
			}
		}
		
		/*字符串判断：滑块功能（固定协议格式）*/
		if (strcmp (string1,"slider")==0) // 判断数据包类型为滑块调参数据包
		{
			char *string2=strtok (NULL,","); // 读取参数名称
			char *string3=strtok (NULL,","); // 读取参数数值字符串
			if (strcmp (string2,"AngleKp")==0) // 目标滑块：角度环Kp
			{
				AnglePID.Kp=atof(string3); // 字符串转浮点数，赋值给角度环Kp
			}
			else if (strcmp (string2,"AngleKi")==0) // 目标滑块：角度环Ki
			{
				AnglePID.Ki=atof(string3);
			}
			else if (strcmp (string2,"AngleKd")==0) // 目标滑块：角度环Kd
			{
				AnglePID.Kd=atof(string3);
			}
			else if (strcmp (string2,"GyroKp")==0) // 目标滑块：陀螺仪环Kp
			{
				GyroPID.Kp=atof(string3);
			}
			else if (strcmp (string2,"GyroKi")==0) // 目标滑块：陀螺仪环Ki
			{
				GyroPID.Ki=atof(string3);
			}
			else if (strcmp (string2,"GyroKd")==0) // 目标滑块：陀螺仪环Kd
			{
				GyroPID.Kd=atof(string3);
			}
			else if (strcmp (string2,"SpeedKp")==0) // 目标滑块：速度环Kp
			{
				SpeedPID.Kp=atof(string3);
			}
			else if (strcmp (string2,"SpeedKi")==0) // 目标滑块：速度环Ki
			{
				SpeedPID.Ki=atof(string3);
			}
			else if (strcmp (string2,"SpeedKd")==0) // 目标滑块：速度环Kd
			{
				SpeedPID.Kd=atof(string3);
			}
			else if (strcmp (string2,"TurnKp")==0) // 目标滑块：转向环Kp
			{
				TurnPID.Kp=atof(string3);
			}
			else if (strcmp (string2,"TurnKi")==0) // 目标滑块：转向环Ki
			{
				TurnPID.Ki=atof(string3);
			}
			else if (strcmp (string2,"TurnKd")==0) // 目标滑块：转向环Kd
			{
				TurnPID.Kd=atof(string3);
			}
			else if (strcmp (string2,"Offset")==0) // 目标滑块：角度偏移量（直立补偿）
			{
				AnglePID.OutOffset=atof(string3);
			}
		}
		
		/*字符串判断：摇杆功能（固定协议格式）*/
		if (strcmp (string1,"joystick")==0) // 判断数据包类型为摇杆数据包
		{
			int8_t LH=atoi(strtok (NULL,",")); // 左摇杆水平值
			int8_t LV=atoi(strtok (NULL,",")); // 左摇杆竖直值（用来控制前进后退速度）
			int8_t RH=atoi(strtok (NULL,",")); // 右摇杆水平值（用来控制转向）
			int8_t RV=atoi(strtok (NULL,",")); // 右摇杆竖直值，本工程暂时没有使用
			
			SpeedPID.Target=LV/50.0; // 左摇杆竖直分量缩放，作为速度环目标速度
			TurnPID.Target=RH/50.0;	// 右摇杆水平分量缩放，作为转向环目标
		}
		Serial_RxFlag=0; // 清接收标志，表示这一包数据解析完成，等待下一包
	}
}
