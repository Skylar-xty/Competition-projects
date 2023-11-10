/**********************718创新实验室开发板例程*********************
*  编写：718创新实验室
*  平台：STM32F103RCT6
*  说明：由于作者的水平有限，若有不足之处，还请大家谅解。
*		 建议大家多看看数据手册。     
******************************************************************/


/*************************功能说明********************************
空白工程，需要什么功能自己添加
*************************功能说明********************************/

#include "stm32f10x.h"
#include "bsp_delay.h"
#include "bsp_servo.h"
#include "bsp_usart.h"
#include "bsp_adc.h"
#include "bsp_pwm.h"
#include "bsp_track.h"
#include "bsp_control.h"
#include "bsp_usart_blt.h"

extern __IO uint16_t ADC_ConvertedValue;
vu16 status=0;

//// 局部变量，用于保存转换计算后的电压值 	 
float ADC_ConvertedValueLocal;        

//// 软件延时
void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
} 
char temp[1500];
vu16 cnt=0;
int main()
{
	
	 DEBUG_UART_Config();
	//NVIC_Configuration();
	
//	// 配置串口
//	USART_Config();
//	
//	// ADC 初始化
//	ADCx_Init();
//	
//	printf("\r\n ----这是一个ADC单通道中断读取实验----\r\n");
//	
//	while (1)
//	{
//		ADC_ConvertedValueLocal =(float) ADC_ConvertedValue/4096*3.3; 
////		ADC_ConvertedValueLocal =(float) Get_Adc(1)/4096*3.3; 	//Verified OK
//	
//		printf("\r\n The current AD value = 0x%04X \r\n", 
//		       ADC_ConvertedValue); 
//		printf("\r\n The current AD value = %f V \r\n",
//		       ADC_ConvertedValueLocal); 
//		printf("\r\n\r\n");

//		delay_nms(200);  
//	}
	
	
	
	
	
	
	
	
	
//	// 配置串口
//	USART_Config();
//	
//	// ADC 初始化
//	Adc_Init();
//	
//	printf("\r\n ----这是一个ADC单通道中断读取实验----\r\n");
//	
//	while(1)
//	{
//		double temp;
//		temp = GetTraceDate();
//		printf("\r\n trace feedback = %f \r\n",temp);
//		GetParament();
//		delay_nms(200);
//	}
	
	
	
	
	
	
//		ADVANCE_TIM_Init();
//		
//		//Servor_Init();
//		//Servor_Test();
//		
//		vu16 cnt;
//		
//		while(1)
//		{
//			for(cnt=0;cnt<=180;cnt+=10)
//			{
//				SetSteerAngle1(cnt);
//				SetSteerAngle2(cnt);
//				SetSteerAngle3(cnt);
//				SetSteerAngle4(cnt);
//				delay_nms(200);
//			}
//			for(cnt=170;cnt>0;cnt-=10)
//			{
//				SetSteerAngle1(cnt);
//				SetSteerAngle2(cnt);
//				SetSteerAngle3(cnt);
//				SetSteerAngle4(cnt);
//				delay_nms(200);
//			}
//	
//		}
	
	
	
		
//	short int kcnt=1;
//	SystemInit();
//	GPIO_Tim3PWM(2);
//	TIM3PinReMap(1);
//	TIM_Init(TIM3,10000,72);
//	GPIO_IN1IN2();
//	TIM_PWMMode(TIM3,2,1,kcnt);
//	while(1)
//	{
//		MotorRightLeft(1);	//clockwise
//		for(kcnt=10000;kcnt>=0;kcnt=kcnt-1000)
//		{
//			TIM_SetCompare2(TIM3,kcnt);
//			delay_nms(2000);
//		}
//		MotorRightLeft(0);	//counter-clockwise
//		for(kcnt=10000;kcnt>=0;kcnt=kcnt-1000)
//		{
//			TIM_SetCompare2(TIM3,kcnt);
//			delay_nms(2000);
//		}
//	}
		
		

		
//	ADVANCE_TIM_Init();
//		
//	short int kcnt=1;
//	SystemInit();
//	GPIO_IN1IN2_CH1();
//	GPIO_IN1IN2_CH2();
//	while(1)
//	{
//		MotorRightLeft_CH1(1);	//clockwise
//		MotorRightLeft_CH2(1);	//clockwise
//		for(kcnt=10000;kcnt>=0;kcnt=kcnt-1000)
//		{
//			TIM_SetCompare1(TIM1,kcnt);
//			TIM_SetCompare2(TIM1,kcnt);
//			delay_nms(2000);
//		}
//		MotorRightLeft_CH1(0);	//counter-clockwise
//		MotorRightLeft_CH2(0);	//counter-clockwise
//		for(kcnt=10000;kcnt>=0;kcnt=kcnt-1000)
//		{
//			TIM_SetCompare1(TIM1,kcnt);
//			TIM_SetCompare2(TIM1,kcnt);
//			delay_nms(2000);
//		}
//	}


/////////////////////////////test//////////////////////////
//		SystemInit();
//		MotorInit();
//		while(1)
//		{
//		RightTurn();
//		delay_nms(600);
//		LeftTurn();
//		delay_nms(600);
//		}


	
	
	
	
	
//		// 配置串口
//	USART_Config();
//	
//		SystemInit();
//		MotorInit();
//	
//	// ADC 初始化
//	Adc_Init();
//	Foward();
	
	
	
	
	status =0;
			// 配置串口
	USART_Config();
	
		SystemInit();
		MotorInit();
	
	// ADC 初始化
	Adc_Init();
	SetSteerAngle3(0);
	while(1)
	{
		Forward();
		TIM_SetCompare1(TIM1,0); //13-right
		TIM_SetCompare2(TIM1,0); //11-left  (12-Mid)
	}
	
	
	
	
	
	
	
	
	
//		// 配置串口
//	USART_Config();
//	
//	// ADC 初始化
//	Adc_Init();
//	
//	printf("\r\n ----这是一个ADC单通道中断读取实验----\r\n");
//	
//	while(1)
//	{
//		printf("\r\n LEFT LEFT = %f \r\n\r\n LEFT = %f \r\n\r\n MID = %f \r\n\r\n Right = %f \r\n\r\n RIGHT RIGHT= %f \r\n\r\n Front = %f \r\n",Get_Adc(7)/1.0,Get_Adc(11)/1.0,Get_Adc(12)/1.0,Get_Adc(13)/1.0,Get_Adc(2)/1.0,Get_Adc(10)/1.0);
////		GetParament();
//		delay_nms(200);
//	}
	
}
