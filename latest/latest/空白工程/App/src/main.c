/**********************718����ʵ���ҿ���������*********************
*  ��д��718����ʵ����
*  ƽ̨��STM32F103RCT6
*  ˵�����������ߵ�ˮƽ���ޣ����в���֮�����������½⡣
*		 �����Ҷ࿴�������ֲᡣ     
******************************************************************/


/*************************����˵��********************************
�հ׹��̣���Ҫʲô�����Լ����
*************************����˵��********************************/

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

//// �ֲ����������ڱ���ת�������ĵ�ѹֵ 	 
float ADC_ConvertedValueLocal;        

//// �����ʱ
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
	
//	// ���ô���
//	USART_Config();
//	
//	// ADC ��ʼ��
//	ADCx_Init();
//	
//	printf("\r\n ----����һ��ADC��ͨ���ж϶�ȡʵ��----\r\n");
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
	
	
	
	
	
	
	
	
	
//	// ���ô���
//	USART_Config();
//	
//	// ADC ��ʼ��
//	Adc_Init();
//	
//	printf("\r\n ----����һ��ADC��ͨ���ж϶�ȡʵ��----\r\n");
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


	
	
	
	
	
//		// ���ô���
//	USART_Config();
//	
//		SystemInit();
//		MotorInit();
//	
//	// ADC ��ʼ��
//	Adc_Init();
//	Foward();
	
	
	
	
	status =0;
			// ���ô���
	USART_Config();
	
		SystemInit();
		MotorInit();
	
	// ADC ��ʼ��
	Adc_Init();
	SetSteerAngle3(0);
	while(1)
	{
		Forward();
		TIM_SetCompare1(TIM1,0); //13-right
		TIM_SetCompare2(TIM1,0); //11-left  (12-Mid)
	}
	
	
	
	
	
	
	
	
	
//		// ���ô���
//	USART_Config();
//	
//	// ADC ��ʼ��
//	Adc_Init();
//	
//	printf("\r\n ----����һ��ADC��ͨ���ж϶�ȡʵ��----\r\n");
//	
//	while(1)
//	{
//		printf("\r\n LEFT LEFT = %f \r\n\r\n LEFT = %f \r\n\r\n MID = %f \r\n\r\n Right = %f \r\n\r\n RIGHT RIGHT= %f \r\n\r\n Front = %f \r\n",Get_Adc(7)/1.0,Get_Adc(11)/1.0,Get_Adc(12)/1.0,Get_Adc(13)/1.0,Get_Adc(2)/1.0,Get_Adc(10)/1.0);
////		GetParament();
//		delay_nms(200);
//	}
	
}
