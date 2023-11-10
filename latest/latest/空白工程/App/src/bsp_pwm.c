#include "bsp_pwm.h"
#include "stm32f10x.h"
#include "bsp_servo.h"
#include "bsp_delay.h"

	#define Ch1 GPIO_Pin_4	//GPIOB right
	#define Ch2 GPIO_Pin_5	//GPIOB left
	#define Ch3 GPIO_Pin_0	//GPIOB
	#define Ch4 GPIO_Pin_1	//GPIOB

void GPIO_IN1IN2_CH1(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}

void GPIO_IN1IN2_CH2(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
}

void MotorRightLeft_CH1(u8 rorl)
{
	if(rorl==1)
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_6);
		GPIO_ResetBits(GPIOB,GPIO_Pin_7);
	}
	else
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_7);
		GPIO_ResetBits(GPIOB,GPIO_Pin_6);
	}
}

void MotorRightLeft_CH2(u8 rorl)
{
	if(rorl==1)
	{
		GPIO_SetBits(GPIOC,GPIO_Pin_6);
		GPIO_ResetBits(GPIOC,GPIO_Pin_7);
	}
	else
	{
		GPIO_SetBits(GPIOC,GPIO_Pin_7);
		GPIO_ResetBits(GPIOC,GPIO_Pin_6);
	}
}

void MotorInit(void)
{
	ADVANCE_TIM_Init();
	GPIO_IN1IN2_CH1();
	GPIO_IN1IN2_CH2();
}

void RightTurn(void)
{
	MotorRightLeft_CH1(0);	//clockwise
	MotorRightLeft_CH2(1);	//clockwise
			
			TIM_SetCompare1(TIM1,10000);
			TIM_SetCompare2(TIM1,10000);
			delay_nms(570);
			
}

void LeftTurn(void)
{
	MotorRightLeft_CH1(1);	//clockwise
	MotorRightLeft_CH2(0);	//clockwise
			
			TIM_SetCompare1(TIM1,10000);
			TIM_SetCompare2(TIM1,10000);
			delay_nms(520);			 //500
			
}

void Converse(void)
{
	MotorRightLeft_CH1(1);	//clockwise
	MotorRightLeft_CH2(0);	//clockwise
			
			TIM_SetCompare1(TIM1,10000);
			TIM_SetCompare2(TIM1,10000);
			delay_nms(1200);
			
}











//void GPIO_Tim3PWM(u8 chx)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOA,ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
//	
//	switch(chx)
//	{
//		case 1:
//			GPIO_InitStructure.GPIO_Pin=Ch1;
//			break;
//		case 2:
//			GPIO_InitStructure.GPIO_Pin=Ch2;
//			break;
//		case 3:
//			GPIO_InitStructure.GPIO_Pin=Ch3;
//			break;
//		case 4:
//			GPIO_InitStructure.GPIO_Pin=Ch4;
//			break;
//	}
//	
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

//	GPIO_Init(GPIOB,&GPIO_InitStructure);

//}

//void TIM3PinReMap(u8 remap)
//{
//	switch(remap)
//	{
//		case 0:
//			break;
//		case 1:
//			GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3,ENABLE);
//			break;
//		case 2:
//			GPIO_PinRemapConfig(GPIO_FullRemap_TIM3,ENABLE);
//		break;
//	}
//}

//void TIM_Init(TIM_TypeDef *TIMx,u16 arr,u16 psc)
//{
//	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
//	
//	TIM_DeInit(TIMx);
//	TIM_InternalClockConfig(TIMx);
//	TIM_TimeBaseStructure.TIM_Period = arr - 1;
//	TIM_TimeBaseStructure.TIM_Prescaler = psc - 1;
//	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_TimeBaseInit(TIMx,&TIM_TimeBaseStructure);
//	TIM_ARRPreloadConfig(TIMx,ENABLE);
//	
//}

//void TIM_PWMMode(TIM_TypeDef *TIMx,u8 chx,u8 H2L,u16 pulse)
//{
//	
//	TIM_OCInitTypeDef TIM_OCInitStructure;
//	switch(chx)
//	{
//		case 1:
//			if(H2L)
//				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//			else
//				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
//			
//			TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
//			TIM_OCInitStructure.TIM_Pulse = pulse-1;
//			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//			TIM_OC1Init(TIMx,&TIM_OCInitStructure);
//			TIM_OC1PreloadConfig(TIMx,TIM_OCPreload_Enable);
//			TIM_CtrlPWMOutputs(TIMx,ENABLE);
//			TIM_ARRPreloadConfig(TIMx,ENABLE);
//			TIM_Cmd(TIMx,ENABLE);
//			break;
//			
//		case 2:
//			if(H2L)
//				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//			else
//				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
//			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//			TIM_OCInitStructure.TIM_Pulse = pulse-1;
//			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//			TIM_OC2Init(TIMx,&TIM_OCInitStructure);
//			TIM_OC2PreloadConfig(TIMx,TIM_OCPreload_Enable);
//			TIM_CtrlPWMOutputs(TIMx,ENABLE);
//			TIM_ARRPreloadConfig(TIMx,ENABLE);
//			TIM_Cmd(TIMx,ENABLE);
//			break;
//		case 3:
//			if(H2L)
//				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//			else
//				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
//			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//			TIM_OCInitStructure.TIM_Pulse = pulse-1;
//			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//			TIM_OC3Init(TIMx,&TIM_OCInitStructure);
//			TIM_OC3PreloadConfig(TIMx,TIM_OCPreload_Enable);
//			TIM_CtrlPWMOutputs(TIMx,ENABLE);
//			TIM_ARRPreloadConfig(TIMx,ENABLE);
//			TIM_Cmd(TIMx,ENABLE);
//			break;
//		case 4:
//			if(H2L)
//				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//			else
//				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
//			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//			TIM_OCInitStructure.TIM_Pulse = pulse-1;
//			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//			TIM_OC4Init(TIMx,&TIM_OCInitStructure);
//			TIM_OC4PreloadConfig(TIMx,TIM_OCPreload_Enable);
//			TIM_CtrlPWMOutputs(TIMx,ENABLE);
//			TIM_ARRPreloadConfig(TIMx,ENABLE);
//			TIM_Cmd(TIMx,ENABLE);
//			break;
//		default:	break;
//			
//	}
//}



