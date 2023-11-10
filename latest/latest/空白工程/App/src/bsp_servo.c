#include "bsp_servo.h"
#include "stm32f10x.h"
#include "bsp_delay.h"

void PWM1_GPIOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8;//����TIM1_CH3���PWM����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //ʹ��GPIOA����ʱ��ʹ��
	GPIO_SetBits(GPIOA,GPIO_Pin_8);				 //PA10 ���
}

void PWM2_GPIOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;//����TIM1_CH4���PWM����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //ʹ��GPIOA����ʱ��ʹ��
	GPIO_SetBits(GPIOA,GPIO_Pin_9);				 //PA11 ���
}

void PWM3_GPIOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;//����TIM1_CH3���PWM����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //ʹ��GPIOA����ʱ��ʹ��
	GPIO_SetBits(GPIOA,GPIO_Pin_10);				 //PA10 ���
}

void PWM4_GPIOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;//����TIM1_CH4���PWM����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //ʹ��GPIOA����ʱ��ʹ��
	GPIO_SetBits(GPIOA,GPIO_Pin_11);				 //PA11 ���
}













void PWMInit(u16 arr,u16 psc)
{		 		
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //ʹ��TIMx����
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //ʹ��GPIOA����ʱ��ʹ��
 
	TIM_TimeBaseStructure.TIM_Period = arr; //�����Զ���װ������ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����Ԥ��Ƶֵ ����Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx
	
	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //CH1 PWM2ģʽ	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //OC1 �͵�ƽ��Ч 
	
	TIM_OCInitStructure.TIM_Pulse = 0; //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);  //����ָ���Ĳ�����ʼ������TIMx
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH1 Ԥװ��ʹ��
	
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);  //����ָ���Ĳ�����ʼ������TIMx
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH1 Ԥװ��ʹ��
	
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);  //����ָ���Ĳ�����ʼ������TIMx
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH1 Ԥװ��ʹ��
	
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //����ָ���Ĳ�����ʼ������TIMx
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH1 Ԥװ��ʹ��
	
	TIM_ARRPreloadConfig(TIM1, ENABLE); //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���
	
	TIM_CtrlPWMOutputs(TIM1,ENABLE);	//MOE �����ʹ��,�߼���ʱ�����뿪����� 
	
	TIM_Cmd(TIM1, ENABLE);  //ʹ��TIMx
} 






//���ö���ǶȺ���
//������
//angle��0-180��

void SetSteerAngle1(u8 angle)
{
	u16 PWM;
	PWM=angle*2+360;
	TIM_SetCompare1(TIM1,PWM);
}

void SetSteerAngle2(u8 angle)
{
	u16 PWM;
	PWM=angle*2+360;
	TIM_SetCompare2(TIM1,PWM);
}

void SetSteerAngle3(u8 angle)
{
	u16 PWM;
	PWM=angle*2+360;
	TIM_SetCompare3(TIM1,PWM);
}

void SetSteerAngle4(u8 angle)
{
	u16 PWM;
	PWM=angle*2+360;
	TIM_SetCompare4(TIM1,PWM);
}








void ADVANCE_TIM_Init(void)
{
	
	
	
	PWMInit(7199,199);//���Ƶ��50HZ
	
	PWM1_GPIOInit();
	TIM_SetCompare3(TIM1,165);
	
	
	PWM2_GPIOInit();
	TIM_SetCompare3(TIM1,165);
	
	
	PWM3_GPIOInit();
	TIM_SetCompare3(TIM1,165);
	
	
	PWM4_GPIOInit();
	TIM_SetCompare4(TIM1,165);
}



////�����ʼ������
//void Servor_Init(void)
//{
//	PWM4Init(7199,199);//���Ƶ��50HZ
//	PWM4_GPIOInit();
//	TIM_SetCompare4(TIM1,165);
//}












//������Ժ���
void Servor_Test(void)
{
	TIM_SetCompare4(TIM1,660);
	delay_nms(1000);
	TIM_SetCompare4(TIM1,560);
	delay_nms(1000);
	TIM_SetCompare4(TIM1,460);
	delay_nms(1000);
	TIM_SetCompare4(TIM1,560);
	delay_nms(1000);
}
