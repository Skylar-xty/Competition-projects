#include "bsp_servo.h"
#include "stm32f10x.h"
#include "bsp_delay.h"

void PWM1_GPIOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8;//设置TIM1_CH3输出PWM波形
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //使能GPIOA外设时钟使能
	GPIO_SetBits(GPIOA,GPIO_Pin_8);				 //PA10 输出
}

void PWM2_GPIOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;//设置TIM1_CH4输出PWM波形
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //使能GPIOA外设时钟使能
	GPIO_SetBits(GPIOA,GPIO_Pin_9);				 //PA11 输出
}

void PWM3_GPIOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;//设置TIM1_CH3输出PWM波形
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //使能GPIOA外设时钟使能
	GPIO_SetBits(GPIOA,GPIO_Pin_10);				 //PA10 输出
}

void PWM4_GPIOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;//设置TIM1_CH4输出PWM波形
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //使能GPIOA外设时钟使能
	GPIO_SetBits(GPIOA,GPIO_Pin_11);				 //PA11 输出
}













void PWMInit(u16 arr,u16 psc)
{		 		
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //使能TIMx外设
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //使能GPIOA外设时钟使能
 
	TIM_TimeBaseStructure.TIM_Period = arr; //设置自动重装载周期值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置预分频值 不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx
	
	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //CH1 PWM2模式	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //OC1 低电平有效 
	
	TIM_OCInitStructure.TIM_Pulse = 0; //设置待装入捕获比较寄存器的脉冲值
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);  //根据指定的参数初始化外设TIMx
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH1 预装载使能
	
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);  //根据指定的参数初始化外设TIMx
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH1 预装载使能
	
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);  //根据指定的参数初始化外设TIMx
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH1 预装载使能
	
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //根据指定的参数初始化外设TIMx
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH1 预装载使能
	
	TIM_ARRPreloadConfig(TIM1, ENABLE); //使能TIMx在ARR上的预装载寄存器
	
	TIM_CtrlPWMOutputs(TIM1,ENABLE);	//MOE 主输出使能,高级定时器必须开启这个 
	
	TIM_Cmd(TIM1, ENABLE);  //使能TIMx
} 






//设置舵机角度函数
//参数：
//angle：0-180度

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
	
	
	
	PWMInit(7199,199);//输出频率50HZ
	
	PWM1_GPIOInit();
	TIM_SetCompare3(TIM1,165);
	
	
	PWM2_GPIOInit();
	TIM_SetCompare3(TIM1,165);
	
	
	PWM3_GPIOInit();
	TIM_SetCompare3(TIM1,165);
	
	
	PWM4_GPIOInit();
	TIM_SetCompare4(TIM1,165);
}



////舵机初始化函数
//void Servor_Init(void)
//{
//	PWM4Init(7199,199);//输出频率50HZ
//	PWM4_GPIOInit();
//	TIM_SetCompare4(TIM1,165);
//}












//舵机测试函数
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
