#include "stm32f10x.h"
#ifndef __BSP_GENERALTIME_H
#define __BSP_GENERALTIME_H


#define            GENERAL_TIM                   TIM1
#define            GENERAL_TIM_APBxClock_FUN     RCC_APB2PeriphClockCmd
#define            GENERAL_TIM_CLK               RCC_APB2Periph_TIM1
#define            GENERAL_TIM_Period            19999
#define            GENERAL_TIM_Prescaler         71
// TIM1 输出比较通道1
#define            GENERAL_TIM_CH1_GPIO_CLK      RCC_APB2Periph_GPIOA
#define            GENERAL_TIM_CH1_PORT          GPIOA
#define            GENERAL_TIM_CH1_PIN           GPIO_Pin_8
// TIM1 输出比较通道2
#define            GENERAL_TIM_CH2_GPIO_CLK      RCC_APB2Periph_GPIOA
#define            GENERAL_TIM_CH2_PORT          GPIOA
#define            GENERAL_TIM_CH2_PIN           GPIO_Pin_9


/**************************函数声明********************************/

void ADVANCE_TIM_Init(void);
void SetSteerAngle1(u8 angle);//
void SetSteerAngle2(u8 angle);//
void SetSteerAngle3(u8 angle);//
void SetSteerAngle4(u8 angle);//


//void Servor_Init(void);
void Servor_Test(void);


#endif	/* __BSP_GENERALTIME_H */
