#include "stm32f10x.h"

void GPIO_Tim3PWM(u8 chx);
void TIM3PinReMap(u8 remap);
void TIM_Init(TIM_TypeDef *TIMx,u16 arr,u16 psc);
void GPIO_IN1IN2_CH1(void);
void GPIO_IN1IN2_CH2(void);
void TIM_PWMMode(TIM_TypeDef *TIMx,u8 chx,u8 H2L,u16 pulse);
void MotorRightLeft_CH1(u8 rorl);
void MotorRightLeft_CH2(u8 rorl);
void MotorInit(void);
void RightTurn(void);
void LeftTurn(void);
void Converse(void);
