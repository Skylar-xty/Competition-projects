#include "bsp_delay.h"
#include "stm32f10x.h"

void delay_nms(uint16_t time) //72MHZ
{
	uint16_t i=0;
	while(time--)
	{
		i=12000;
		while(i--);
	}
}
