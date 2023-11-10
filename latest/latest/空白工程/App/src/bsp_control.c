///////////////////////////////////////////5+1////////////////////////////////////////////////
#include "stm32f10x.h"
#include "bsp_delay.h"
#include "bsp_servo.h"
#include "bsp_usart.h"
#include "bsp_adc.h"
#include "bsp_pwm.h"
#include "bsp_track.h"
#include "bsp_control.h"

extern vu16 status;
void Foward(void)
{
		SystemInit();
		MotorInit();
		while(1)
		{
			if(Get_Adc(11)>4000)
			{
				LeftTurn();
			}
			if(Get_Adc(13)>4000)
			{
				RightTurn();
			}
			MotorRightLeft_CH1(1);	//clockwise
			MotorRightLeft_CH2(1);	//clockwise
			
			TIM_SetCompare1(TIM1,10000+GetTraceDate()*150);
			TIM_SetCompare2(TIM1,10000-GetTraceDate()*150);
			delay_nms(50);
		}
		
}


void Forward(void)
{
		int Err=0;
		int error = 0;
		int i;
		
//		SystemInit();
//		MotorInit();
		while(status%2==0)
		{
			
		if (Err > 3000)
		{
			Err = 3000;
		}
		if (Err < -3000)
		{
			Err = -3000;
		}
		
		Err = Err + (Get_Adc(11)-Get_Adc(13));
		error = Get_Adc(11)-Get_Adc(13);
		
//			if(Get_Adc(13)>3800&&Get_Adc(11)<4000)
//			{
//				delay_nms(10);
//				if(Get_Adc(13)>3800&&Get_Adc(11)<4000)
//					{
//						RightTurnSlight();
//					}
//			}
//						
//			if(Get_Adc(11)>3800&&Get_Adc(13)<4000)
//			{
//				delay_nms(10);
//				if(Get_Adc(11)>3800&&Get_Adc(13)<4000)
//					{
//						LeftTurnSlight();
//					}
//			}
			
			
			
			if(Get_Adc(13)>3700&&Get_Adc(2)<200)
			{
				delay_nms(10);
						if(Get_Adc(13)>3700&&Get_Adc(2)<200)
						{
							RightTurnSlight();
						}
						if(Get_Adc(13)>3700&&Get_Adc(2)>200)
						{
							RightTurn();
						}

			}

			if(Get_Adc(11)>3900&&Get_Adc(7)<1500)
			{
				delay_nms(10);
				if(Get_Adc(11)>3900&&Get_Adc(7)<1500)
						LeftTurnSlight();
				if(Get_Adc(11)>3900&&Get_Adc(7)>1500)
						LeftTurn();
			}
			
			if(Get_Adc(2)>200)
			{
				RightTurn();
			}
			
			
			
			if(Get_Adc(7)>1500&&Get_Adc(10)<700)//
			{  
				LeftTurn();
			}
			
//			if(Get_Adc(13)<3700&&Get_Adc(11)<3900&&Get_Adc(10)<700&&Get_Adc(7)<1500&&Get_Adc(2)<200&&Get_Adc(12)<3800)
//			{
//				
//				Converse();
//			}
			 
			MotorRightLeft_CH1(1);	//clockwise          //right TIM1_CH1 ENA
			MotorRightLeft_CH2(1);	//clockwise
			
			TIM_SetCompare1(TIM1,12000-Get_Adc(13)*2.5 + Err*0.25); //13-right
			TIM_SetCompare2(TIM1,12000-Get_Adc(11)*2.5 - Err*0.25); //11-left  (12-Mid)
			
//				TIM_SetCompare1(TIM1,12000-Get_Adc(13)*2.8 + Err*0.28); //13-right
//				TIM_SetCompare2(TIM1,12000-Get_Adc(11)*2.8 - Err*0.28); //11-left  (12-Mid)
			
//			TIM_SetCompare1(TIM1,12000+error*2 + Err*0.3); //13-right
//			TIM_SetCompare2(TIM1,12000-error*2 - Err*0.3); //11-left  (12-Mid)
			
			delay_nms(15);
//			if(USART_GetITStatus(DEBUG_USARTx,USART_IT_RXNE)!=RESET)
//			{
//				TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
//				break;
//			}
		}
		
}

void LeftTurnSlight(void)
{
			MotorRightLeft_CH1(1);	//clockwise
			MotorRightLeft_CH2(0);	//clockwise
			
			TIM_SetCompare1(TIM1,5000);
			TIM_SetCompare2(TIM1,5000);
			delay_nms(190);
}

void RightTurnSlight(void)
{
			MotorRightLeft_CH1(0);	//clockwise
			MotorRightLeft_CH2(1);	//clockwise
			
			TIM_SetCompare1(TIM1,5000);
			TIM_SetCompare2(TIM1,5000);
			delay_nms(190);
}

void LeftTurnHeavy(void)
{
		MotorRightLeft_CH1(1);	//clockwise
	MotorRightLeft_CH2(0);	//clockwise
			
			TIM_SetCompare1(TIM1,10000);
			TIM_SetCompare2(TIM1,11000);
			delay_nms(4000);
}

void RightTurnHeavy(void)
{
		MotorRightLeft_CH1(0);	//clockwise
	MotorRightLeft_CH2(1);	//clockwise
			
			TIM_SetCompare1(TIM1,10000);
			TIM_SetCompare2(TIM1,11000);
			delay_nms(4000);
}

void PID(void)
{
	
}





/////////////////////////////////////////PID/////////////////////////////////////////////////
//#include "stm32f10x.h"
//#include "bsp_delay.h"
//#include "bsp_servo.h"
//#include "bsp_usart.h"
//#include "bsp_adc.h"
//#include "bsp_pwm.h"
//#include "bsp_track.h"
//#include "bsp_control.h"

//void Foward(void)
//{
//		SystemInit();
//		MotorInit();
//		while(1)
//		{
//			if(Get_Adc(11)>4000)
//			{
//				LeftTurn();
//			}
//			if(Get_Adc(13)>4000)
//			{
//				RightTurn();
//			}
//			MotorRightLeft_CH1(1);	//clockwise
//			MotorRightLeft_CH2(1);	//clockwise
//			
//			TIM_SetCompare1(TIM1,10000+GetTraceDate()*150);
//			TIM_SetCompare2(TIM1,10000-GetTraceDate()*150);
//			delay_nms(50);
//		}
//		
//}


//void Forward(void)
//{
//		int Err=0;
//		int error = 0;
//		
////		SystemInit();
////		MotorInit();
//		while(1)
//		{
//			
//		if (Err > 3000)
//		{
//			Err = 3000;
//		}
//		if (Err < -3000)
//		{
//			Err = -3000;
//		}
//		
////		Err = Err + (Get_Adc(11)-Get_Adc(13));
//		Err=Err+GetTraceDate();
////		error = Get_Adc(11)-Get_Adc(13);
//		error = GetTraceDate();
////			if(Get_Adc(13)>3800&&Get_Adc(11)<4000)
////			{
////				delay_nms(10);
////				if(Get_Adc(13)>3800&&Get_Adc(11)<4000)
////					{
////						RightTurnSlight();
////					}
////			}
////						
////			if(Get_Adc(11)>3800&&Get_Adc(13)<4000)
////			{
////				delay_nms(10);
////				if(Get_Adc(11)>3800&&Get_Adc(13)<4000)
////					{
////						LeftTurnSlight();
////					}
////			}
//			
//			
//			
//			if(Get_Adc(13)>3700&&Get_Adc(11)<3900)
//			{
//				delay_nms(40);
//				if(Get_Adc(13)<3700&&Get_Adc(11)<3900)
//					{
//						RightTurn();
//					}
//				if(Get_Adc(13)>3700&&Get_Adc(11)<3900)
//					{
//						RightTurnSlight();
//					}
//			}

//			if(Get_Adc(11)>3900&&Get_Adc(13)<3700)
//			{
//				delay_nms(40);
//				if(Get_Adc(11)<3900&&Get_Adc(13)<3700)
//					{
//						LeftTurn();
//					}
//				if(Get_Adc(11)>3900&&Get_Adc(13)<3700)
//					{
//						LeftTurnSlight();
//					}
//			}
//			
//			
//			
//			if(Get_Adc(13)>3700&&Get_Adc(11)>3900)
//			{  
//				RightTurn();
//			}
//			
////			if(Get_Adc(13)<3700&&Get_Adc(11)<3900&&Get_Adc(12)<4000)
////			{
////				delay_nms(35);
////				Converse();
////			}
//			 
//			MotorRightLeft_CH1(1);	//clockwise          //right TIM1_CH1 ENA
//			MotorRightLeft_CH2(1);	//clockwise
//			
//			////////////////////////////////////////////////////////////////////////////////////////////
////			TIM_SetCompare1(TIM1,12000-Get_Adc(13)*2.5 + Err*0.25); //13-right
////			TIM_SetCompare2(TIM1,12000-Get_Adc(11)*2.5 - Err*0.25); //11-left  (12-Mid)
//			////////////////////////////////////////////////////////////////////////////////////////////
//			
//			
//			
////				TIM_SetCompare1(TIM1,12000-Get_Adc(13)*2.8 + Err*0.28); //13-right
////				TIM_SetCompare2(TIM1,12000-Get_Adc(11)*2.8 - Err*0.28); //11-left  (12-Mid)
//			
////			TIM_SetCompare1(TIM1,12000+error*2 + Err*0.3); //13-right
////			TIM_SetCompare2(TIM1,12000-error*2 - Err*0.3); //11-left  (12-Mid)

//				TIM_SetCompare1(TIM1,10000+error*16 + Err*0.8 ); //13-right
//  			TIM_SetCompare2(TIM1,10000-error*16 - Err*0.8); //11-left  (12-Mid)
//			
//			delay_nms(15);
//		}
//		
//}

//void LeftTurnSlight(void)
//{
//			MotorRightLeft_CH1(1);	//clockwise
//			MotorRightLeft_CH2(0);	//clockwise
//			
//			TIM_SetCompare1(TIM1,5000);
//			TIM_SetCompare2(TIM1,5000);
//			delay_nms(190);
//}

//void RightTurnSlight(void)
//{
//			MotorRightLeft_CH1(0);	//clockwise
//			MotorRightLeft_CH2(1);	//clockwise
//			
//			TIM_SetCompare1(TIM1,5000);
//			TIM_SetCompare2(TIM1,5000);
//			delay_nms(190);
//}

//void LeftTurnHeavy(void)
//{
//		MotorRightLeft_CH1(1);	//clockwise
//	MotorRightLeft_CH2(0);	//clockwise
//			
//			TIM_SetCompare1(TIM1,10000);
//			TIM_SetCompare2(TIM1,11000);
//			delay_nms(4000);
//}

//void RightTurnHeavy(void)
//{
//		MotorRightLeft_CH1(0);	//clockwise
//	MotorRightLeft_CH2(1);	//clockwise
//			
//			TIM_SetCompare1(TIM1,10000);
//			TIM_SetCompare2(TIM1,11000);
//			delay_nms(4000);
//}

//void PID(void)
//{
//	
//}

/////////////////////////////////////////FEED BACK/////////////////////////////////////////////////////

//#include "stm32f10x.h"
//#include "bsp_delay.h"
//#include "bsp_servo.h"
//#include "bsp_usart.h"
//#include "bsp_adc.h"
//#include "bsp_pwm.h"
//#include "bsp_track.h"
//#include "bsp_control.h"

//void Foward(void)
//{
//		SystemInit();
//		MotorInit();
//		while(1)
//		{
//			if(Get_Adc(11)>4000)
//			{
//				LeftTurn();
//			}
//			if(Get_Adc(13)>4000)
//			{
//				RightTurn();
//			}
//			MotorRightLeft_CH1(1);	//clockwise
//			MotorRightLeft_CH2(1);	//clockwise
//			
//			TIM_SetCompare1(TIM1,10000+GetTraceDate()*150);
//			TIM_SetCompare2(TIM1,10000-GetTraceDate()*150);
//			delay_nms(50);
//		}
//		
//}


//void Forward(void)
//{
//		int Err=0;
//		int error = 0;
//		
////		SystemInit();
////		MotorInit();
//		while(1)
//		{
//			
//		if (Err > 3000)
//		{
//			Err = 3000;
//		}
//		if (Err < -3000)
//		{
//			Err = -3000;
//		}
//		
//		Err = Err + (Get_Adc(11)-Get_Adc(13));
//		error = Get_Adc(11)-Get_Adc(13);
//		
////			if(Get_Adc(13)>3800&&Get_Adc(11)<4000)
////			{
////				delay_nms(10);
////				if(Get_Adc(13)>3800&&Get_Adc(11)<4000)
////					{
////						RightTurnSlight();
////					}
////			}
////						
////			if(Get_Adc(11)>3800&&Get_Adc(13)<4000)
////			{
////				delay_nms(10);
////				if(Get_Adc(11)>3800&&Get_Adc(13)<4000)
////					{
////						LeftTurnSlight();
////					}
////			}
//			
//			
//			
//			if(Get_Adc(13)>3700&&Get_Adc(11)<3900)
//			{
//				delay_nms(40);
//				if(Get_Adc(13)<3700&&Get_Adc(11)<3900)
//					{
//						RightTurn();
//					}
//				if(Get_Adc(13)>3700&&Get_Adc(11)<3900)
//					{
//						RightTurnSlight();
//					}
//			}

//			if(Get_Adc(11)>3900&&Get_Adc(13)<3700)
//			{
//				delay_nms(40);
//				if(Get_Adc(11)<3900&&Get_Adc(13)<3700)
//					{
//						LeftTurn();
//					}
//				if(Get_Adc(11)>3900&&Get_Adc(13)<3700)
//					{
//						LeftTurnSlight();
//					}
//			}
//			
//			
//			
//			if(Get_Adc(13)>3700&&Get_Adc(11)>3900)
//			{  
//				RightTurn();
//			}
//			
//			if(Get_Adc(13)<3700&&Get_Adc(11)<3900&&Get_Adc(10)<700&&Get_Adc(7)<1500&&Get_Adc(2)<200&&Get_Adc(12)<3800)
//			{
//				delay_nms(35);
//				if(Get_Adc(13)<3700&&Get_Adc(11)<3900&&Get_Adc(10)<700&&Get_Adc(7)<1500&&Get_Adc(2)<200&&Get_Adc(12)<3800)
//					Converse();
//			}
//			 
//			MotorRightLeft_CH1(1);	//clockwise          //right TIM1_CH1 ENA
//			MotorRightLeft_CH2(1);	//clockwise
//			
//			TIM_SetCompare1(TIM1,12000-Get_Adc(13)*2.5 + Err*0.25); //13-right
//			TIM_SetCompare2(TIM1,12000-Get_Adc(11)*2.5 - Err*0.25); //11-left  (12-Mid)
//			
////				TIM_SetCompare1(TIM1,12000-Get_Adc(13)*2.8 + Err*0.28); //13-right
////				TIM_SetCompare2(TIM1,12000-Get_Adc(11)*2.8 - Err*0.28); //11-left  (12-Mid)
//			
////			TIM_SetCompare1(TIM1,12000+error*2 + Err*0.3); //13-right
////			TIM_SetCompare2(TIM1,12000-error*2 - Err*0.3); //11-left  (12-Mid)
//			
//			delay_nms(15);
//		}
//		
//}

//void LeftTurnSlight(void)
//{
//			MotorRightLeft_CH1(1);	//clockwise
//			MotorRightLeft_CH2(0);	//clockwise
//			
//			TIM_SetCompare1(TIM1,5000);
//			TIM_SetCompare2(TIM1,5000);
//			delay_nms(190+GetTraceDate()*0.005);
//}

//void RightTurnSlight(void)
//{
//			MotorRightLeft_CH1(0);	//clockwise
//			MotorRightLeft_CH2(1);	//clockwise
//			
//			TIM_SetCompare1(TIM1,5000);
//			TIM_SetCompare2(TIM1,5000);
//			delay_nms(190-GetTraceDate()*0.005);
//}

//void LeftTurnHeavy(void)
//{
//		MotorRightLeft_CH1(1);	//clockwise
//	MotorRightLeft_CH2(0);	//clockwise
//			
//			TIM_SetCompare1(TIM1,10000);
//			TIM_SetCompare2(TIM1,11000);
//			delay_nms(4000);
//}

//void RightTurnHeavy(void)
//{
//		MotorRightLeft_CH1(0);	//clockwise
//	MotorRightLeft_CH2(1);	//clockwise
//			
//			TIM_SetCompare1(TIM1,10000);
//			TIM_SetCompare2(TIM1,11000);
//			delay_nms(4000);
//}

//void PID(void)
//{
//	
//}

/////////////////////////////////Classic/////////////////////////////////////////////////

//#include "stm32f10x.h"
//#include "bsp_delay.h"
//#include "bsp_servo.h"
//#include "bsp_usart.h"
//#include "bsp_adc.h"
//#include "bsp_pwm.h"
//#include "bsp_track.h"
//#include "bsp_control.h"

//void Foward(void)
//{
//		SystemInit();
//		MotorInit();
//		while(1)
//		{
//			if(Get_Adc(11)>4000)
//			{
//				LeftTurn();
//			}
//			if(Get_Adc(13)>4000)
//			{
//				RightTurn();
//			}
//			MotorRightLeft_CH1(1);	//clockwise
//			MotorRightLeft_CH2(1);	//clockwise
//			
//			TIM_SetCompare1(TIM1,10000+GetTraceDate()*150);
//			TIM_SetCompare2(TIM1,10000-GetTraceDate()*150);
//			delay_nms(50);
//		}
//		
//}


//void Forward(void)
//{
//		int Err=0;
//		int error = 0;
//		
////		SystemInit();
////		MotorInit();
//		while(1)
//		{
//			
//		if (Err > 3000)
//		{
//			Err = 3000;
//		}
//		if (Err < -3000)
//		{
//			Err = -3000;
//		}
//		
//		Err = Err + (Get_Adc(11)-Get_Adc(13));
//		error = Get_Adc(11)-Get_Adc(13);
//		
////			if(Get_Adc(13)>3800&&Get_Adc(11)<4000)
////			{
////				delay_nms(10);
////				if(Get_Adc(13)>3800&&Get_Adc(11)<4000)
////					{
////						RightTurnSlight();
////					}
////			}
////						
////			if(Get_Adc(11)>3800&&Get_Adc(13)<4000)
////			{
////				delay_nms(10);
////				if(Get_Adc(11)>3800&&Get_Adc(13)<4000)
////					{
////						LeftTurnSlight();
////					}
////			}
//			
//			
//			
//			if(Get_Adc(13)>3700&&Get_Adc(11)<3900)
//			{
//				delay_nms(40);
//				if(Get_Adc(13)<3700&&Get_Adc(11)<3900)
//					{
//						RightTurn();
//					}
//				if(Get_Adc(13)>3700&&Get_Adc(11)<3900)
//					{
//						RightTurnSlight();
//					}
//			}

//			if(Get_Adc(11)>3900&&Get_Adc(13)<3700)
//			{
//				delay_nms(40);
//				if(Get_Adc(11)<3900&&Get_Adc(13)<3700)
//					{
//						LeftTurn();
//					}
//				if(Get_Adc(11)>3900&&Get_Adc(13)<3700)
//					{
//						LeftTurnSlight();
//					}
//			}
//			
//			
//			
//			if(Get_Adc(13)>3700&&Get_Adc(11)>3900)
//			{  
//				RightTurn();
//			}
//			
//			if(Get_Adc(13)<3700&&Get_Adc(11)<3900&&Get_Adc(10)<700&&Get_Adc(7)<1500&&Get_Adc(2)<200&&Get_Adc(12)<3800)
//			{
//				delay_nms(35);
//				if(Get_Adc(13)<3700&&Get_Adc(11)<3900&&Get_Adc(10)<700&&Get_Adc(7)<1500&&Get_Adc(2)<200&&Get_Adc(12)<3800)
//					Converse();
//			}
//			 
//			MotorRightLeft_CH1(1);	//clockwise          //right TIM1_CH1 ENA
//			MotorRightLeft_CH2(1);	//clockwise
//			
//			TIM_SetCompare1(TIM1,12000-Get_Adc(13)*2.5 + Err*0.25); //13-right
//			TIM_SetCompare2(TIM1,12000-Get_Adc(11)*2.5 - Err*0.25); //11-left  (12-Mid)
//			
////				TIM_SetCompare1(TIM1,12000-Get_Adc(13)*2.8 + Err*0.28); //13-right
////				TIM_SetCompare2(TIM1,12000-Get_Adc(11)*2.8 - Err*0.28); //11-left  (12-Mid)
//			
////			TIM_SetCompare1(TIM1,12000+error*2 + Err*0.3); //13-right
////			TIM_SetCompare2(TIM1,12000-error*2 - Err*0.3); //11-left  (12-Mid)
//			
//			delay_nms(15);
//		}
//		
//}

//void LeftTurnSlight(void)
//{
//			MotorRightLeft_CH1(1);	//clockwise
//			MotorRightLeft_CH2(0);	//clockwise
//			
//			TIM_SetCompare1(TIM1,5000);
//			TIM_SetCompare2(TIM1,5000);
//			delay_nms(190);
//}

//void RightTurnSlight(void)
//{
//			MotorRightLeft_CH1(0);	//clockwise
//			MotorRightLeft_CH2(1);	//clockwise
//			
//			TIM_SetCompare1(TIM1,5000);
//			TIM_SetCompare2(TIM1,5000);
//			delay_nms(190);
//}

//void LeftTurnHeavy(void)
//{
//		MotorRightLeft_CH1(1);	//clockwise
//	MotorRightLeft_CH2(0);	//clockwise
//			
//			TIM_SetCompare1(TIM1,10000);
//			TIM_SetCompare2(TIM1,11000);
//			delay_nms(4000);
//}

//void RightTurnHeavy(void)
//{
//		MotorRightLeft_CH1(0);	//clockwise
//	MotorRightLeft_CH2(1);	//clockwise
//			
//			TIM_SetCompare1(TIM1,10000);
//			TIM_SetCompare2(TIM1,11000);
//			delay_nms(4000);
//}

//void PID(void)
//{
//	
//}

/////////////////////////////////////////////////////////////////////////////

