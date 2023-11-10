#include "bsp_usart_blt.h"
#include "bsp_servo.h"
#include "bsp_control.h"
#include "bsp_pwm.h"
#include "bsp_delay.h"
extern char temp[100];
extern vu16 cnt;
extern vu16 status;

vu16 cnt2=0;
// �����жϷ�����
void DEBUG_USART_IRQHandler(void)
{
	uint8_t ucTemp;
	uint8_t ucTemp1;
	if(USART_GetITStatus(DEBUG_USARTx,USART_IT_RXNE)!=RESET)
	{		

		ucTemp1=USART_ReceiveData(DEBUG_USARTx);
		ucTemp=USART_ReceiveData(DEBUG_USARTx);
		int i;  
	//	while(1){
//				for(i=0;i<2;i++){
//					temp[i]=USART_ReceiveData(DEBUG_USARTx);
//					if(temp[1]=='1')ucTemp='S';
//					if(temp[1]=='2')ucTemp='T';
//					if(temp[1]=='3')ucTemp='L';
//					if(temp[1]=='4')ucTemp='R';
//					if(temp[1]=='5')ucTemp='+';
//					if(temp[1]=='6')ucTemp='-';
//				}
		//USART_SendData(DEBUG_USARTx, ucTemp);	
		if(ucTemp=='5')
			{	
				cnt+=100;
				SetSteerAngle3(cnt);
			}				
			if(ucTemp=='6')
			{	
				cnt-=100;
				SetSteerAngle3(cnt);
			}	
			if(ucTemp=='1')
			{
				//	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable); 
				
			}
			if(ucTemp=='3')
			{
				
					MotorRightLeft_CH1(0);	//clockwise
					MotorRightLeft_CH2(1);	//clockwise
							
							TIM_SetCompare1(TIM1,3000);
							TIM_SetCompare2(TIM1,3000);
				delay_nms(400);
			}
			if(ucTemp=='4')
			{
					MotorRightLeft_CH1(1);	//clockwise
			    MotorRightLeft_CH2(0);	//clockwise
					
					TIM_SetCompare1(TIM1,3000);
					TIM_SetCompare2(TIM1,3000);
				delay_nms(400);
					//USART_SendData(DEBUG_USARTx, ucTemp);
			}
			if(ucTemp=='U')
			{
				
			}
			if(ucTemp=='2')
			{
				status++;
//				delay_nms(10000);
				//TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable); 
				
			}
			if(ucTemp=='7')
			{
				cnt2+=100;
				SetSteerAngle4(cnt2);
			}
			if(ucTemp=='8')
			{
				cnt2-=100;
				SetSteerAngle4(cnt2);
			}
		//}
	}
}

void DEBUG_UART_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/*��һ������ʼ��GPIO*/
	// �򿪴���GPIO��ʱ��
	DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK, ENABLE);
	DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK2, ENABLE);
	// ��USART Tx��GPIO����Ϊ���츴��ģʽ
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  // ��USART Rx��GPIO����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);
	
	/*�ڶ��������ô��ڵĳ�ʼ���ṹ��*/
	// �򿪴��������ʱ��
	DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK, ENABLE);
	
	// ���ô��ڵĹ�������
	// ���ò�����
	USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
	// ���� �������ֳ�
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// ����ֹͣλ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// ����У��λ
	USART_InitStructure.USART_Parity = USART_Parity_No ;  //�첽��ʹ��
	// ����Ӳ��������
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// ���ù���ģʽ���շ�һ��
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// ��ɴ��ڵĳ�ʼ������
	USART_Init(DEBUG_USARTx, &USART_InitStructure);
	/*��������ʹ�ܴ���*/
	USART_Cmd(DEBUG_USARTx, ENABLE);	
	//	// �����ж����ȼ�����
	NVIC_Configuration();
	
	// ʹ�ܴ��ڽ����ж�
	USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE);	
	
}
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Ƕ�������жϿ�������ѡ�� */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* ����USARTΪ�ж�Դ */
  NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ;
  /* �������ȼ�*/
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  /* �����ȼ� */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  /* ʹ���ж� */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  /* ��ʼ������NVIC */
  NVIC_Init(&NVIC_InitStructure);
}