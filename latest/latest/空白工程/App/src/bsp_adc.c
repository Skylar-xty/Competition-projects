#include "bsp_adc.h"
#include "bsp_delay.h"

__IO uint16_t ADC_ConvertedValue;

/**
  * @brief  ���ADC��ֵ
  * @param  ��
  * @retval ��
  */
u16 Get_Adc(u8 ch)   
{
  	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADCͨ��,����ʱ��Ϊ239.5����	  			    
  
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1�����ת����������	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//�ȴ�ת������

	return ADC_GetConversionValue(ADC1);	//�������һ��ADC1�������ת�����
}
u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		delay_nms(5);
	}
	return temp_val/times;
} 	 

/**
  * @brief  ADC GPIO ��ʼ��
  * @param  ��
  * @retval ��
  */
static void ADCx_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// �� ADC IO�˿�ʱ��
	ADC_GPIO_APBxClock_FUN ( ADC_GPIO_CLK, ENABLE );
	
	// ���� ADC IO ����ģʽ
	// ����Ϊģ������
	//��Ҫ������������ź�
	GPIO_InitStructure.GPIO_Pin = ADC_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	
	// ��ʼ�� ADC IO
	GPIO_Init(ADC_PORT, &GPIO_InitStructure);				
}

/**
  * @brief  ����ADC����ģʽ
  * @param  ��
  * @retval ��
  */
static void ADCx_Mode_Config(void)
{
	ADC_InitTypeDef ADC_InitStructure;	
	ADC_DeInit(ADC1);
	// ��ADCʱ��
	ADC_APBxClock_FUN ( ADC_CLK, ENABLE );
	
	// ADC ģʽ����
	// ֻʹ��һ��ADC�����ڶ���ģʽ
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	
	// ��ֹɨ��ģʽ����ͨ����Ҫ����ͨ������Ҫ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE ; 

	// ����ת��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;

	// �����ⲿ����ת���������������
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;

	// ת������Ҷ���
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	
	// ת��ͨ��1��
	ADC_InitStructure.ADC_NbrOfChannel = 1;	
		
	// ��ʼ��ADC
	ADC_Init(ADCx, &ADC_InitStructure);
	
	// ����ADCʱ��ΪPCLK2��8��Ƶ����9MHz
	RCC_ADCCLKConfig(RCC_PCLK2_Div8); 
	
	// ���� ADC ͨ��ת��˳��Ͳ���ʱ��
	ADC_RegularChannelConfig(ADCx, ADC_CHANNEL, 1, 
	                         ADC_SampleTime_55Cycles5);
	
	// ADC ת�����������жϣ����жϷ�������ж�ȡת��ֵ
	ADC_ITConfig(ADCx, ADC_IT_EOC, ENABLE);
	
	// ����ADC ������ʼת��
	ADC_Cmd(ADCx, ENABLE);
	
	// ��ʼ��ADC У׼�Ĵ���  
	ADC_ResetCalibration(ADCx);
	// �ȴ�У׼�Ĵ�����ʼ�����
	while(ADC_GetResetCalibrationStatus(ADCx));
	
	// ADC��ʼУ׼
	ADC_StartCalibration(ADCx);
	// �ȴ�У׼���
	while(ADC_GetCalibrationStatus(ADCx));
	
	// ����û�в����ⲿ����������ʹ���������ADCת�� 
	ADC_SoftwareStartConvCmd(ADCx, ENABLE);
}

static void ADC_NVIC_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
	// ���ȼ�����
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  // �����ж����ȼ�
  NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  ADC��ʼ��
  * @param  ��
  * @retval ��
  */
void ADCx_Init(void)
{
	ADCx_GPIO_Config();
	ADCx_Mode_Config();
	ADC_NVIC_Config();
}

//��ʼ��ADC
//�������ǽ��Թ���ͨ��Ϊ��
//����Ĭ�Ͻ�����ͨ��0~3																	   
void  Adc_Init(void)
{ 	
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC1	, ENABLE );	  //ʹ��ADC1ͨ��ʱ��
 

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M

	//PA2 PA3 PA4��Ϊģ��ͨ����������                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	//PB0 ��Ϊģ��ͨ����������                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	ADC_DeInit(ADC1);  //��λADC1 
//////////////////////////////////////////////////////////////////////////
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//ģ��ת�������ڵ�ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//ģ��ת�������ڵ���ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);	//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���   

  
	ADC_Cmd(ADC1, ENABLE);	//ʹ��ָ����ADC1
	
	ADC_ResetCalibration(ADC1);	//ʹ�ܸ�λУ׼  
	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//�ȴ���λУ׼����
	
	ADC_StartCalibration(ADC1);	 //����ADУ׼
 
	while(ADC_GetCalibrationStatus(ADC1));	 //�ȴ�У׼����
 
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1�����ת����������

}				  


/*********************************************END OF FILE**********************/
