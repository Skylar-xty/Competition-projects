#include "stdio.h"
#include "bsp_track.h"
#include "bsp_adc.h"
#include "stdlib.h"
int PosFlagValue=(int)((LEFT_MAX+RIGHT_MAX-LEFT_THERSH-RIGHT_THERSH)/3.0f);
//������У׼
//void (void)
//{
//	//���pid

////��ȡѭ���������������
////����ֵ��int���ͣ���Χѭ�����������ݣ����ݴ�ֵ������С������Ƕ�
int GetTraceDate(void)//main.c
{
	int Data_Out;//���������������
	int Left_AD,Right_AD,Mid_AD;//���������д�����ADֵ����
	static char PosFlag=0;//���崫����λ�ñ�־λ��0��Ϊ�������ں���ƫ��λ�ã�1��ΪС���ڴ�����ƫ��λ��
	
	Left_AD=Get_Adc(11);
 	Mid_AD=Get_Adc(12);
	Right_AD=Get_Adc(13);
	
	Data_Out=(Left_AD-Right_AD+D_AD_VALUE);
	if(Data_Out>PosFlagValue)
	{
		PosFlag=1;
	}
	else if(Data_Out<-PosFlagValue)
	{
		PosFlag=0;
	}
	if(Mid_AD<LEFT_THERSH)
	{	
		if(Data_Out>PosFlagValue)
		{
			Data_Out=(2*LEFT_MAX-Left_AD)*2-LEFT_SPAN;
		}
		else if((Data_Out<PosFlagValue)&&(PosFlag==1))
		{
			Data_Out=abs((2*LEFT_MAX-Left_AD)*2-LEFT_SPAN);
		}
		
	} 
	
	if(Mid_AD<RIGHT_THERSH)
	{	
		if(Data_Out<-PosFlagValue)
		{
			Data_Out=(Right_AD-2*RIGHT_MAX)*2-RIGHT_SPAN;
		}
		else if((Data_Out>-PosFlagValue)&&(PosFlag==0))
		{
			Data_Out=-abs((Right_AD-2*RIGHT_MAX)*2-RIGHT_SPAN);
		}
	}
	
	return Data_Out;
}

//ѭ��������У׼����
void GetParament(void)//main.c
{
	int DValue=0;
	int Left_AD,Right_AD,Mid_AD;//���������д�����ADֵ����
	
	static int LeftMax=0;
	static int RightMax=0;
	static int Left_Thersh=0;
	static int Right_Thersh=0;
	static int Left_Span=0;
	static int Right_Span=0;
	
	
	Right_AD=Get_Adc(13); 	//�Ҵ�������ȡ��ADֵ
	Mid_AD=Get_Adc(12);	//�м䴫������ȡ��ADֵ
	Left_AD=Get_Adc(11);		//�󴫸�����ȡ��ADֵ
		
	
	if(Left_AD>LeftMax)	
	{
		LeftMax=Left_AD;
		Left_Thersh=Mid_AD;
		Left_Span=(2*LeftMax-Left_AD)*2-(Left_AD-Right_AD+D_AD_VALUE);
		
	}
	if(Right_AD>RightMax)
	{
		RightMax=Right_AD;
		Right_Thersh=Mid_AD;
		Right_Span=(Right_AD-2*RightMax)*2-(Left_AD-Right_AD+D_AD_VALUE);	
	}		
	
	
	DValue=Right_AD-Left_AD;//��ֵ���Ҵ��������󴫸���
	
	
	printf("Right_AD:%d Mid_AD:%d Left_AD:%d\r\n",Right_AD,Mid_AD,Left_AD);
	printf("D_AD_VALUE:%d LeftMax:%d RightMax:%d Left_Thersh:%d Right_Thersh:%d Left_Span:%d Right_Span:%d\r\n",DValue,LeftMax,RightMax,Left_Thersh,Right_Thersh,Left_Span,Right_Span);
}





