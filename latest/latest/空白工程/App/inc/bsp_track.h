#ifndef _TRACK_H__
#define _TRACK_H__
/*****************����ȷ������**************/

//#define D_AD_VALUE -400 		//ȷ�����Ҵ�������ֵ-560 output:left minus
//#define LEFT_MAX 2927   	//�󴫸�����ֵ2546    mid 2550
//#define RIGHT_MAX 2868  	//�Ҵ�������ֵ2570
//#define LEFT_THERSH 954	//�󴫸�����ֵ
//#define RIGHT_THERSH 1258	//�Ҵ�������ֵ
//#define LEFT_SPAN 3763		//�����������ƶ���Ծ��ֵ   //790
//#define RIGHT_SPAN -2831		//�����������ƶ���Ծ��ֵ   //1023

#define D_AD_VALUE -34 		//ȷ�����Ҵ�������ֵ-560
#define LEFT_MAX 2297   	//�󴫸�����ֵ2546    mid 2550	//2595
#define RIGHT_MAX 2854  	//�Ҵ�������ֵ2570							//2653
#define LEFT_THERSH 954	//�󴫸�����ֵ
#define RIGHT_THERSH 1258	//�Ҵ�������ֵ
#define LEFT_SPAN 3763		//�����������ƶ���Ծ��ֵ   //790
#define RIGHT_SPAN -2831		//�����������ƶ���Ծ��ֵ   //1023



/****************��������********************/
extern int GetTraceDate(void);
extern void GetParament(void);


#endif
