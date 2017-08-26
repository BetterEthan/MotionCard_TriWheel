#ifndef _RINGBUFFER_H
#define _RINGBUFFER_H
#include "calculate.h"
#include "stm32f4xx.h"

/*********************************************************************
* @name 		addring
* @brief  	���λ������з���һ����
* @param  	pose����ӵ�Ԫ��
* @retval 	�����Ƿ񽫻����������� 1��δ���� 0���Ѿ�����
********************************************************************/
int PutRingBuffer(KeyPointInf_t pose);

/*********************************************************************
* @name 		GetRingBufferPoint
* @brief  	����һ����    
* @param  	numȡ1����ȡ������ĵ�һ������û�е�0����
* @retval 	������ȡ������
********************************************************************/
Point_t GetRingBufferPoint(int num);



/*********************************************************************
* @name 		GetRingBufferPointAngle
* @brief  	����һ���Ƕ�    
* @param  	numȡ1����ȡ������ĵ�һ������û�е�0����
* @retval 	������ȡ��Ƕ���Ϣ
********************************************************************/
float GetRingBufferPointAngle(int num);


/*********************************************************************
* @name 		GetRingBufferPointPoseAngle
* @brief  	����һ�����ֵ���̬�Ƕ�    
* @param  	numȡ1����ȡ������ĵ�һ������û�е�0����
* @retval 	������ȡ��Ƕ���Ϣ
********************************************************************/
float GetRingBufferPointPoseAngle(int num);


/*********************************************************************
* @name 		GetCount
* @brief  	���ش�����Ԫ�ظ���    
* @param  	��
* @retval 	��
********************************************************************/
int GetCount(void);

/*********************************************************************
* @name 		GetUpPointer
* @brief  	������ָ��   
* @param  	��
* @retval 	��
********************************************************************/
int GetUpPointer(void);



/*********************************************************************
* @name 		GetDownPointer
* @brief  	������ָ��   
* @param  	��
* @retval 	��
********************************************************************/
int GetDownPointer(void);



/*********************************************************************
* @name 		DeleteData
* @brief  	ɾ�����ݵ�  
* @param  	num ��Ҫɾ����ĸ������ӵ�һ���㿪ʼɾ��
* @retval 	��
********************************************************************/
void DeleteData(int num);



/*********************************************************************
* @name 		SetUpPointer
* @brief  	������ָ��   
* @param  	��
* @retval 	��
********************************************************************/
void SetUpPointer(int num);


u32* GetFristAdress(void);

void SetRingBufferPointVell(int num,float vell);
float GetRingBufferPointVell(int num);
float GetRingBufferPointLen(int num);
float GetRingBufferAverCurvature(int num);
float GetLength(void);

void SetLength(float len);
float GetPredictTime(void);
void ClearRingBuffer(void);
#endif


