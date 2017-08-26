#ifndef _SAMPLING_H
#define  _SAMPLING_H



#include  "MotionCard.h"
#include  "calculate.h"


/*********************************************************************
* @name 		CaculateBeginAngle
* @brief  	���������ʼʱ��һ��͵ڶ���ķ���
* @param  	point1 ��ʼ��
* @param    point2 ������
* @retval 	������ʼ��ͽ��������̬�Ƕ�
********************************************************************/
float CaculateBeginAngle(Point_t point1,Point_t point2);


/*********************************************************************
* @name 		CaculateProcessAngle
* @brief  	������������ɽ�������������ķ���
* @param  	point1 ��ʼ��
* @param    point2 ������
* @param    angleLast  ��ʼ��ķ���
* @retval 	����ʸ���ĽǶ� -180~180
********************************************************************/
float CaculateProcessAngle(Point_t point1, Point_t point2, float angleLast);


/*********************************************************************
* @name 		CaculateDirectionAngle
* @brief  	����ʸ����ƽ������ϵ�еķ���Ƕ�
* @param  	direction ����ʸ��
* @retval 	����ʸ���ĽǶ� -180~180
********************************************************************/
float CaculateDirectionAngle(Point_t direction);

void PoseSamplingLastPoint(void);

#endif

