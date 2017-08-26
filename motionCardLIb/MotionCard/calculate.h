/********************************************************************
*Copyright(C��2014-2016,��������������˼��������������ι�˾
*FileName��	   caculate.h
*Author��      Peng Xu
*Date��        2016/10/21
*Description�� ƽ�����ѧ���㺯��ͷ�ļ�
*
*Version��     V1.0
*
********************************************************************/


#ifndef _CALCULATE_H
#define _CALCULATE_H
#include "MotionCard.h"


//�Ƕ���ת��Ϊ������ϵ��
#define CHANGE_TO_RADIAN    0.017453f   
//������ת��Ϊ�Ƕ���ϵ��
#define CHANGE_TO_ANGLE     57.2958f				
//Բ����
#define PI                  3.1415926f

#define NULL 0



//���ֿ�����̬����
typedef struct
{
	Point_t  point;
	float    angle;
	float    poseAngle;
	float    length;
	float    curvatureR;
	float    vellMax;
}KeyPointInf_t;

typedef struct
{
	float speed;
	float direction;
	float rotationVell;
}TriWheelVel2_t;



//KeyPointInf_t�ṹ���д�ű�����ռ�ֽ�����С������flash������
#define BYTESNUM 7

typedef struct
{
	Point_t point;
	float u;
}PointU_t;



float CalculateAngleAdd(float angle1, float angle2);
float CalculateAngleSub(float minuend, float subtrahend);
Point_t CalculateTwoLineIntersection2(Pose_t line1, Pose_t line2);
float CalculateLineAngle(Point_t pointStart, Point_t pointEnd);
Pose_t CalculateLine2(Point_t pointStart, Point_t pointEnd);
float CalculatePoint2PointDistance(Point_t point1, Point_t point2);
float CalculateDisPointToLine2(Point_t point, Pose_t line);
void FreeMemory(float **a, int n);
float **CreateMemory(int n);
void Gauss(float** A, float** B, int n);
void Matrix(float* constantTerm, int num,  float* m,  float* n,  float* k,  float* solution);


#endif

