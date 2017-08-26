#include "Bspline.h"
#include "math.h"
#include "ringBuffer.h"
#include <stdlib.h>
#include "calculate.h"
#include "posSystem.h"

/*********************************************************************************
* @name 	CaculateBsplineLen
* @brief    ���ڼ�������ȷ����һ��B�������ߵĳ���
* @param	point1 ��ʼ��				��λ mm
* @param	point2 ������				��λ mm
* @param	angle1 ��ʼ�㷽��    ��λ ��
* @param	angle2 �����㷽��    ��λ ��
* @retval   ���ظ��������߳���  ��λ mm
*********************************************************************************/
float CaculateBsplineLen(Point_t point1, Point_t point2, float angle1, float angle2)
{

		//������
		Point_t P[2];
		//��ֵ��
		//Point_t *passPoint = NULL;
		//ϵ������������
		float invA[2][2] = { { 0.333333333f, 0.00000f }, { 0.0000f, 0.333333333f } };
		//���Ƶ�����ʸ�������ǰ��
		float length = 0.0f;
		//��ʱ���Ƶ�����
		Point_t dataPoint[2];
		//���տ��Ƶ�����
		Point_t finalDataPoint[6];
		Point_t tempPoint;
		Point_t tempPointOld;
		int i, j;
		//��B�������߳���
		float u, b1, b2, b3, b0;



		length = 0.60f*sqrt((point1.x - point2.x)*(point1.x - point2.x) + (point1.y - point2.y)*(point1.y - point2.y));

		//�Գ����ֵ    angle��90��Ϊ�˴��������ǽǶ�������ϵ�ǶȲ���
		P[0].x = 3 * point1.x + length*cos((angle1) * CHANGE_TO_RADIAN);
		P[0].y = 3 * point1.y + length*sin((angle1) * CHANGE_TO_RADIAN);
		P[1].x = 3 * point2.x - length*cos((angle2) * CHANGE_TO_RADIAN);
		P[1].y = 3 * point2.y - length*sin((angle2) * CHANGE_TO_RADIAN);

		//����
		for (i = 0; i < 2; i++)
		{
			dataPoint[i].x = 0;
			dataPoint[i].y = 0;
		}


		//d = inv(a).*P  ����ʱ���Ƶ�����
		for (i = 0; i < 2; i++)
		{
			for (j = 0; j < 2; j++)
			{
				dataPoint[i].x += invA[i][j] * P[j].x;
				dataPoint[i].y += invA[i][j] * P[j].y;
			}
		}

		//�������տ��Ƶ�����
		finalDataPoint[0] = point1;
		finalDataPoint[1] = point1;
		finalDataPoint[2] = dataPoint[0];
		finalDataPoint[3] = dataPoint[1];
		finalDataPoint[4] = point2;
		finalDataPoint[5] = point2;


		length = 0.0f;
		//��B�����ϵ�һ�㵽�׶˵��ֵ����
		tempPointOld = point1;
		for (i = 0; i < 3; i++)
		{
			for (j = 0; j < 25; j++)
			{
				u = (float)(j)* 0.04f;
				b0 = 1.0f / 6 * (1 - u) * (1 - u) * (1 - u);
				b1 = 1.0f / 6 * (3 * u * u * u - 6 * u * u + 4);
				b2 = 1.0f / 6 * (-3 * u * u * u + 3 * u * u + 3 * u + 1);
				b3 = 1.0f / 6 * u * u * u;

				tempPoint.x = b0*finalDataPoint[i].x + b1*finalDataPoint[i + 1].x + b2*finalDataPoint[i + 2].x + b3*finalDataPoint[i + 3].x;
				tempPoint.y = b0*finalDataPoint[i].y + b1*finalDataPoint[i + 1].y + b2*finalDataPoint[i + 2].y + b3*finalDataPoint[i + 3].y;

				length += CalculatePoint2PointDistance(tempPoint,tempPointOld);

				tempPointOld = tempPoint;
			}
		}
		//��B���������һ�㵽ĩ�˵�ľ������
		length += CalculatePoint2PointDistance(tempPoint, point2);
		
		return length;
	}


	
	
/*********************************************************************************
* @name 	SerchVirtualPoint
* @brief  ���ݾ�������·������ȷ������Ӧ�������
* @param	robotLen ��������·������  ��λ mm 
* @retval ���� ��������� ����Ӧ���������������ϵı���ϵ��
*********************************************************************************/
PointU_t SerchVirtualPoint(float robotLen)
{
	int i = 0;
	PointU_t result;

	//ȷ��Ŀ�곤������һ����
	for (i = 1; i < GetCount(); i++)
	{
		if (robotLen < GetRingBufferPointLen(i))
			break;
	}
	
	
	
	//ȷ�����ڵ�i-1 �� i �ϵľ���
	robotLen -= GetRingBufferPointLen(i-1);
	
	result = SerchBsplineVirtualPoint(GetRingBufferPoint(i - 1), GetRingBufferPoint(i), GetRingBufferPointAngle(i - 1), GetRingBufferPointAngle(i), robotLen);

	if((i-2) > 0)
	{
			DeleteData(i-2);
	}
	return result;

}



/*********************************************************************************
* @name 	SerchBsplineVirtualPoint
* @brief  ���ݾ��������������߳���ȷ������Ӧ�������
* @param	robotLen ���������������߳���  ��λ mm
* @retval ���� ��������� ����Ӧ���������������ϵı���ϵ��
*********************************************************************************/
PointU_t SerchBsplineVirtualPoint(Point_t point1, Point_t point2, float angle1, float angle2,float robotlen)
{
	//������
	Point_t P[2];
	//ϵ������������
	float invA[2][2] = { { 0.333333333f, 0.00000f }, { 0.0000f, 0.333333333f } };
	//���Ƶ�����ʸ�������ǰ��
	float length = 0.0f;
	//��ʱ���Ƶ�����
	Point_t dataPoint[2];
	//���տ��Ƶ�����
	Point_t finalDataPoint[6];
	Point_t tempPoint;
	Point_t tempPointOld;
	int i, j;
	//��B�������߳���
	float u, b1, b2, b3, b0;
	PointU_t result;



	length = 0.60f*sqrt((point1.x - point2.x)*(point1.x - point2.x) + (point1.y - point2.y)*(point1.y - point2.y));

	//�Գ����ֵ    angle��90��Ϊ�˴��������ǽǶ�������ϵ�ǶȲ���
	P[0].x = 3 * point1.x + length*cos((angle1) * CHANGE_TO_RADIAN);
	P[0].y = 3 * point1.y + length*sin((angle1) * CHANGE_TO_RADIAN);
	P[1].x = 3 * point2.x - length*cos((angle2) * CHANGE_TO_RADIAN);
	P[1].y = 3 * point2.y - length*sin((angle2) * CHANGE_TO_RADIAN);

	//����
	for (i = 0; i < 2; i++)
	{
		dataPoint[i].x = 0;
		dataPoint[i].y = 0;
	}


	//d = inv(a).*P  ����ʱ���Ƶ�����
	for (i = 0; i < 2; i++)
	{
		for (j = 0; j < 2; j++)
		{
			dataPoint[i].x += invA[i][j] * P[j].x;
			dataPoint[i].y += invA[i][j] * P[j].y;
		}
	}


	//�������տ��Ƶ�����
	finalDataPoint[0] = point1;
	finalDataPoint[1] = point1;
	finalDataPoint[2] = dataPoint[0];
	finalDataPoint[3] = dataPoint[1];
	finalDataPoint[4] = point2;
	finalDataPoint[5] = point2;


	length = 0.0f;
	//��B�����ϵ�һ�㵽�׶˵��ֵ����
	tempPointOld = point1;
	int num = 0;
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 50; j++)
		{
			//���ڼ���Uֵ
			num++;

			u = (float)(j)* 0.02f;

			b0 = 1.0f / 6 * (1 - u) * (1 - u) * (1 - u);
			b1 = 1.0f / 6 * (3 * u * u * u - 6 * u * u + 4);
			b2 = 1.0f / 6 * (-3 * u * u * u + 3 * u * u + 3 * u + 1);
			b3 = 1.0f / 6 * u * u * u;

			tempPoint.x = b0*finalDataPoint[i].x + b1*finalDataPoint[i + 1].x + b2*finalDataPoint[i + 2].x + b3*finalDataPoint[i + 3].x;
			tempPoint.y = b0*finalDataPoint[i].y + b1*finalDataPoint[i + 1].y + b2*finalDataPoint[i + 2].y + b3*finalDataPoint[i + 3].y;

			length += CalculatePoint2PointDistance(tempPoint, tempPointOld);
			if (robotlen < length)
			{
				result.point = tempPoint;
				result.u = (float)num / 150.0f;
				return result;
			}

			tempPointOld = tempPoint;
		}
	}

	//��B���������һ�㵽ĩ�˵�ľ������
	result.point = point2;

	result.u = 1.0f;

	return result;
}

//ȷ������Ŀ���
PointU_t SerchVirtualPoint2(float robotLen)
{
	int i = 0;
	PointU_t result;

	//ȷ��Ŀ�곤������һ����
	for (i = 1; i < GetCount(); i++)
	{
		if (robotLen < GetRingBufferPointLen(i))
			break;
	}
	
	
	
	//ȷ�����ڵ�i-1 �� i �ϵľ���
	robotLen -= GetRingBufferPointLen(i-1);
	
	result = SerchBsplineVirtualPoint(GetRingBufferPoint(i - 1), GetRingBufferPoint(i), GetRingBufferPointAngle(i - 1), GetRingBufferPointAngle(i), robotLen);

	return result;

}





/***************************************************************************
* @name		BspSegment
* @brief	�Ѵ����B�����Ĺؼ���ֶγ�15cmһ�ε����������߷���ĵ�
* @num		����ĵ����
* @points	�����B�����ؼ���
* @keyPoint	�ֶκ����ĵ�
**************************************************************************/
int BspSegment(int num, Pose_t* points, KeyPointInf_t* keyPoint)
{
	//ϵ������Խ���
	float* a = NULL;
	a = (float *)malloc(sizeof(float)* num);
	//ϵ������Խ�����
	float* b = NULL;
	b = (float *)malloc(sizeof(float)* num);
	//ϵ������Խ�����
	float* c = NULL;
	c = (float *)malloc(sizeof(float)* num);
	//����soluctionX��soluctionYΪ���Է��̵Ľ�
	float* soluctionX = NULL;
	soluctionX = (float *)malloc(sizeof(float)* num);
	float* soluctionY = NULL;
	soluctionY = (float *)malloc(sizeof(float)* num);
	//����dataX��dataY,�������inPoint���X��Y����
	float* dataX = NULL;
	dataX = (float *)malloc(sizeof(float)* num);
	float* dataY = NULL;
	dataY = (float *)malloc(sizeof(float)* num);

	//����controlPoint������ſ��Ƶ�
	Point_t* controlPoint = NULL;
	controlPoint = (Point_t*)malloc(sizeof(Point_t)*(num + 4));
	//��Ż��ߵ�����ʹ�õ�
 	Point_t lines[2];

	//��ʼ�� a,b,c
	a[0] = 18;
	a[num - 1] = 18;

	for (int i = 1; i < num - 1; i++)
	{
		a[i] = 4;
	}

	for (int i = 1; i < num - 1; i++)
	{
		b[i] = 1;
		c[i] = 1;
	}

	c[num - 1] = -9;
	b[0] = -9;



	for (int i = 0; i<num; i++)
	{
		dataX[i] = 6.0f * points[i].point.x;
		dataY[i] = 6.0f * points[i].point.y;
	}

	dataX[0] *= 1.5f;
	dataY[0] *= 1.5f;
	dataX[num - 1] *= 1.5f;
	dataY[num - 1] *= 1.5f;






	//����outdataX,outdataY;

	//����Matrix��׷�Ϸ�������Է���
	Matrix(dataX, num, a, b, c, soluctionX);
	Matrix(dataY, num, a, b, c, soluctionY);



	controlPoint[num + 3].x = dataX[num - 1] / 9;
	controlPoint[num + 2].x = dataX[num - 1] / 9;
	controlPoint[0].x = dataX[0] / 9;
	controlPoint[1].x = dataX[0] / 9;


	for (int i = 0; i < num; i++)
	{
		controlPoint[i + 2].x = soluctionX[i];

	}

	controlPoint[num + 3].y = dataY[num - 1] / 9;
	controlPoint[num + 2].y = dataY[num - 1] / 9;
	controlPoint[0].y = dataY[0] / 9;
	controlPoint[1].y = dataY[0] / 9;


	for (int i = 0; i < num; i++)
	{
		controlPoint[i + 2].y = soluctionY[i];

	}



	//������ֵ�㣬�ֳɺܽ��ĵ㣬ȡ15cmһ��
	//�ӳ�ʼ�㿪ʼ
	lines[0].x = (int)controlPoint[0].x;
	lines[0].y = (int)controlPoint[0].y;

	//�ۻ��ĳ��ȴ���length
	float length = 0;
	float Len = 0;
	float startLength = 0;
	float beforeLength = 0;
	float endLength = 0;

	//����ֶ�֮��ĵ�ĸ�����Ȼ������ڴ�ռ�
	int number = 0;


	for (int i = 0; i < num + 1; i++)
	{
		if (i == num - 1)
			beforeLength = length;
		for (float u = 0.01f; u <= 1; u += 0.01f)
		{
			float b0 = 1.0f / 6 * (1 - u) * (1 - u) * (1 - u);
			float b1 = 1.0f / 6 * (3 * u * u * u - 6 * u * u + 4);
			float b2 = 1.0f / 6 * (-3 * u * u * u + 3 * u * u + 3 * u + 1);
			float b3 = 1.0f / 6 * u * u * u;

			lines[1].x = (int)(b0 * controlPoint[i].x + b1 * controlPoint[i + 1].x + b2 * controlPoint[i + 2].x + b3 * controlPoint[i + 3].x);
			lines[1].y = (int)(b0 * controlPoint[i].y + b1 * controlPoint[i + 1].y + b2 * controlPoint[i + 2].y + b3 * controlPoint[i + 3].y);

			//gra.DrawLine(pen, lines[0], lines[1]);
			length += CalculatePoint2PointDistance(lines[0], lines[1]);
			if ( i <= 1)
				startLength += CalculatePoint2PointDistance(lines[0], lines[1]);
			if (i >= num -1)
				endLength += CalculatePoint2PointDistance(lines[0], lines[1]);
			if (length -Len > 149.25f)
			{
				Len = length;
				number++;
			}

			lines[0] = lines[1];
		}
	}
//	keyPoint = (Act3WheelPose *)malloc(sizeof(ActLine2)* number);
	int resultNum = number + 1;
	
	float sumLen = length;
	Len = 0;
	length = 0;
	
	lines[0].x = (int)controlPoint[0].x;
	lines[0].y = (int)controlPoint[0].y;
	
	

	number = 1;
	for (int i = 0; i < num + 1; i++)
	{

		for (float u = 0.01f; u <= 1; u += 0.01f)
		{
			float b0 = 1.0f / 6 * (1 - u) * (1 - u) * (1 - u);
			float b1 = 1.0f / 6 * (3 * u * u * u - 6 * u * u + 4);
			float b2 = 1.0f / 6 * (-3 * u * u * u + 3 * u * u + 3 * u + 1);
			float b3 = 1.0f / 6 * u * u * u;

			lines[1].x = (int)(b0 * controlPoint[i].x + b1 * controlPoint[i + 1].x + b2 * controlPoint[i + 2].x + b3 * controlPoint[i + 3].x);
			lines[1].y = (int)(b0 * controlPoint[i].y + b1 * controlPoint[i + 1].y + b2 * controlPoint[i + 2].y + b3 * controlPoint[i + 3].y);

			//gra.DrawLine(pen, lines[0], lines[1]);
			length += CalculatePoint2PointDistance(lines[0], lines[1]);
			if (length - Len > 149)
			{
				Len = length;
				float Db0 = -1.0f / 2 * (u - 1) * (u - 1);
				float Db1 = 1.0f / 2 * (3 * u * u - 4 * u);
				float Db2 = 1.0f / 2 * (-3 * u * u + u * 2 + 1);
				float Db3 = 1.0f / 2 * u * u;
				
				//���߷���
				keyPoint[number].angle = CHANGE_TO_ANGLE * atan2f((float)(Db0 * controlPoint[i].y + Db1 * controlPoint[i + 1].y + Db2 * controlPoint[i + 2].y + Db3 * controlPoint[i + 3].y), \
					(float)(Db0 * controlPoint[i].x + Db1 * controlPoint[i + 1].x + Db2 * controlPoint[i + 2].x + Db3 * controlPoint[i + 3].x));


				keyPoint[number].point.x = lines[1].x;
				keyPoint[number].point.y = lines[1].y;

				
					
				//��̬�Ƕ�
				if (num == 2)
				{
					keyPoint[number].poseAngle =CalculateAngleAdd( points[0].direction, CalculateAngleSub(points[1].direction,points[0].direction) / sumLen * length);
				}
				else 
				{
					if (i > 1 && i < num - 1)
					{
						keyPoint[number].poseAngle =CalculateAngleAdd( points[i - 1].direction , u * CalculateAngleSub(points[i].direction,points[i - 1].direction));
					}
					if (i <= 1)
					{
						keyPoint[number].poseAngle = CalculateAngleAdd(points[0].direction , CalculateAngleSub(points[1].direction,points[0].direction) / startLength * length);
					}
					if (i >= num - 1)
					{
						keyPoint[number].poseAngle = CalculateAngleAdd(points[num - 2].direction , CalculateAngleSub(points[num - 1].direction,points[num - 2].direction) * (length - beforeLength) / endLength);
					}
				}

				number++;
			}

			lines[0] = lines[1];
		}
	}
	
	
	keyPoint[0].point = GetPosPresent().point;
	keyPoint[0].angle = GetPosPresent().direction;
	keyPoint[0].poseAngle = keyPoint[1].poseAngle;
	keyPoint[resultNum].point = points[num-1].point;
	keyPoint[resultNum].angle = keyPoint[resultNum - 1].angle;
	keyPoint[resultNum].poseAngle = points[num - 1].direction;
	resultNum++;
	
	free(a);
	free(b);
	free(c);
	free(soluctionX);
	free(soluctionY);
	free(dataX);
	free(dataY);
	free(controlPoint);
	return resultNum;
}



