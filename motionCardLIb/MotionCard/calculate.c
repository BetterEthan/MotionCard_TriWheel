/********************************************************************
*Copyright(C��2014-2016,��������������˼��������������ι�˾
*FileName��	   caculate.c
*Author��      Peng Xu
*Date��        2016/10/21
*Description�� ƽ�����ѧ���㺯��
*
*Version��     V1.0
*
********************************************************************/


#include "math.h"
#include "calculate.h"
#include <stdlib.h>



/*********************************************************************************
* @name 	CalculateAngleAdd
* @brief	��-180,180���紦������
* @param	angle1:�Ƕ�1;
* @param    angle2:�Ƕ�2;
* @retval   �������֮��ĽǶ� -180~180
**********************************************************************************/
float CalculateAngleAdd(float angle1, float angle2)
{
	float result = 0.0f;
	result = angle1 + angle2;
	if (result >  180.0f)  result -= 360.0f;
	if (result < -180.0f)  result += 360.0f;
	return result;
}



/*********************************************************************************
* @name 	CalculateAngleSub
* @brief	��-180,180���紦������
* @param	minuend: ������;
			subtrahend: ���� A - B,AΪ��������BΪ����;
* @retval	���ؼ����ĽǶ�ֵ -180~180
*********************************************************************************/
float CalculateAngleSub(float minuend, float subtrahend)
{
	float result = 0.0f;
	result = minuend - subtrahend;
	if (result >  180.0f)  result -= 360.0f;
	if (result < -180.0f)  result += 360.0f;
	return result;
}

/*********************************************************************************
* @name 	CalculateTwoLineIntersection2
* @brief	��������ֱ�ߵĽ���
* @param	line1:ֱ��1;
* @param    line2:ֱ��2;
* @retval	���ؽ�������
**********************************************************************************/
Point_t CalculateTwoLineIntersection2(Pose_t line1, Pose_t line2)
{
	Point_t intersection;
	//б��
	float k1 = 0.0f;
	float k2 = 0.0f;

	//��Ϊ��������,δ����x�ᴹֱ��ֱ�ߴ���
	k1 = tan(line1.direction * CHANGE_TO_RADIAN);
	k2 = tan(line2.direction * CHANGE_TO_RADIAN);

	intersection.x = (line1.point.x*k1 - line1.point.y - line2.point.x * k2 + line2.point.y)
		/ (k1 - k2);
	intersection.y = k1 * (intersection.x - line1.point.x) + line1.point.y;

	return intersection;
}


/*********************************************************************************
* @name 	CalculateLineAngle
* @brief	��������ʸ������Ƕ�
* @param	pointStart:��ʼ�㣺
* @param	pointEnd:��ֹ��;
* @retval	����ʸ������Ƕ� -180~180
*********************************************************************************/
float CalculateLineAngle(Point_t pointStart, Point_t pointEnd)
{
	float a = 0.0f;
	float b = 0.0f;

	a = pointEnd.y - pointStart.y;
	b = pointEnd.x - pointStart.x;
	//atan2f��Χ���԰���-180��180  
	return (atan2f(a, b) * CHANGE_TO_ANGLE);
}

/*********************************************************************************
* @name 	CalculateLine2
* @brief	����ȷ��һ��ֱ��
* @param	pointStart:��ʼ��;
* @param	pointEnd:��ֹ��;
* @retval   Pose_t��ʽ������ֱ�߷���Ƕȣ�ֱ����һ������
*********************************************************************************/
Pose_t CalculateLine2(Point_t pointStart, Point_t pointEnd)
{
	float a = 0.0f;
	float b = 0.0f;
	Pose_t line2;

	a = pointEnd.y - pointStart.y;
	b = pointEnd.x - pointStart.x;
	line2.direction = (atan2f(a, b) * CHANGE_TO_ANGLE);
	line2.point = pointStart;
	//atan2f��Χ���԰���-180��180  
	return line2;
}

/*********************************************************************************
* @name 	CalculatePoint2PointDistance
* @brief	����㵽��ľ���
* @param	point1 ��ʼ��
* @param	point2 ������
* @retval   ��������֮��ľ���
*********************************************************************************/
float CalculatePoint2PointDistance(Point_t point1, Point_t point2)
{
	float dis;
	dis = sqrt((point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y));
	return dis;
}





/***********************************************************************************
* @name 	CalculateDisPointToLine2
* @brief	����㵽ֱ�߾��� ����ֵΪ��������ֱ���Ҳ࣬Ϊ��������ֱ�����
* @param	point ������
* @param	line  ֱ�߲���
* @retval	�㵽ֱ�ߵľ���
**********************************************************************************/


float CalculateDisPointToLine2(Point_t point, Pose_t line)
{
	float dis, k, b;

	k = tanf(line.direction * CHANGE_TO_RADIAN);
	b = line.point.y - k * line.point.x;
	dis = -(k * point.x + b - point.y) / sqrt(1 + k * k);
	
	return dis;
}


/*********************************************************************************
* @name 	**CreateMemory(int n)
* @brief	����n*n���ڴ棬������ֵΪ0
* @param	n��������Ҳ������
* @retval
**********************************************************************************/
float **CreateMemory(int n)
{
	float **a = NULL;
	int i, j;

	//Ϊ��ά�������row��
	a = (float**)malloc(sizeof(void *)* n);
	//Ϊÿ�з���n����С�ռ�
	for (i = 0; i < n; ++i){
		a[i] = (float*)malloc(sizeof(float)* n);
	}
	for (i = 0; i < n; i++){
		for (j = 0; j < n; j++){
			a[i][j] = 0;
		}
	}

	return a;
}

/*********************************************************************************
* @name 	FreeMemory(float **a, int n)
* @brief	�ͷ��ڴ溯��
* @param	a�����ͷŵĶ�λ�����׵�ַ��   n������������
* @retval
**********************************************************************************/
void FreeMemory(float **a, int n)
{
	
	
	int i;
	for (i = 0; i < n; ++i){
		free(a[i]);
	}
	free(a);
}

/*********************************************************************************
* @name 	Gauss
* @brief	���ò�����Ԫ�ĸ�˹��ȥ������A�������B
* @param	A:���뷽��;B:�������;
* @retval
**********************************************************************************/
void Gauss(float** A, float** B, int n)
{
	int i, j, k;
	float max, temp;
	float **t = NULL;

	t = CreateMemory(n);
	// **t  ��ʱ����  
	//��A����������ʱ����t[n][n]��  
	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			t[i][j] = A[i][j];
		}
	}
	//��ʼ��B����Ϊ��λ��  
	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			B[i][j] = (i == j) ? (float)1 : 0;
		}
	}
	for (i = 0; i < n; i++)
	{
		//Ѱ����Ԫ  
		max = t[i][i];
		k = i;
		for (j = i + 1; j < n; j++)
		{
			if (fabs(t[j][i]) > fabs(max))
			{
				max = t[j][i];
				k = j;
			}
		}
		//�����Ԫ�����в��ǵ�i�У������н���  
		if (k != i)
		{
			for (j = 0; j < n; j++)
			{
				temp = t[i][j];
				t[i][j] = t[k][j];
				t[k][j] = temp;
				//B���潻��  
				temp = B[i][j];
				B[i][j] = B[k][j];
				B[k][j] = temp;
			}
		}
		//�ж���Ԫ�Ƿ�Ϊ0, ����, �����A�������Ⱦ���,�����������  
		if (t[i][i] == 0)
		{

		}
		//��ȥA�ĵ�i�г�ȥi������ĸ���Ԫ��  
		temp = t[i][i];
		for (j = 0; j < n; j++)
		{
			//���Խ����ϵ�Ԫ�ر�Ϊ1  
			t[i][j] = t[i][j] / temp;
			//�������  			
			B[i][j] = B[i][j] / temp;
		}
		//��0��->��n��  
		for (j = 0; j < n; j++)
		{
			//���ǵ�i��  
			if (j != i)
			{
				temp = t[j][i];
				//��j��Ԫ�� - i��Ԫ��*j��i��Ԫ��  
				for (k = 0; k < n; k++)
				{
					t[j][k] = t[j][k] - t[i][k] * temp;
					B[j][k] = B[j][k] - B[i][k] * temp;
				}
			}
		}
	}


	FreeMemory(t,n);
}


/*************************************************************
* @name Matrix
* @brief ׷�Ϸ�������Է�����
* @constantTerm ���Է�����Ⱥ��ұߵ��о���
* @solution ��
* @num ����������
* @m ϵ������Խ�����
* @n �Խ��Ϸ�����
* @k �Խ��·�����
* ***********************************************************/

 void Matrix(float* constantTerm, int num,  float* m,  float* n,  float* k,  float* solution)
{
	//bΪ�ֽ��������Ǿ���ĶԽ�����
	float* a = NULL;
	a = (float *)malloc(sizeof(float)* num);
	//aΪ�ֽ��ĵ�λ�����Ǿ���ĶԽ��Ϸ�����
	float* b = NULL;
	b = (float *)malloc(sizeof(float)* (num - 1));
	//cΪ�ֽ��ĵ�λ�����Ǿ���ĶԽ��Ϸ�����
	float* c = NULL;
	c = (float *)malloc(sizeof(float)* num);
	//xΪ�������еļ�ӽ�
	float* x = NULL;
	x = (float *)malloc(sizeof(float)* num);
	int i;

	a[0] = m[0];
	b[0] = n[0] / a[0];

	//���ֽ�������Ǿ���ĶԽ��·�����c��ֵ
	for (i = 1; i < num; i++)
	{
		c[i] = k[i];
	}


	//���ֽ��ĵ�λ�����Ǿ���ĶԽ��Ϸ�����a�ͷֽ��ĵ�λ�����Ǿ���ĶԽ��Ϸ�����b��ֵ
	for (i = 1; i < num - 1; i++)
	{
		a[i] = m[i] - k[i] * b[i - 1];
		b[i] = n[i] / a[i];

	}

	a[num - 1] = m[num - 1] - k[num - 1] * b[num - 2];
	//�м��x�ĳ�ʼֵ
	x[0] = constantTerm[0] / a[0];

	//���м�⸳ֵ
	for (i = 1; i < num; i++)
	{
		x[i] = (constantTerm[i] - k[i] * x[i - 1]) / a[i];
	}

	//������ս�
	solution[num - 1] = x[num - 1];

	for (i = num - 1; i > 0; i--)
	{
		solution[i - 1] = x[i - 1] - solution[i] * b[i - 1];
	}
	free(a);
	free(b);
	free(c);
	free(x);
}





