#include "SpeedPlaning.h"
#include <stdlib.h>
#include "MotionCard.h"
#include "ringBuffer.h"
#include "math.h"
#include "Move.h" 
#include "calculate.h"

//ͨ�����õ����������ٶȽ��н���
//�ʵ������Ľ��٣������ǵð������ٶ����ݷ���ringbuffer��
//wheelOne һ�����ٶ������׵�ַ
//wheelTwo 	�������ٶ������׵�ַ
//wheelThree �������ٶ������׵�ַ
void DynamicalAjusting(float* wheelOne, float* wheelTwo, float* wheelThree)
{
	float time = 0.0f;
	int n = GetCount();
	float tempAcc = 0.0f;

	//ÿ�μ��ٶȽ������ϴεİٷ�ֵ
	float percent = 0.9;

	//�����������ٶ�
	for (int i = 2; i < n + 1; i++)
	{

		//���Լ���ÿ��ʾ�̵�֮����˶���ʱ��
		time = (GetRingBufferPointLen(i) - GetRingBufferPointLen(i - 1)) / (GetRingBufferPointVell(i) + GetRingBufferPointVell(i - 1)) * 2;
		//��1
		//ֻ�����ٶ�ͬ������
		if (wheelOne[i - 1] * wheelOne[i - 2] > 0)
		{
			tempAcc = (fabs(wheelOne[i - 1]) - fabs(wheelOne[i - 2])) / time;
		}
		else
		{
			tempAcc = 0.0f;
		}

		if (tempAcc > GetAccMax())
		{
			//ÿ������0.05�ļ��ٶ�
			wheelOne[i - 1] = wheelOne[i - 1] > 0 ? wheelOne[i - 2] + tempAcc*percent * time : wheelOne[i - 2] - tempAcc*percent * time;
		}
		//��2
		//ֻ�����ٶ�ͬ������
		if (wheelTwo[i - 1] * wheelTwo[i - 2] > 0)
		{
			tempAcc = (fabs(wheelTwo[i - 1]) - fabs(wheelTwo[i - 2])) / time;
		}
		else
		{
			tempAcc = 0.0f;
		}

		if (tempAcc > GetAccMax())
		{
			//ÿ������0.05�ļ��ٶ�
			wheelTwo[i - 1] = wheelTwo[i - 1] > 0 ? wheelTwo[i - 2] + tempAcc*percent * time : wheelTwo[i - 2] - tempAcc*percent * time;
		}
		//��3
		//ֻ�����ٶ�ͬ������
		if (wheelThree[i - 1] * wheelThree[i - 2] > 0)
		{
			tempAcc = (fabs(wheelThree[i - 1]) - fabs(wheelThree[i - 2])) / time;
		}
		else
		{
			tempAcc = 0.0f;
		}

		if (tempAcc > GetAccMax())
		{
			//ÿ������0.05�ļ��ٶ�
			wheelThree[i - 1] = wheelThree[i - 1] > 0 ? wheelThree[i - 2] + tempAcc*percent * time : wheelThree[i - 2] - tempAcc*percent * time;
		}
	}

	//���������ٶ�
	for (int i = n; i > 0; i--)
	{

		//���Լ���ÿ��ʾ�̵�֮����˶���ʱ��
		time = (GetRingBufferPointLen(i) - GetRingBufferPointLen(i - 1)) / (GetRingBufferPointVell(i) + GetRingBufferPointVell(i - 1)) * 2;
		//��1
		//ֻ�����ٶ�ͬ������
		if (wheelOne[i - 1] * wheelOne[i - 2] > 0)
		{
			tempAcc = (fabs(wheelOne[i - 1]) - fabs(wheelOne[i - 2])) / time;
		}
		else
		{
			tempAcc = 0.0f;
		}

		if (tempAcc < -GetAccMax())
		{
			//ÿ������0.05�ļ��ٶ�
			wheelOne[i - 2] = wheelOne[i - 2] > 0 ? wheelOne[i - 1] - tempAcc*percent * time : wheelOne[i - 1] + tempAcc*percent * time;
		}


		//��2
		//ֻ�����ٶ�ͬ������
		if (wheelTwo[i - 1] * wheelTwo[i - 2] > 0)
		{
			tempAcc = (fabs(wheelTwo[i - 1]) - fabs(wheelTwo[i - 2])) / time;
		}
		else
		{
			tempAcc = 0.0f;
		}

		if (tempAcc < -GetAccMax())
		{
			//ÿ������0.05�ļ��ٶ�
			wheelTwo[i - 2] = wheelTwo[i - 2] > 0 ? wheelTwo[i - 1] - tempAcc*percent * time : wheelTwo[i - 1] + tempAcc*percent * time;
		}


		//��3
		//ֻ�����ٶ�ͬ������
		if (wheelThree[i - 1] * wheelThree[i - 2] > 0)
		{
			tempAcc = (fabs(wheelThree[i - 1]) - fabs(wheelThree[i - 2])) / time;
		}
		else
		{
			tempAcc = 0.0f;
		}

		if (tempAcc < -GetAccMax())
		{
			//ÿ������0.05�ļ��ٶ�
			wheelThree[i - 2] = wheelThree[i - 2] > 0 ? wheelThree[i - 1] - tempAcc*percent * time : wheelThree[i - 1] + tempAcc*percent * time;
		}
	}



	for (int i = 0; i < n - 1; i++)
	{
		TriWheelVel_t tempTrueVell;
		TriWheelVel2_t tempVel2;
		tempTrueVell.v1 = wheelOne[i];
		tempTrueVell.v2 = wheelTwo[i];
		tempTrueVell.v3 = wheelThree[i];
		
		tempVel2 = GetTrueVell(tempTrueVell,GetRingBufferPointPoseAngle(i+1));
		SetRingBufferPointVell(i + 1, tempVel2.speed);
	}

}










//ͨ��ringBuffer������ݼ���ÿһ�㴦�������ӵ��ٶ�
//Ŀ�ĸ���wheelOne wheelTwo wheelThree������������������ٶȣ�������һ�ε��ٶ�����
//wheelOne һ�����ٶ������׵�ַ
//wheelTwo 	�������ٶ������׵�ַ
//wheelThree �������ٶ������׵�ַ
void CalculateThreeWheelVell(float* wheelOne,float* wheelTwo,float* wheelThree)
{
	//�ֽ⵽�����ֶ�ȫ���ٶȽ��й滮
	TriWheelVel_t threeVell;
	float n = GetCount();


	for (int i = 2; i < n + 1; i++)
	{
		float angErr = GetRingBufferPointPoseAngle(i) - GetRingBufferPointPoseAngle(i - 1);
		angErr = angErr > 180 ? angErr - 360 : angErr;
		angErr = angErr < -180 ? 360 + angErr : angErr;

		float time = 0.0f;

		time = (GetRingBufferPointLen(i) - GetRingBufferPointLen(i - 1)) / (GetRingBufferPointVell(i) + GetRingBufferPointVell(i - 1)) * 2;

		float rotationVell = angErr / time;


		threeVell = CaculateThreeWheelVel(GetRingBufferPointVell(i), GetRingBufferPointAngle(i), rotationVell, GetRingBufferPointPoseAngle(i));


		wheelOne[i - 1] = threeVell.v1;

		wheelTwo[i - 1] = threeVell.v2;

		wheelThree[i - 1] = threeVell.v3;

	}
	wheelOne[0] = wheelOne[1];
	wheelTwo[0] = wheelTwo[1];
	wheelThree[0] = wheelThree[1];
	
}





//ͨ�����ͺ��ٶȱ�֤ĳ�ֵ��ٶ�Ҫ��
//vellCar ����ǰ��ǰ�����ٶ� ��λ mm/s
//orientation �ٶȳ��� ��λ ��
//rotationalVell ��ת�ٶ� ��λ ��ÿ��
//wheelNum  �����ٵ��ֺ� 
// targetWheelVell   �����ٵ�Ŀ��
// ���������ͺ�ĺ��ٶ�
float DecreseVellByOneWheel(float vellCar, float orientation, float rotationalVell,float zAngle, int wheelNum, float targetWheelVell)
{
	TriWheelVel_t vell;
	int i;
	switch (wheelNum)
	{
		case 1:
			//ÿ�κ��ٶȳ�0.9,ֱ������һ�����ٶȽ�����Ŀ���ٶȡ�����һЩ��������ģ�ѭ��10�κ��Զ��˳�
			for (i = 0; i < 10; i++)
			{

				vellCar *= 0.9f;
				vell = CaculateThreeWheelVel(vellCar, orientation, rotationalVell, zAngle);
				if (fabs(vell.v1) < fabs(targetWheelVell))
				{
					break;
				}
			}
		break;
		
		case 2:
			for (i = 0; i < 10; i++)
			{

				vellCar *= 0.9f;
				vell = CaculateThreeWheelVel(vellCar, orientation, rotationalVell, zAngle);
				if (fabs(vell.v2) < fabs(targetWheelVell))
				{
					break;
				}
			}
		break;

		case 3:
			for (i = 0; i < 10; i++)
			{

				vellCar *= 0.9f;
				vell = CaculateThreeWheelVel(vellCar, orientation, rotationalVell, zAngle);
				if (fabs(vell.v3) < fabs(targetWheelVell))
				{
					break;
				}
			}
			break;
	}
	return vellCar;
}

#define MIN_VELL 70
//�ٶȹ滮����
void SpeedPlaning()
{
	float* vell = NULL;
	float* curvature = NULL;
	float* wheelOne = NULL;
	float* wheelTwo = NULL;
	float* wheelThree = NULL;


	int n = GetCount();

	vell = (float *)malloc(n*sizeof(float));
	curvature = (float *)malloc(n*sizeof(float));
	wheelOne = (float *)malloc(n*sizeof(float));
	wheelTwo = (float *)malloc(n*sizeof(float));
	wheelThree = (float *)malloc(n*sizeof(float));


	for (int i = 0; i < n; i++)
	{
		curvature[i] = GetRingBufferAverCurvature(i + 1);
	}
	curvature[n - 1] = curvature[n - 2];
	//	curvature[0] = curvature[1];


	//�������ٶ��ܹ��������С���ʰ뾶
	float curvatureMaxVell = GetVelMax() * GetVelMax() / (2 * GetAccMax());


	//����������ʰ뾶ȫ��Ϊ������ٶ����������С���ʰ뾶
	for (int i = 0; i < n; i++)
	{
		if (curvature[i] > curvatureMaxVell)
		{
			curvature[i] = curvatureMaxVell;
		}
	}
	curvature[0] = curvature[1];
	curvature[n - 1] = curvature[n - 2];


	//ͨ�����ʰ뾶����ö������������ٶ�                                         
	for (int i = 0; i < n; i++)
	{
		vell[i] = sqrt((2 * GetAccMax()) * curvature[i]);
	}




	float tempVell = 0.0f;
	//ͨ��v2^2 - v1^2 = 2*a*s���ٶ��ٴι滮
	for (int i = 0; i < n - 1; i++)
	{
		if (vell[i + 1] > vell[i])
		{
			tempVell = sqrt(2 * (1 * GetAccMax()) * (GetRingBufferPointLen(i + 2) - GetRingBufferPointLen(i + 1)) + vell[i] * vell[i]);
			if (tempVell < vell[i + 1])
			{
				vell[i + 1] = tempVell;
			}
		}
	}

	for (int i = n - 1; i > 0; i--)
	{
		if (vell[i - 1] > vell[i])
		{
			tempVell = sqrt(2 * (1 * GetAccMax()) * (GetRingBufferPointLen(i + 1) - GetRingBufferPointLen(i)) + vell[i] * vell[i]);
			if (tempVell < vell[i - 1])
			{
				vell[i - 1] = tempVell;
			}
		}
	}

	//����ʱ�滮���ٶȷ��뻷��������
	for (int i = 0; i < n; i++)
	{
		SetRingBufferPointVell(i + 1, vell[i]);
	}


	//�����ʱ�����ֵ��ٶ�
	CalculateThreeWheelVell(wheelOne, wheelTwo, wheelThree);

	//��̬�Ķ��ٶȽ���ƽ��
	while (1)
	{
		int ipoint = 0;

		for (ipoint = 3; ipoint < n; ipoint++)
		{
			float time = 0.0f;

			float lll;
			float vvv;
			lll = (GetRingBufferPointLen(ipoint) - GetRingBufferPointLen(ipoint - 1));
			vvv = (GetRingBufferPointVell(ipoint) + GetRingBufferPointVell(ipoint - 1)) / 2;
			time = lll / vvv;

			
			float a1, a2, a3;
			//����ж�ĳһ�����Ӽ��ٶȴ��������ٶ�ʱ�����е���

			a1 = (wheelOne[ipoint - 1] - wheelOne[ipoint - 2]) / time;
			a2 = (wheelTwo[ipoint - 1] - wheelTwo[ipoint - 2]) / time;
			a3 = (wheelThree[ipoint - 1] - wheelThree[ipoint - 2]) / time;
			if (((a1 > GetAccMax()) && (wheelOne[ipoint - 1] * wheelOne[ipoint - 2] > 0)) \
				|| ((a2 > GetAccMax()) && (wheelTwo[ipoint - 1] * wheelTwo[ipoint - 2] > 0))\
				|| ((a3 > GetAccMax()) && (wheelThree[ipoint - 1] * wheelThree[ipoint - 2] > 0)))
			{
				//ƽ�ⷨ�滮�ٶ�
				DynamicalAjusting(wheelOne, wheelTwo, wheelThree);

				break;
			}
		}
	

		if (ipoint == n)
		{

			for (int i = 1; i < n; i++)
			{
				TriWheelVel_t tempTrueVell;
				tempTrueVell.v1 = wheelOne[i];
				tempTrueVell.v2 = wheelTwo[i];
				tempTrueVell.v3 = wheelThree[i];
				float vellCar1, vellCar2, vellCar3, vellCar;
				float angErr = GetRingBufferPointPoseAngle(i + 2) - GetRingBufferPointPoseAngle(i + 1);
				angErr = angErr > 180 ? angErr - 360 : angErr;
				angErr = angErr < -180 ? 360 + angErr : angErr;
				//���Լ���ÿ��ʾ�̵�֮����˶���ʱ��
				float time = (GetRingBufferPointLen(i + 2) - GetRingBufferPointLen(i + 1)) / (GetRingBufferPointVell(i + 2) + GetRingBufferPointVell(i + 1)) * 2;

				vellCar1 = DecreseVellByOneWheel(GetRingBufferPointVell(i + 1), GetRingBufferPointAngle(i + 1), angErr / time, GetRingBufferPointPoseAngle(i + 1), 1, tempTrueVell.v1);

				vellCar2 = DecreseVellByOneWheel(GetRingBufferPointVell(i + 1), GetRingBufferPointAngle(i + 1), angErr / time, GetRingBufferPointPoseAngle(i + 1), 2, tempTrueVell.v2);

				vellCar3 = DecreseVellByOneWheel(GetRingBufferPointVell(i + 1), GetRingBufferPointAngle(i + 1), angErr / time, GetRingBufferPointPoseAngle(i + 1), 3, tempTrueVell.v3);

				if (fabs(vellCar1) >= fabs(vellCar2) && fabs(vellCar1) >= fabs(vellCar3))
				{
					vellCar = vellCar1;
					//����������º��ٶȷ��뻺�����
					SetRingBufferPointVell(i + 1, vellCar);
				}
				else if (fabs(vellCar2) >= fabs(vellCar1) && fabs(vellCar2) >= fabs(vellCar3))
				{
					vellCar = vellCar2;
					//����������º��ٶȷ��뻺�����
					SetRingBufferPointVell(i + 1, vellCar);
				}
				else if (fabs(vellCar3) >= fabs(vellCar2) && fabs(vellCar3) >= fabs(vellCar1))
				{
					vellCar = vellCar3;
					//����������º��ٶȷ��뻺�����
					SetRingBufferPointVell(i + 1, vellCar);
				}
			}
		}


		if (ipoint == n)
		{
			SetRingBufferPointVell(1, 100);
			SetRingBufferPointVell(n, 100);
			for (int i = 0; i < n - 1; i++)
			{
				if (GetRingBufferPointVell(i + 2) > GetRingBufferPointVell(i + 1))
				{
					tempVell = sqrt(2 * (1.0f * GetAccMax()) * (GetRingBufferPointLen(i + 2) - GetRingBufferPointLen(i + 1)) + GetRingBufferPointVell(i + 1) * GetRingBufferPointVell(i + 1));
					if (tempVell < GetRingBufferPointVell(i + 2))
					{
						SetRingBufferPointVell(i + 2, tempVell);
					}
				}
			}

			

			for (int i = n - 1; i > 0; i--)
			{
				if (GetRingBufferPointVell(i) > GetRingBufferPointVell(i + 1))
				{
					tempVell = sqrt(2.0f * (1.0f * GetAccMax()) * (GetRingBufferPointLen(i + 1) - GetRingBufferPointLen(i)) + GetRingBufferPointVell(i + 1) * GetRingBufferPointVell(i + 1));
					if (tempVell < GetRingBufferPointVell(i))
					{
						SetRingBufferPointVell(i, tempVell);
					}
				}
			}

			//���ٶ�С����С�ٶȵ�������
			for (int i = 2; i < n; i++)
			{
				if (GetRingBufferPointVell(i) < MIN_VELL)
				{
					SetRingBufferPointVell(i, MIN_VELL);
				}
			}


			free(wheelOne);
			free(wheelTwo);
			free(wheelThree);
			break;
		}
	}

}






