/********************************************************************
*Copyright(C��2014-2016,��������������˼��������������ι�˾
*FileName��	   Sampling.c
*Author��      Peng Xu
*Date��        2016/10/28
*Description�� ������ֵ������
*		       
*
*Version��     V1.0
*
********************************************************************/

//����FLASH �����ַ(����Ϊż��������������,Ҫ���ڱ�������ռ�õ�������.
//����,д������ʱ��,���ܻᵼ�²�����������,�Ӷ����𲿷ֳ���ʧ.��������.
//���鳤�ȴ�ŵ�ַ
#define FLASH_NUM  0x08040000 	
//������Ϣ��ŵ�ַ
#define FLASH_INFORMATION  0x08040004



#include "Sampling.h"
#include "math.h"
#include "MotionCard.h"
#include "ringBuffer.h"
#include "Bspline.h"
#include "posSystem.h"
#include "flash.h"
#include "SpeedPlaning.h"

//��ʱ��10ms��һ��
_Bool USARTSEND = 0;
/*********************************************************************
* @name 		FunSampling
* @brief  	��ȡλ�˻��ȡ���겢���ɵ㷽��
* @param  	��
* @retval 	��
********************************************************************/
int PoseSampling(void)
{
	static _Bool FIRST_FLAG = 0;
	static Point_t point1={0,0},point2;
	static float anglee,angleeOld;
	static float posAngle = 0.0f;
	KeyPointInf_t pos;
	static float pathOld;
	static int step = 1;
	float tempLenth = 0.0f;
	//�Ե�һ��������
	if(FIRST_FLAG == 0)
	{
		if(GetPath() > 50)
		{
			//�����ʸ������
			anglee = CalculateLineAngle(point1,GetPosPresent().point);
			//�����һ��
			pos.point = point1;
			pos.angle = anglee;
			pos.poseAngle = GetAngleZ();
			pos.length = 0;
			//��һ���Ȳ������ʰ뾶��ֵ
			PutRingBuffer(pos);
			
			angleeOld = anglee;
			pathOld = 0.0f;
			FIRST_FLAG = 1;
		}
	}
	else
	{
		switch(step)
		{
			//��ȡ����ͽǶ�
			case 1:
					if((GetPath()-pathOld) > 150.0f)
					{
						point2  = GetPosPresent().point;
						posAngle= GetAngleZ();
						pathOld = GetPath();
						step = 2;
					}
					

				break;
			
			//�����һ�����ɵ����ߵ����߷���
			case 2:
					if((GetPath()-pathOld) > 5.0f)
					{
						anglee =  CalculateLineAngle(point2,GetPosPresent().point);
						//����������ߵ����߳���
						tempLenth = CaculateBsplineLen(point1,point2,angleeOld,anglee);
						
						pos.point = point2;
						pos.poseAngle = posAngle;
						pos.angle = anglee;

						//�����ܾ��볤��
						pos.length = tempLenth + GetLength();
						
						//��¼�öε����ʰ뾶
						//��ֱ��������
						if(fabs(CalculateAngleSub(angleeOld,anglee)) < 0.01f)
						{
							pos.curvatureR = tempLenth/0.0001f;
						}
						else
						{
							pos.curvatureR = fabs(tempLenth/(CalculateAngleSub(angleeOld,anglee)*CHANGE_TO_RADIAN));
						}
						
						SetLength(pos.length);
						PutRingBuffer(pos);
						
						angleeOld = anglee;
						point1 = point2;
						pathOld = GetPath();
						step = 1;
					}
				break;
					
			
			default:
				break;
		}
	}
	return 1;
}


void PoseSamplingLastPoint(void)
{
		Point_t point2,point1;
		KeyPointInf_t pos;
		float tempLenth = 0.0f;
	
		int n = GetCount();
		point1 = GetRingBufferPoint(n);
		point2  = GetPosPresent().point;

		tempLenth = CalculatePoint2PointDistance(point1,point2);
		
		pos.point = point2;
	
		pos.poseAngle = GetAngleZ();
	
		pos.angle = GetRingBufferPointAngle(n);

		//�����ܾ��볤��
		pos.length = tempLenth + GetLength();
		
		//��¼�öε����ʰ뾶
		pos.curvatureR = 10000.0f;

		SetLength(pos.length);
		PutRingBuffer(pos);
}




/*********************************************************************
* @name 		CaculateBeginAngle
* @brief  	���������ʼʱ��һ��͵ڶ���ķ���
* @param  	point1 ��ʼ��
* @param    point2 ������
* @retval 	������ʼ��ͽ��������̬�Ƕ�
********************************************************************/
float CaculateBeginAngle(Point_t point1,Point_t point2)
{
	Point_t direction;
	direction.x = point2.x - point1.x;
	direction.y = point2.y - point1.y;
	return CaculateDirectionAngle(direction);
}




/*********************************************************************
* @name 		CaculateProcessAngle
* @brief  	������������ɽ�������������ķ���
* @param  	point1 ��ʼ��
* @param    point2 ������
* @param    angleLast  ��ʼ��ķ���
* @retval 	����ʸ���ĽǶ� -180~180
********************************************************************/
float CaculateProcessAngle(Point_t point1, Point_t point2, float angleLast)
{
	//������
	Point_t P[2];
	//��ֵ��
	//ϵ������������
	float invA[2][2] = { { 0.333333333f, 0.00000f }, { 0.166666667f, 0.055555555f } };
	//���Ƶ�����ʸ�������ǰ��
	float length = 0.0f;
	//��ʱ���Ƶ�����
	Point_t dataPoint[2];
	//���տ��Ƶ�����
	Point_t finalDataPoint[6];
	Point_t direction;
	int i, j;


	//����ֵ���ڴ濪��																		
	//passPoint = (Point_t*)malloc(sizeof(Point_t) * 30);


	length = sqrt((point1.x - point2.x)*(point1.x - point2.x) + (point1.y - point2.y)*(point1.y - point2.y));


	//�Գ����ֵ
	P[0].x = 3 * point1.x + length*cos((angleLast) / 180.0f * 3.1415926f);
	P[0].y = 3 * point1.y + length*sin((angleLast) / 180.0f * 3.1415926f);
	P[1].x = 9 * point2.x;
	P[1].y = 9 * point2.y;


	//����
	for (i = 0; i < 2; i++)
	{
		dataPoint[i].x = 0;
		dataPoint[i].y = 0;
	}



		for (j = 0; j < 2; j++)
		{
			dataPoint[1].x += invA[1][j] * P[j].x;
			dataPoint[1].y += invA[1][j] * P[j].y;
		}



	finalDataPoint[3] = dataPoint[1];

	finalDataPoint[5] = point2;


	direction.x = -0.5f*finalDataPoint[3].x + 0.5f*finalDataPoint[5].x;
	direction.y = -0.5f*finalDataPoint[3].y + 0.5f*finalDataPoint[5].y;
	
	return CaculateDirectionAngle(direction);

}





/*********************************************************************
* @name 		CaculateDirectionAngle
* @brief  	����ʸ����ƽ������ϵ�еķ���Ƕ�
* @param  	direction ����ʸ��
* @retval 	����ʸ���ĽǶ� -180~180
********************************************************************/
float CaculateDirectionAngle(Point_t direction)
{
	Point_t OriginalPoint = {0,0};
 	return CalculateLineAngle(OriginalPoint,direction);
}





//��ȡflash��Ϣ
int ReadFlashPointInformation(void)
{
	u32 numCount;
	
	//��ȡ�����
	STMFLASH_Read(FLASH_NUM,&numCount,1);

	//��ȡ����Ϣ��ringBuffer
	STMFLASH_Read(FLASH_INFORMATION,GetFristAdress(),numCount*7);
	
	//���õ����
	SetUpPointer(numCount);
	
	//�����һ���ԭ��ĳ��ȼ�¼Ϊ�ܳ���
	SetLength(GetRingBufferPointLen(numCount));
	
	return 1;
}



//�������
int PoseSamplingDone(void)
{
	u32 numCount;
	PoseSamplingLastPoint();
	//��ȡ�����
	numCount = GetCount();
	
	
	//���滮���ٶȴ���ringBuffer
	SpeedPlaning();
	//�������д��flash
	STMFLASH_Write(FLASH_NUM,&numCount,1);
	//������Ϣд��flash
	STMFLASH_Write(FLASH_INFORMATION,GetFristAdress(),numCount*7);
	
	return 1;
}












