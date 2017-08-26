#include "posSystem.h"
#include "MotionCard.h"
#include "math.h"


 


/***********************************************************************************
* @name 		CaculatePath
* @brief  	�����߹������
* @param  	
* @retval 	
**********************************************************************************/
static float lengthTwoWheel = 0.0f;
//0����¼
//1��¼����
//Ĭ�ϳ�ʼ�Ϳ�ʼ��¼����
static _Bool CaculateLenFlag = 1;


//ͨ����λϵͳ������������ߵ�·������
void CaculatePath(void)
{
	static float posXOld = 0.0f,posYOld = 0.0f;
	float err = -0.02f;
	
	if(CaculateLenFlag == 1)
	{
		err = sqrt((GetPosx() - posXOld)*(GetPosx() - posXOld) + (GetPosy() - posYOld)*(GetPosy() - posYOld));
		if(err> 0.5f)
		{
			lengthTwoWheel += err;
		}

	}

	posXOld = GetPosx();
	posYOld = GetPosy();
}


//����·�����ȼ�¼
void UpdateLenBegin(void)
{
	CaculateLenFlag = 1;
}


//ֹͣ·�����ȼ�¼
void UpdateLenStop(void)
{
	CaculateLenFlag = 0;
}




//���Ӿ���
void AddPath(float dis)
{
	lengthTwoWheel += dis;
}



/***********************************************************************************
* @name 		GetPath
* @brief  	���������
* @param  	��
* @retval 	lengthTwoWheel
**********************************************************************************/
int GetPath(void)
{
	return lengthTwoWheel;
}



/***********************************************************************************
* @name 		GetPosPresent
* @brief  	���ص�ǰ��̬
* @param  	��
* @retval 	
**********************************************************************************/
Pose_t GetPosPresent(void)
{
	Pose_t pos;
	pos.point.x = GetPosx();
	pos.point.y = GetPosy();
	pos.direction   = GetAngleZ();
	return pos;
}



void ClearPathLen(void)
{
	lengthTwoWheel = 0.0f;
	UpdateLenBegin();
}








