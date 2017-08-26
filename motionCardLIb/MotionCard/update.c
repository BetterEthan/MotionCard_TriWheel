/********************************************************************
*Copyright(C��2014-2016,��������������˼��������������ι�˾
*FileName��	   update.c
*Author��      Peng Xu
*Date��        2016/10/28
*Description�� ���ڸ���������������
*		       
*
*Version��     
*
********************************************************************/




#include "update.h"
#include "stdint.h"
#include "math.h"

static float g_zAngle		    = 0;   //������z����Ƕ� ��Χ-180~180
static float g_PosX   			= 0;	 //��λϵͳ���ص�X����
static float g_PosY   			= 0;	 //��λϵͳ���ص�Y����
static int   g_VelLft 			= 0;   //�����ٶ�
static int   g_VelRgt 			= 0;	 //�����ٶ�



/***********************************************************************************
* @name 		SetAngleZ
* @brief  	�ж϶�ȡ�趨zangleֵ
* @param  	temp
* @retval 	��
**********************************************************************************/
void SetAngleZ(float temp)
{
	g_zAngle = temp;
}






/***********************************************************************************
* @name 		GetAngleZ
* @brief  	��ȡzangleֵ   ��Ϊ���Ƕ�0��ת����x�����������
* @param  	��
* @retval 	�����ǽǶ�
**********************************************************************************/
float GetAngleZ(void)
{
//ע�����ֺ�˫�ֲ�ͬ ˫�ֶ�λϵͳ��0��Ϊy����������Ҫ��Ϊ��90�ȴ������ֶ�λϵͳ0��Ϊx��������
	return g_zAngle;
}






/***********************************************************************************
* @name 		SetPosx
* @brief  	�ж϶�ȡ�趨X����
* @param  	temp
* @retval 	��
**********************************************************************************/
void SetPosx(float temp)
{
	g_PosX = temp;
}






/***********************************************************************************
* @name 		GetPosx
* @brief  	��ȡX����
* @param  	��
* @retval 	X����
**********************************************************************************/

float GetPosx(void)
{
	return g_PosX;
}





/***********************************************************************************
* @name 		SetPosy
* @brief  	�ж϶�ȡ�趨Y����
* @param  	temp
* @retval 	��
**********************************************************************************/
void SetPosy(float temp)
{
	g_PosY = temp;
}





/***********************************************************************************
* @name 		GetPosy
* @brief  	��ȡY����
* @param  	��
* @retval 	Y����
**********************************************************************************/

float GetPosy(void)
{
	return g_PosY;
}





/***********************************************************************************
* @name 		SetVelLft
* @brief  	���������ٶ�
* @param  	temp
* @retval 	��
**********************************************************************************/
void SetVelLft(float temp)
{
	g_VelLft = temp;
}





/***********************************************************************************
* @name 		GetVelLft
* @brief  	��ȡ�����ٶ�
* @param  	��
* @retval 	g_VelLft
**********************************************************************************/
int GetVelLft(void)
{
	return g_VelLft;
}





/***********************************************************************************
* @name 		SetVelRgt
* @brief  	���������ٶ�
* @param  	temp
* @retval 	��
**********************************************************************************/
void SetVelRgt(float temp)
{
	g_VelRgt = temp;
}





/***********************************************************************************
* @name 		GetVelRgt
* @brief  	��ȡ�����ٶ�
* @param  	��
* @retval 	g_VelRgt
**********************************************************************************/
int GetVelRgt(void)
{
	return g_VelRgt;
}





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
void CaculatePath(void)
{
	static float posXOld = 0.0f,posYOld = 0.0f;
	float err;
	
	if(CaculateLenFlag == 1)
	{
		err = sqrt((GetPosx() - posXOld)*(GetPosx() - posXOld) + (GetPosy() - posYOld)*(GetPosy() - posYOld));
	
//		USART_OUT(SENDUSART,(uint8_t *)"**********\r\n");
		if(err> 0.5f)
		{
			lengthTwoWheel += err;
		}
	}
	posXOld = GetPosx();
	posYOld = GetPosy();
}

void UpdateLenBegin(void)
{
	CaculateLenFlag = 1;
}


void UpdateLenStop(void)
{
	CaculateLenFlag = 0;
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
ActLine2 GetPosPresent(void)
{
	ActLine2 pos;
	pos.point.x = g_PosX;
	pos.point.y = g_PosY;
	pos.angle   = g_zAngle;
	return pos;
}

//���Ӿ���
void AddPath(float dis)
{
	lengthTwoWheel += dis;
}


static	ActTrueVell s_trueVell;


//ͨ����λϵͳ�������õ�ʵ���ٶ�
//��Ҫ10ms����һ��
void CcltPoseVell(void)
{
	static ActPoint poseOld;
	static float angleOld;
	s_trueVell.vell = CcltPoint2PointDistance(poseOld,GetPosPresent().point) / 0.01f;
	s_trueVell.angle = CHANGE_TO_ANGLE * atan2f(GetPosy()-poseOld.y,GetPosx()-poseOld.x);
	s_trueVell.angularVell = (GetAngleZ() - angleOld) / 0.01f;
	poseOld = GetPosPresent().point;
	angleOld = GetAngleZ();
}




ActTrueVell GetPoseVell(void)
{
	return s_trueVell;
}








