/********************************************************************
*Copyright(C��2014-2016,��������������˼��������������ι�˾
*FileName��	   roundView.c
*Author��      Peng Xu
*Date��        2016/10/28
*Description�� Բ����ҰѲ������
*		       
*
*Version��     V1.0
*
********************************************************************/



#include "pathFollowing.h"
#include "math.h"
#include "ringBuffer.h"
#include "Bspline.h"
#include "posSystem.h"
#include "MotionCard.h"
#include "Move.h"


extern int jishuuu;


/*********************************************************************************
* @name 	PathFollowingNew
* @brief	·�����溯��
* @param	percent �ٶȵİٷֱȣ���Ϊ1����100%���滮���ٶ����С���ΧΪ����0,�������1���ᳬ�������˳����ٶ�
* @retval	��
**********************************************************************************/
int PathFollowing(float percent,float Kp,float viewPercent)
{
	static float vell = 200.0f;
	float angle1 = 0.0f;
	float angularVel = 0.0f;
	float angleErr = 0.0f;
	float posAngle = 0.0f;
	float robotlen = 0.0f;
	static float disRealPos2VirTarget = 0.0f;
	static float disRealPos2VirPos = 0.0f;
	static PointU_t virtualPos,virtualTarget;
	Pose_t presentLine;
	
	//��ǰ��������λ�õ���� �� ����λ�õ㵽����Ŀ������֮��
	float VIEW_L = 0.0f;
	
	if(percent < 0 || percent >1)
	{
		return -1;
	}
	
	
	presentLine = GetPosPresent();
	
	//���ݲɼ��ļ�ʮ�����ݻ��ƶ���
	VIEW_L = (8.333e-07f * vell * vell * vell + 0.0006667f * vell * vell - 0.2988f * vell + 107.1f) * viewPercent;
	VIEW_L = VIEW_L > 500 ? 500 : VIEW_L;
	VIEW_L = VIEW_L < 100 ? 100 : VIEW_L;
	
	//��ȡ��λϵͳ������Ļ�����ʵ������·������
	robotlen = GetPath();

	//����λ�õ�
	virtualPos = SerchVirtualPoint(robotlen);
	
	//���㵱ǰ�㵽����λ�õ�ľ���
	disRealPos2VirPos = CalculatePoint2PointDistance(presentLine.point,virtualPos.point);
	
	
	if(GetPath() + VIEW_L - disRealPos2VirPos  > robotlen)
	{
		robotlen = GetPath() + VIEW_L - disRealPos2VirPos;
	}
	
	//��ȡ����Ŀ���
	virtualTarget = SerchVirtualPoint2(robotlen);
	
	//����ʵ��λ�þ�������Ŀ������
	disRealPos2VirTarget = CalculatePoint2PointDistance(presentLine.point,virtualTarget.point);

	float disAdd = (VIEW_L - disRealPos2VirPos) - disRealPos2VirTarget;
	if(disAdd > 0)
	{
		AddPath(2*disAdd);
	}
	else
	{
		//���������Χ��ֹͣ���¾��룬����Ŀ��㲻��
		if(disRealPos2VirPos > VIEW_L)
		{
			UpdateLenStop();
		}
		else
		{
			UpdateLenBegin();
		}
	}
	
	//���㵱ǰ�㵽Ŀ��㷽��Ƕ�
	angle1 = CalculateLineAngle(presentLine.point,virtualTarget.point);
	
	
	//���˵�֮��ǶȵĲ�ֵ
	angleErr = CalculateAngleSub(GetRingBufferPointPoseAngle(2) , GetRingBufferPointPoseAngle(1));
	
	
	//��¼��ʱ�̵�Ŀ��Ƕ�
	posAngle = CalculateAngleAdd(GetRingBufferPointPoseAngle(1),angleErr*virtualPos.u);
	
	angularVel = AngleControl(presentLine.direction,posAngle);
	
	vell = GetRingBufferPointVell(1)+(GetRingBufferPointVell(2) - GetRingBufferPointVell(1))*virtualPos.u;
	
	vell = vell*percent;

	float time;
	
	//�ö�����������β�ĽǶȲ�
	float angErr = GetRingBufferPointPoseAngle(2) - GetRingBufferPointPoseAngle(1);
	angErr = angErr > 180.0f ? angErr - 360.0f : angErr;
	angErr = angErr < -180.0f ? 360.0f + angErr : angErr;
	
	//���Լ���ÿ��ʾ�̵�֮����˶���ʱ��
	time = (GetRingBufferPointLen(2) - GetRingBufferPointLen(1)) / (GetRingBufferPointVell(2) + GetRingBufferPointVell(1)) * 2;
	
	
//	UsartSendXP3(posAngle,GetAngleZ(),disRealPos2VirPos);
	if(GetPath() < GetLength())
	{
		ThreeWheelVelControl(vell,angle1,angularVel + angErr/time*percent);
	}
	else
	{
		virtualTarget.point = GetRingBufferPoint(GetCount());
		//���㵱ǰ�㵽Ŀ��㷽��Ƕ�
		angle1 = CalculateLineAngle(presentLine.point,virtualTarget.point);
		
		angularVel = AngleControl(presentLine.direction,GetRingBufferPointPoseAngle(GetCount()));
		
		
		//����ʵ��λ�þ�������Ŀ������
		disRealPos2VirTarget = CalculatePoint2PointDistance(presentLine.point,virtualTarget.point);
		
		//�������������� ͣ��
		ThreeWheelVelControl(Kp*disRealPos2VirTarget,angle1,angularVel);
		if(disRealPos2VirTarget < 10)
		{
			return 2;
		}
	}
	
	return 1;
}




/*********************************************************************************
* @name 	AngleControl
* @brief	�Ƕȱջ����Ƴ���
* @param	anglePresent ��ǰ�ĽǶ� ��λ ��
* @param  angleTarget  Ŀ��Ƕ�   ��λ ��
* @retval	��
**********************************************************************************/
float AngleControl(float anglePresent,float angleTarget)
{
	#define P_ANGLE_CONTROL 1.425f
	float angleErr = 0.0f,angularVel = 0.0f;
	//Ŀ��Ƕȼ�ȥ��ǰ�Ƕ�
	angleErr = CalculateAngleSub(angleTarget,anglePresent);
	
	angularVel = angleErr * P_ANGLE_CONTROL;

	return angularVel;
	
}


















