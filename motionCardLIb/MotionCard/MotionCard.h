#ifndef _MOTIONCARD_H
#define _MOTIONCARD_H

//ע�� ʾ��ģʽ�������洢�� 0x08040000 ��ַ��  stm32f407��Ƭ��



//��Ľṹ�� ��λmm
typedef struct
{
	float x;
	float y;
}Point_t;



typedef struct 
{
	float v1;
	float v2;
	float v3;
}TriWheelVel_t;



//��бʽ�ṹ�� ��б���ýǶ��ƵĽǶȴ���
typedef struct
{
	Point_t point;
	//�Ƕ���
	float   direction;
}Pose_t;


extern void VelControlTriWheel(float v1,float v2,float v3);

extern float GetRobotRadius(void);


//ACCMAXĦ�����͵�����ṩ�������ٶ� mm/s^2
extern float GetAccMax(void);
//VELLMAX�������ܴﵽ������ٶ� mm/s  ��ֵ��ͨ���Ƚ�Ħ�����ṩ�������ٶ��������ṩ�������˵������ٶȣ�ȡ��Сֵ��
extern float GetVelMax(void);



//��Ҫ�ⲿʵ�֣������������̬
extern float GetAngleZ(void);
extern float GetPosx(void);
extern float GetPosy(void);


//�����ڴ棬ʹ��ǰ�������ȵ���  num ����ȡ�������һ����ռ28�ֽ�  ;����1�����ٳɹ�
int BufferZizeInit(int num);

//·�����溯�� percent: ��ΧΪ0~1.1����滮������ٶ�;���� -1���������������Χ������1����ɹ�����
int PathFollowing(float percent);

//ʾ�̲ɵ㺯�����ɵ�ʱ����Ҫһֱ����;����1����ɹ��ɼ�
int PoseSampling(void);

//��ȡflash���ʾ�̵���Ϣ������ӵ�ringBuffer��;  ���� 1 ����ɹ���ȡ
int ReadFlashPointInformation(void);

//�ɵ����ʱ��Ҫ���ã������ɵ���Ϣ����flash��;���� 1 ����ɹ���ȡ
int PoseSamplingDone(void);

//���ringBuffer,һ����գ����̽���ͣ������
void ClearRingBuffer(void);

//����ڴ�غ������µĹؼ��㣬����·���滮; ����1�������ɹ�ִ��
int InputPoints2RingBuffer(Pose_t *points,int num);

//�ú������ڼ�¼���������ߵ�·��������ȫ�̵��ã�����
void CaculatePath(void);

////���˵�������
//int GetCount(void);

//float GetPredictTime(void);

//float GetRingBufferPointPoseAngle(int num);
//float GetRingBufferPointAngle(int num);
//float GetRingBufferPointVell(int num);
//float GetRingBufferPointLen(int num);
//float GetRingBufferAverCurvature(int num);
//float GetLength(void);
//Point_t GetRingBufferPoint(int num);
#endif



