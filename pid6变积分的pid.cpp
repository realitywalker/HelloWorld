//λ����pid, ��ʽ��pid.kp*pid.err + pid.ki*pid.integral + pid.kd*(pid.err-pid.err_last) 

//�����λ��ְ汾��ʼ������ϵ����д������K*��Ϊ��k* 

//���������������������������趨ʱ����ʱ����ϵͳ����кܴ��ƫ������PID����Ļ��ֻ��ۣ�
//���¿���������ִ�л�������������������Χ��Ӧ���޿��������Ӷ�����ϴ�ĳ������������𵴣����Ǿ��Բ������
//���ַ���ĸ���ǵ����������趨ֵƫ��ϴ�ʱ��ȡ����������;���������ӽ�����ֵʱ��������ֿ��ƣ������������߾���

//�����ӵı����pid�У�û��ʹ�ÿ����ֱ��������λ��� 

//�����PID���Կ����ǻ��ַ����PID�㷨�ĸ�һ�����ʽ������ͨ��PID�У�����ϵ��ki�ǳ������������������ƹ����У����������ǲ���ġ�
//���ǣ�ϵͳ���ڻ������Ҫ���ǣ�ϵͳƫ���ʱ����������Ӧ�ü���������ȫ�ޣ�����ƫ��СʱӦ�ü�ǿ��
//����ϵ��ȡ���˻�����������������ֱ��ͣ�ȡС���ֲ��ܶ�ʱ�������������ˣ�����ϵͳ��ƫ���С�ı�����ٶ����б�Ҫ�ġ�
//�����PID�Ļ���˼�����跨�ı��������ۼ��ٶȣ�ʹ����ƫ���С���Ӧ��ƫ��Խ�󣬻���Խ��; ƫ��ԽС������Խ�졣
#include <stdio.h>
#include <stdlib.h>

//1.����PID�ṹ�� 
typedef struct
{
	float setSpeed;		//�����趨ֵ
	float actualSpeed;	//����ʵ��ֵ
	float err;			//����ƫ��ֵ
	float err_last;		//������һ��ƫ��ֵ
	float Kp,Ki,Kd;		//������������֡�΢��ϵ��
	float voltage;		//�����ѹֵ������ִ�����ı�����
	float integral;		//�������ֵ
	float umax;
	float umin;
}PID_Struct;
PID_Struct pid;

//2.ͳһ��ʼ�������������ǲ���kp,ki,kdvoid PID_Init(void)
float PID_Init(void)
{
	printf("PID_Init begin\n");
	pid.setSpeed=0.0;
	pid.actualSpeed=0.0;
	pid.err=0.0;
	pid.err_last=0.0;
	pid.voltage=0.0;
	pid.integral=0.0;
	pid.Kp=0.2;		//�������pid���� 
	pid.Ki=0.1;
	pid.Kd=0.2;
	pid.umax=400;
	pid.umin=-200;
	printf("PID_Init end\n");
}

//3.��д�����㷨
//��������Ҫ���趨�ٶ���Ϊ��������������ǵ��豸��ǰӦ�ÿ���Ϊ�������ٶ�
float PID_realize(float speed)
{
	float index=0;	//ע��Ҫʵ�ֱ����pid��index�����Ͳ�����int�ͣ�����float 
	
	pid.setSpeed = speed;
	pid.err = pid.setSpeed-pid.actualSpeed;
	
	//�����pid 
	if(abs(pid.err) > 200)
	{
		index = 0.0;
	}
	else if(abs(pid.err) < 180)
	{
		index = 1.0;
		pid.integral+=pid.err;
	}
	else	//180<err<200,����ֲ��� 
	{
		index = (200-abs(pid.err))/20;
		pid.integral+=pid.err;
	}
	
	//���������err���ۼӣ�����err/2+err_last/2���ۼ� 
    pid.voltage=pid.Kp*pid.err + index*pid.Ki*pid.integral + pid.Kd*(pid.err-pid.err_last);  //���λ���

	pid.err_last = pid.err;
	pid.actualSpeed = pid.voltage*1.0;
	return pid.actualSpeed;
}

//���Դ���
int main(void)
{
	int i=1;
	int count=0;
	printf("system begin\n");
	PID_Init();
	while(count<100) //����1000�Σ���������� 
	{
		float speed = PID_realize(200.0);
		printf("%4d:\t%f\n",i++,speed);
		count++;
	}
	return 0;
}

