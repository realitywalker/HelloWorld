//位置型pid, 公式：pid.kp*pid.err + pid.ki*pid.integral + pid.kd*(pid.err-pid.err_last) 

//从梯形积分版本开始，控制系数的写法，从K*变为了k* 

//但是在启动、结束或大幅度增减设定时，短时间内系统输出有很大的偏差，会造成PID运算的积分积累，
//导致控制量超过执行机构可能允许的最大动作范围对应极限控制量，从而引起较大的超调，甚至是震荡，这是绝对不允许的
//积分分离的概念，是当被控量与设定值偏差较大时，取消积分作用;当被控量接近给定值时，引入积分控制，以消除静差，提高精度

//本例子的变积分pid中，没有使用抗积分饱和与梯形积分 

//变积分PID可以看成是积分分离的PID算法的更一般的形式。在普通的PID中，积分系数ki是常数，所以在整个控制过程中，积分增量是不变的。
//但是，系统对于积分项的要求是，系统偏差大时，积分作用应该减弱甚至是全无，而在偏差小时应该加强。
//积分系数取大了会产生超调，甚至积分饱和，取小了又不能短时间内消除静差。因此，根据系统的偏差大小改变积分速度是有必要的。
//变积分PID的基本思想是设法改变积分项的累加速度，使其与偏差大小相对应：偏差越大，积分越慢; 偏差越小，积分越快。
#include <stdio.h>
#include <stdlib.h>

//1.定义PID结构体 
typedef struct
{
	float setSpeed;		//定义设定值
	float actualSpeed;	//定义实际值
	float err;			//定义偏差值
	float err_last;		//定义上一个偏差值
	float Kp,Ki,Kd;		//定义比例、积分、微分系数
	float voltage;		//定义电压值（控制执行器的变量）
	float integral;		//定义积分值
	float umax;
	float umin;
}PID_Struct;
PID_Struct pid;

//2.统一初始化变量，尤其是参数kp,ki,kdvoid PID_Init(void)
float PID_Init(void)
{
	printf("PID_Init begin\n");
	pid.setSpeed=0.0;
	pid.actualSpeed=0.0;
	pid.err=0.0;
	pid.err_last=0.0;
	pid.voltage=0.0;
	pid.integral=0.0;
	pid.Kp=0.2;		//在这里调pid参数 
	pid.Ki=0.1;
	pid.Kd=0.2;
	pid.umax=400;
	pid.umin=-200;
	printf("PID_Init end\n");
}

//3.编写控制算法
//将我们想要的设定速度作为参数，计算出我们的设备当前应该控制为怎样的速度
float PID_realize(float speed)
{
	float index=0;	//注意要实现变积分pid，index的类型不再是int型，而是float 
	
	pid.setSpeed = speed;
	pid.err = pid.setSpeed-pid.actualSpeed;
	
	//变积分pid 
	if(abs(pid.err) > 200)
	{
		index = 0.0;
	}
	else if(abs(pid.err) < 180)
	{
		index = 1.0;
		pid.integral+=pid.err;
	}
	else	//180<err<200,变积分部分 
	{
		index = (200-abs(pid.err))/20;
		pid.integral+=pid.err;
	}
	
	//积分项不再是err的累加，而是err/2+err_last/2的累加 
    pid.voltage=pid.Kp*pid.err + index*pid.Ki*pid.integral + pid.Kd*(pid.err-pid.err_last);  //梯形积分

	pid.err_last = pid.err;
	pid.actualSpeed = pid.voltage*1.0;
	return pid.actualSpeed;
}

//测试代码
int main(void)
{
	int i=1;
	int count=0;
	printf("system begin\n");
	PID_Init();
	while(count<100) //调整1000次，看调整结果 
	{
		float speed = PID_realize(200.0);
		printf("%4d:\t%f\n",i++,speed);
		count++;
	}
	return 0;
}

