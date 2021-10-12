
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>
#include <iostream> 

#include <algorithm>
#include <iostream>
#include <limits>
#include <string>

using namespace std;
using namespace webots;

int main() {
	Motor *motors[4];//�����ͼ��̶�Ҫ��webots��������
	webots::Keyboard keyboard;
	char wheels_names[4][8] = { "motor1","motor2","motor3","motor4" };//���ڷ������������õ�����

	Robot *robot = new Robot();//ʹ��webots�Ļ���������
	keyboard.enable(1);//���м�����������Ƶ����1ms��ȡһ��

	double speed1[4];
	double speed2[4];
	double velocity = 10;

	//��ʼ��
	for (int i = 0; i < 4; i++)
	{
		motors[i] = robot->getMotor(wheels_names[i]);//�������ڷ������������õ����ֻ�ȡ����
		motors[i]->setPosition(std::numeric_limits<double>::infinity());
		motors[i]->setVelocity(0.0);

		speed1[i] = 0;
		speed2[i] = 0;
	}




	//������һ��С���񣬵��ĸ����Ӱ�����������ת��ʱ�򣬳��ӿ�������ǰ�����ң�תȦ
	//б������ǰ��+����  ������ͬʱ��
	double speed_forward[4] = { velocity ,velocity ,velocity ,velocity };
	double speed_backward[4] = { -velocity ,-velocity ,-velocity ,-velocity };
	double speed_leftward[4] = { velocity ,-velocity ,velocity ,-velocity };
	double speed_rightward[4] = { -velocity ,velocity ,-velocity ,velocity };

	double speed_leftCircle[4] = { velocity ,-velocity ,-velocity ,velocity };
	double speed_rightCircle[4] = { -velocity ,velocity ,velocity ,-velocity };


	int timeStep = (int)robot->getBasicTimeStep();//��ȡ����webots����һ֡��ʱ��
	cout << timeStep << endl;

	while (robot->step(timeStep) != -1) //��������һ֡
	{



		//��ȡ�������룬����д���Ի���ͬʱ���µİ���������֧��7����
		int keyValue1 = keyboard.getKey();
		int keyValue2 = keyboard.getKey();
		cout << keyValue1 << ":" << keyValue2 << endl;

		//���ݰ�������������ô��ת��
		if (keyValue1 == 'W')
		{
			for (int i = 0; i < 4; i++)
			{
				speed1[i] = speed_forward[i];
			}
		}
		else if (keyValue1 == 'S')
		{
			for (int i = 0; i < 4; i++)
			{
				speed1[i] = speed_backward[i];
			}
		}
		else if (keyValue1 == 'A')
		{
			for (int i = 0; i < 4; i++)
			{
				speed1[i] = speed_leftward[i];
			}
		}
		else if (keyValue1 == 'D')
		{
			for (int i = 0; i < 4; i++)
			{
				speed1[i] = speed_rightward[i];
			}
		}
		else if (keyValue1 == 'Q')
		{
			for (int i = 0; i < 4; i++)
			{
				speed1[i] = speed_leftCircle[i];
			}
		}
		else if (keyValue1 == 'E')
		{
			for (int i = 0; i < 4; i++)
			{
				speed1[i] = speed_rightCircle[i];
			}
		}
		else
		{
			for (int i = 0; i < 4; i++)
			{
				speed1[i] = 0;
			}
		}




		if (keyValue2 == 'W')
		{
			for (int i = 0; i < 4; i++)
			{
				speed2[i] = speed_forward[i];
			}
		}
		else if (keyValue2 == 'S')
		{
			for (int i = 0; i < 4; i++)
			{
				speed2[i] = speed_backward[i];
			}
		}
		else if (keyValue2 == 'A')
		{
			for (int i = 0; i < 4; i++)
			{
				speed2[i] = speed_leftward[i];
			}
		}
		else if (keyValue2 == 'D')
		{
			for (int i = 0; i < 4; i++)
			{
				speed2[i] = speed_rightward[i];
			}
		}
		else
		{
			for (int i = 0; i < 4; i++)
			{
				speed2[i] = 0;
			}
		}


		//�õ���ִ��
		for (int i = 0; i < 4; i++)
		{
			motors[i]->setVelocity(speed1[i] + speed2[i]);
		}

		//wb_motor_set_velocity(wheels[0],right_speed);
	}


	return 0;
}
