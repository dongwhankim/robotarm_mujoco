


#include "controller.h"
#pragma once
#include <math.h>
#include <stdio.h>
#include <cmath>
#include "trajectory.h"
#include "FInite_State_Machine.h"

#define PI 3.141592

CFInite_State_Machine FSM;
States CurrentState;
CTrajectory Trajectory[20]; //size = joint dofpos_goal[0] = target[0];


CController::CController(int JDOF, double* q, double* qdot)
{
	_dofj = JDOF;
	_q.setZero(_dofj);
	_qdot.setZero(_dofj);

	for (int i = 0; i < _dofj; i++)
	{
		//_q(i) = q[i];
		_qdot(i) = qdot[i];
		_q(i) = q[i];
	}
	_q(0) = q[0];
	Initialize();
}
CController::~CController()
{
}

void CController::Initialize()
{
	_torque.setZero(_dofj);
	_kpj = 30.0;
	_kdj = 0.2;//0.3;
	//_kij = 0.5;
	_qdes.setZero(_dofj);
	_qdotdes.setZero(_dofj);
}
// q
void CController::JointControl(int type)
{
	_torque.setZero();
	_torque(0) = 29.4 + _kpj * 300.0 * (_qdes(0) - _q(0)) + _kdj * 1000.0 * (_qdotdes(0) - _qdot(0));// +_Ie[0]; //40.345 38.74
	//std::cout << _qdes(0) << " " << _q(0) << " " << _qdotdes(0) << " " << _qdot(0) << std::endl;
	_torque(1) = _kpj  * (_qdes(1) - _q(1)) + _kdj * (_qdotdes(1) - _qdot(1)) ; //0.4
	_torque(2) = _kpj / 3.0 * (_qdes(2) - _q(2)) + _kdj / 3.0  * (_qdotdes(2) - _qdot(2)) ;  //1/10
	_torque(3) = _kpj / 5.0 * (_qdes(3) - _q(3)) + _kdj / 5.0 * (_qdotdes(3) - _qdot(3))  ;  //1/20
	//std::cout << _qdes(3) << " " << _q(3) << " " << _qdotdes(3) << " " << _qdot(3) ;

	/*
	for(int i = 0; i<4 ; i++)
	{ 
	_Ie[i] = _kij * (_qdes(i) - _q(i)) + _Ie_prev[i];
	if (_Ie[i] > 10)
	{
		_Ie[i] = 0;
	}
	_Ie_prev[i] = _Ie[i];
	}
	*/
}

// �Ʒ� �ּ�ó�� �ӿ��� ����ó�� ������ �߻��Ͽ� ������ ��� ���� ������ �Է�����.
void CController::calculate_joint_control_torque(int type, int joint_number, double qdes, double qdotdes)
{
	_qdes(joint_number) = qdes;
	_qdotdes(joint_number) = qdotdes;
	if (joint_number == 3)
	{
	JointControl(type);
	}
}

/*
void CController::calculate_joint_control_torque(int type, double* qdes, double* qdotdes)
{
	for (int i = 0; i < _dofj; i++)
	{
		_qdes(i) = qdes[i];
		_qdotdes(i) = qdotdes[i];
	}

	JointControl(type);
}
*/

void CController::output_torque_mujoco(double* torque)
{
	for (int i = 0; i < _dofj; i++)
	{
		torque[i] = _torque(i);
	}
}
double CController::RANGE(double angle) {
	while (angle > PI || angle <= -PI) {
		if (angle > PI) {
			angle = angle - 2 * PI;
		}
		else {
			angle = angle + 2 * PI;
		}
	}
	return angle;
}

int CController::getValue()
{
	// ���ö� ����
	int Box_Number;
	std::cout << "\n ���ö� ���� 4(��)3(��) \n             2(��)1(��) :";
	std::cin >> Box_Number;
	return Box_Number;
}

int CController::getValue2()
{
	// ���� ���̺� ����
	int Table_Number;
	std::cout << "\n ���� ���̺� ���� 1(��),2(��):";
	std::cin >> Table_Number;
	return Table_Number;

}

void CController::Finite_State_Machine(double time)
{ 
	//cout << CurrentState << endl;

	if (Start_Machine == 0) // ���۰��� start�� �ʱ�ȭ
	{
		CurrentState = start; // enum class�� ���� ���� enum�� states���� ������ ��� ������������ �� state�� ������ �Էµ��� �ʾ� Ȯ���ϱ� ������
		Start_Machine = 1; // �ٽ� �Լ��� ������� �ʵ��� ����
	}

	switch (CurrentState) // CurrentState�� ���� ������ �ٲ�
	{
		// case1: FSM(); 
	case start: // �����ڼ� �� �ڽ���ȣ �غ�
		if (time >= 2)
		{
			box = getValue(); //�ڽ��� ��ġ
			CurrentState = ready; // state�� �ٲ�
			FSM.timereset(2); // �ð� �ʱ�ȭ
			break;
		}
		rev = false;
		x_goal[0] = 0.04;
		x_goal[1] = -0.34;
		x_goal[2] = 0.175; // 0.175�� ���� ������ 165�� simulation�� ���� why??
		x_goal[3] = PI ;
		SetPosition(x_goal, time, 2); // x_goal�� ��ǥ�� 2�ʵ��� �̵�, ����ð�(time)�� �°� ��� ����
		break;

	case ready: // �ش� �ڽ� ���� �غ��ڼ�

		//table = getValue2();

		if (time >= 2)
		{
			CurrentState = movetobox;
			FSM.timereset(2);
			break;
		}
		else
		{
			if (box == 1 || box == 3)
			{
				rev = false;
				x_goal[0] = 0.0;
				x_goal[1] = 0.15;
				x_goal[2] = 0.15;
				x_goal[3] = PI;
				SetPosition(x_goal, time, 2);
				break;
			}

			else if (box == 2 || box == 4)
			{

				rev = true;
				x_goal[0] = 0.0;
				x_goal[1] = 0.15;
				x_goal[2] = -0.15;
				x_goal[3] = PI;
				SetPosition(x_goal, time, 2);
				break;
			}
		}

	case movetobox: // �ڽ� ������ �̵�

		if (time >= 2)
		{
			CurrentState = pickup;
			FSM.timereset(2);

			break;
		}
		else
		{
			if (box == 1) // 1��(���ϴ� ���ö�)
			{
				x_goal[0] = -0.66;
				x_goal[1] = 0.15;
				x_goal[2] = 0.15;
				x_goal[3] = PI;
				SetPosition(x_goal, time, 2);
				break;
			}

			else if (box == 2) // 2��(���ϴ� ���ö�)
			{
				x_goal[0] = -0.66;
				x_goal[1] = 0.15;
				x_goal[2] = -0.15;
				x_goal[3] = PI;
				SetPosition(x_goal, time, 2);
				break;
			}
			else if (box == 3) // 3��(���� ���ö�)
			{
				x_goal[0] = -0.36;
				x_goal[1] = 0.15;
				x_goal[2] = 0.15;
				x_goal[3] = PI;
				SetPosition(x_goal, time, 2);
				break;
			}
			else if (box == 4) // 4��(�»�� ���ö�)
			{
				x_goal[0] = -0.36;
				x_goal[1] = 0.15;
				x_goal[2] = -0.15;
				x_goal[3] = PI;
				SetPosition(x_goal, time, 2);
				break;
			}
		}
	case pickup: // �ڽ��� ��� �ٽ� ������
		if (time >= 6)
		{
			table = getValue2();
			CurrentState = movetotable;
			FSM.timereset(6);
			break;
		}
		if (time >= 4) // 4�ʱ��� ��ٸ��� 6�ʱ���(2�ʰ�) ������
		{
			x_goal[1] = 0.15;
			SetPosition(x_goal, time, 6);
			break;
		}
		else
		{
			x_goal[1] = -0.04;  // 1.�ְ�
			SetPosition(x_goal, time, 2);
			break;
		}


	case movetotable: // ���̺� ������ �̵�

		if (time >= 5)
		{
			CurrentState = dropit;
			FSM.timereset(5);

			break;
		}

		if (table == 1) // ���� ���̺� ���� ���
		{
			if (box == 1 || box == 3) // ���� ���ö��� ���� ���̺� ���� ���
			{
				x_goal[0] = -0.66;
				x_goal[1] = 0.15;
				x_goal[2] = 0.6;
				x_goal[3] = PI;
				SetPosition(x_goal, time, 5); // Table ������ �̵� ( �κ� ��ü�� �������� �ʰ� �̵��ϱ� ����)
				break;
			}
			else // ���� ���ö��� ���� ���̺� ���� ���
			{
					rev = false;
					x_goal[0] = -0.66;
					x_goal[1] = 0.15;
					x_goal[2] = 0.6;
					x_goal[3] = PI;
					SetPosition(x_goal, time, 5);
					break;
			}
		}
		else if (table == 2) // ���� ���̺� ���� ���
		{
			if (box == 1 || box == 3) // ���� ���ö��� ���� ���̺� ���� ���
			{
					rev = true;
					x_goal[0] = -0.87;
					x_goal[1] = 0.15;
					x_goal[2] = -0.6;
					x_goal[3] = PI;
					SetPosition(x_goal, time, 5);
					break;
			}
			else // ���� ���ö��� ���� ���̺� ���� ���
			{
				x_goal[0] = -0.87;
				x_goal[1] = 0.15;
				x_goal[2] = -0.6;
				x_goal[3] = PI;
				SetPosition(x_goal, time, 5);
				break;
			}
		}

	case dropit: // ���������� ���� 2�ʵ��� �̵� , 4�ʱ��� �������°� ���
		if (time >= 4)
		{
			CurrentState = finish;
			FSM.timereset(4);
			break;
		}

		else
		{
			x_goal[1] = 0.0;
			SetPosition(x_goal, time, 2);
			break;
		}

	case finish: // �ʱ���·� �缳��
		if (time >= 4)
		{
			CurrentState = start;
			FSM.timereset(4);
			break;

		}
		else
		{
			if (time >= 2) // �ʱ��ڼ��� �̵�
			{
				rev = false;
				x_goal[0] = 0.04;
				x_goal[1] = -0.34;
				x_goal[2] = 0.165;
				x_goal[3] = PI;
				SetPosition(x_goal, time, 4);
				/* // ����Ʈ������ �����ϴ� ���
				x_goal[0] = 0.04;
				x_goal[1] = 1.20442;
				x_goal[2] = 2.38569;
				x_goal[3] = -0.448506;
				SetTorque(x_goal, time, 4);
				*/
				break;
			}
				x_goal[0] = 0.04;
				x_goal[1] = 0.15;
				x_goal[2] = 0.0; 
				SetPosition(x_goal, time, 2); 
				break;
		}
	}
}

/*  //���ϴ� ������ �̵�
void CController::SetTorque(double target[], double time, double duration)
{
	q_des[0] = 0.04;
	if (state == 0)
	{
		for (int i = 1; i < 4; i++)
		{
			q_init[i] = q_des[i];
			Trajectory[i].reset_initial(time, q_init[i], 0);
		}
		state = 1;
	}

	if (time >= duration - 0.002)
	{
		state = 0;
	}

	for (int i = 1; i < 4; i++)
	{
		Trajectory[i].update_time(time);
		Trajectory[i].update_goal(target[i], 0, duration);
		q_des[i] = Trajectory[i].position_cubicSpline();
		qdot_des[i] = Trajectory[i].velocity_cubicSpline();
	}

	for (int i = 1; i < 4; i++)
	{
		q_prev[i] = q_des[i];
	}
	pos_des[0] = 0.04;
	pos_des[1] = -0.34;
	pos_des[2] = 0.175;
	pos_des[3] =  -PI;
	//cout << time << endl; //�ð� üũ2

	for(int i=0 ; i<4 ; i++)
	{
	calculate_joint_control_torque(0, i, q_des[i], 0);
	}
	std::cout << q_des[3] << std::endl;

}
*/

// ���ϴ� ��ǥ�� �̵�
void CController::SetPosition(double target[], double time, double duration)
{
	if (state == 0)
	{
		for (int i = 0; i < 4; i++)
		{
			pos_init[i] = pos_des[i];
			q_init[i] = q_des[i];
			Trajectory[i].reset_initial(time, pos_init[i], 0);
			Trajectory[i + 4].reset_initial(time, q_init[i], 0);
		}
		state = 1;
	}

	if (time >= duration - 0.002)
	{
		state = 0;
	}

	for (int i = 0; i < 4; i++)
	{
		Trajectory[i].update_time(time);
		Trajectory[i].update_goal(target[i], 0, duration);
		pos_des[i] = Trajectory[i].position_cubicSpline();
		vel_des[i] = Trajectory[i].velocity_cubicSpline();
	}

	std::cout << q_posdes[1] << std::endl;
	std::cout << q_posdes[2] << std::endl;
	std::cout << q_posdes[3] << std::endl;

	inverseKin(pos_des[1], pos_des[2], pos_des[3]);

	/*
	if (abs(q[3] - q_init[3]) >= PI)  // end effector�� ȸ���� �ּ����� �������� ȸ���ϰ� �ϴ� if��
		{
			if (q[3] < 0)
			{
				q[3] = q[3] + 2 * PI;
			}
			else
			{
				q[3] = q[3] - 2 * PI;
			}
		}
	*/
	for (int i = 0; i < 4; i++)
	{
		q_posdes[i] = q[i];
	}
	inverseKin(target[1], target[2], target[3]);


		for (int N = 0; N < 4; N++)
		{
			Trajectory[N + 4].update_time(time);
			q_goal[N] = q[N];

			qdot_goal[N] = 0;
			Trajectory[N + 4].update_goal(q_goal[N], qdot_goal[N], duration);
			q_des[N] = Trajectory[N + 4].position_cubicSpline();
			qdot_des[N] = Trajectory[N + 4].velocity_cubicSpline();
			q_des[0] = pos_des[0];
			q[0] = pos_des[0];

		}

		for (int i = 0; i < 4; i++)
		{
			if (CurrentState == pickup || CurrentState == dropit) // ���ϴ� ���������� position�� trajectory
			{
				calculate_joint_control_torque(0, i, q_posdes[i], qdot_des[i]);
			}
			
			else // ���ϴ� ���������� angle�� trajectory
			{
				calculate_joint_control_torque(0, i, q_des[i], qdot_des[i]);
			}
		}

		

		//cout << CurrentState << endl;
	}
	

void CController::inverseKin(double x, double y, double alpha)
{
	x_ = x - c * cos(alpha);
	y_ = y - c * sin(alpha);

	cos_q2 = (pow(x_, 2) + pow(y_, 2) - pow(a, 2) - pow(b, 2)) 
		/ (2 * a * b);

	if (abs(cos_q2) > 1) {
		std::cout << "�۵������� ����ϴ�. " << endl;
		q[0] = 0.0;// {0.0, 0.0, 0.0 };
		q[1] = 0.0;
		q[2] = 0.0;
	}
	else {

		sin_q2_1 = sqrt(1 - pow(cos_q2, 2));
		sin_q2_2 = -sqrt(1 - pow(cos_q2, 2));

		q2_1 = atan2(sin_q2_1, cos_q2);
		q2_2 = atan2(sin_q2_2, cos_q2);

		double q2[2] = { q2_1, q2_2 };
		k1_1 = b * cos(q2_1) + a;
		k1_2 = b * cos(q2_2) + a;
		double k1[2] = { k1_1,k1_2 };
		// k1
		k2_1 = b * sin(q2_1);
		k2_2 = b * sin(q2_2);
		double k2[2] = { k2_1, k2_2 };
		// k2
		gamma_1 = atan2(k2_1, k1_1);
		gamma_2 = atan2(k2_2, k1_2);
		
		double gamma[2] = { gamma_1,gamma_2 };
		z = atan2(y_, x_);
		q1_1 = z - gamma_1;
		q1_2 = z - gamma_2;
		q3_1 = alpha - q1_1 - q2_1;
		q3_2 = alpha - q1_2 - q2_2;
		double q1[2] = { q1_1,q1_2 };
		//q1
		double q3[2] = { q3_1,q3_2 };
		//q3
		double q_1[4] = { 0, q1[0],q2[0],q3[0] };
		double q_2[4] = { 0, q1[1],q2[1],q3[1] };
		for (int i = 1; i < 4; i++) {
			q_1[i] = RANGE(q_1[i]);
			q_2[i] = RANGE(q_2[i]);
			if (rev == true)
			{
				q[i] = q_2[i];
				//std::cout << q_2[3] << std::endl;
			}
			else
			{
				q[i] = q_1[i];
				//std::cout << "reverse" << std::endl;
			}
			// q[i]
			// 1 �̳� 2�� ���� ����
		}
	}

}

/*
if (time >= 3.5)
{
	rev = true;
	x_goal[0] = -0.88;
	x_goal[1] = 0.0;
	x_goal[2] = -0.6;
	x_goal[3] = PI;
	SetPosition(x_goal, time, 4);
	break;
}
else
{
	*/

	/*
	if (q[N] - q_init[N] >= PI)
	{
		q[N] = q[N] - 2 * PI;
	}
	*/
