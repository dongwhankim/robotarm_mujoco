


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

// 아래 주석처리 속에서 예외처리 에러가 발생하여 포인터 대신 직접 값으로 입력해줌.
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
	// 도시락 고르기
	int Box_Number;
	std::cout << "\n 도시락 선택 4(↖)3(↗) \n             2(↙)1(↘) :";
	std::cin >> Box_Number;
	return Box_Number;
}

int CController::getValue2()
{
	// 놓을 테이블 고르기
	int Table_Number;
	std::cout << "\n 놓을 테이블 선택 1(→),2(←):";
	std::cin >> Table_Number;
	return Table_Number;

}

void CController::Finite_State_Machine(double time)
{ 
	//cout << CurrentState << endl;

	if (Start_Machine == 0) // 시작값을 start로 초기화
	{
		CurrentState = start; // enum class를 사용시 기존 enum의 states들을 여러곳 사용 가능해지지만 각 state에 정수가 입력되지 않아 확인하기 불편함
		Start_Machine = 1; // 다시 함수가 실행되지 않도록 설정
	}

	switch (CurrentState) // CurrentState에 따라서 행위가 바뀜
	{
		// case1: FSM(); 
	case start: // 시작자세 및 박스번호 준비
		if (time >= 2)
		{
			box = getValue(); //박스의 위치
			CurrentState = ready; // state를 바꿈
			FSM.timereset(2); // 시간 초기화
			break;
		}
		rev = false;
		x_goal[0] = 0.04;
		x_goal[1] = -0.34;
		x_goal[2] = 0.175; // 0.175가 계산상 맞지만 165가 simulation상 맞음 why??
		x_goal[3] = PI ;
		SetPosition(x_goal, time, 2); // x_goal의 좌표로 2초동안 이동, 현재시간(time)에 맞게 경로 생성
		break;

	case ready: // 해당 박스 위로 준비자세

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

	case movetobox: // 박스 앞으로 이동

		if (time >= 2)
		{
			CurrentState = pickup;
			FSM.timereset(2);

			break;
		}
		else
		{
			if (box == 1) // 1번(우하단 도시락)
			{
				x_goal[0] = -0.66;
				x_goal[1] = 0.15;
				x_goal[2] = 0.15;
				x_goal[3] = PI;
				SetPosition(x_goal, time, 2);
				break;
			}

			else if (box == 2) // 2번(좌하단 도시락)
			{
				x_goal[0] = -0.66;
				x_goal[1] = 0.15;
				x_goal[2] = -0.15;
				x_goal[3] = PI;
				SetPosition(x_goal, time, 2);
				break;
			}
			else if (box == 3) // 3번(우상단 도시락)
			{
				x_goal[0] = -0.36;
				x_goal[1] = 0.15;
				x_goal[2] = 0.15;
				x_goal[3] = PI;
				SetPosition(x_goal, time, 2);
				break;
			}
			else if (box == 4) // 4번(좌상단 도시락)
			{
				x_goal[0] = -0.36;
				x_goal[1] = 0.15;
				x_goal[2] = -0.15;
				x_goal[3] = PI;
				SetPosition(x_goal, time, 2);
				break;
			}
		}
	case pickup: // 박스를 들고 다시 나오기
		if (time >= 6)
		{
			table = getValue2();
			CurrentState = movetotable;
			FSM.timereset(6);
			break;
		}
		if (time >= 4) // 4초까지 기다린후 6초까지(2초간) 나오기
		{
			x_goal[1] = 0.15;
			SetPosition(x_goal, time, 6);
			break;
		}
		else
		{
			x_goal[1] = -0.04;  // 1.넣고
			SetPosition(x_goal, time, 2);
			break;
		}


	case movetotable: // 테이블 앞으로 이동

		if (time >= 5)
		{
			CurrentState = dropit;
			FSM.timereset(5);

			break;
		}

		if (table == 1) // 우측 테이블에 놓는 경우
		{
			if (box == 1 || box == 3) // 우측 도시락을 우측 테이블에 놓는 경우
			{
				x_goal[0] = -0.66;
				x_goal[1] = 0.15;
				x_goal[2] = 0.6;
				x_goal[3] = PI;
				SetPosition(x_goal, time, 5); // Table 앞으로 이동 ( 로봇 본체와 겹쳐지지 않게 이동하기 위해)
				break;
			}
			else // 좌측 도시락을 우측 테이블에 놓는 경우
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
		else if (table == 2) // 좌측 테이블에 놓는 경우
		{
			if (box == 1 || box == 3) // 우측 도시락을 좌측 테이블에 놓는 경우
			{
					rev = true;
					x_goal[0] = -0.87;
					x_goal[1] = 0.15;
					x_goal[2] = -0.6;
					x_goal[3] = PI;
					SetPosition(x_goal, time, 5);
					break;
			}
			else // 좌측 도시락을 좌측 테이블에 놓는 경우
			{
				x_goal[0] = -0.87;
				x_goal[1] = 0.15;
				x_goal[2] = -0.6;
				x_goal[3] = PI;
				SetPosition(x_goal, time, 5);
				break;
			}
		}

	case dropit: // 내려놓으러 가기 2초동안 이동 , 4초까지 내려놓는거 대기
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

	case finish: // 초기상태로 재설정
		if (time >= 4)
		{
			CurrentState = start;
			FSM.timereset(4);
			break;

		}
		else
		{
			if (time >= 2) // 초기자세로 이동
			{
				rev = false;
				x_goal[0] = 0.04;
				x_goal[1] = -0.34;
				x_goal[2] = 0.165;
				x_goal[3] = PI;
				SetPosition(x_goal, time, 4);
				/* // 조인트값으로 제어하는 방법
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

/*  //원하는 각도로 이동
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
	//cout << time << endl; //시간 체크2

	for(int i=0 ; i<4 ; i++)
	{
	calculate_joint_control_torque(0, i, q_des[i], 0);
	}
	std::cout << q_des[3] << std::endl;

}
*/

// 원하는 좌표로 이동
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
	if (abs(q[3] - q_init[3]) >= PI)  // end effector가 회전이 최소한인 방향으로 회전하게 하는 if문
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
			if (CurrentState == pickup || CurrentState == dropit) // 원하는 지점까지의 position의 trajectory
			{
				calculate_joint_control_torque(0, i, q_posdes[i], qdot_des[i]);
			}
			
			else // 원하는 지점까지의 angle의 trajectory
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
		std::cout << "작동범위를 벗어납니다. " << endl;
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
			// 1 이냐 2냐 고를수 있음
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
