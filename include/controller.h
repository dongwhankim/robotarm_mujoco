#pragma once // ?
#ifndef __CONTROLLER_H //?
#define __CONTROLLER_H //?
#include <iostream>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;

typedef enum{
	start, ready, movetobox, pickup, movetotable, dropit, finish
} States;

class CController
{
public:
	CController(int JDOF, double* q, double* qdot);
	virtual ~CController(); //	

	void calculate_joint_control_torque(int type, int joint_number, double qdes, double qdotdes);
	void output_torque_mujoco(double* torque);
	void inverseKin(double x, double y, double alpha); // inverse Kinematics
	void Finite_State_Machine(double time);
	void SetPosition(double target[], double time, double duration);
	// void SetTorque(double target[], double time, double duration);

	double RANGE(double angle);
	double a = 0.35;
	double b = 0.35;
	double c = 0.15;
	bool rev;

	double pos_goal[4];
	double pos_des[4];
	double q_des[4];
	double pos_init[4];
	double q_init[4];
	double vel_des[4];
	double vel_goal[4];
	double qdot_goal[4];
	double qdot_des[4];
	double q_goal[4];
	double q_prev[4];
	double x_goal[4];

	int state;
	int box;
	int table;

public:
	VectorXd _torque;
	VectorXd _q; //joint angle vector
	VectorXd _qdot; //joint velocity vector
	double q[4],z,x_,y_, gamma_1, k1_1, k2_1;
	double q_posdes[4];

private:
	int _dofj; //number of joint
	double _kpj; //joint control P gain
	double _kdj; //joint control D gain
	int Start_Machine;
	double cos_q2, sin_q2_1, sin_q2_2, q2_1, q2_2, k1_2, k2_2, gamma_2, q1_1, q1_2, q3_1, q3_2;

	VectorXd _qdes; //desired joint angle vector
	VectorXd _qdotdes; //desired joint velocity vector

	void Initialize();
	void JointControl(int type);
	int getValue();
	int getValue2();
	States CurrentState;

};

#endif
