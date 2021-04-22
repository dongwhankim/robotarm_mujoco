#pragma once
#ifndef __TRAJECTORY_H
#define __TRAJECTORY_H

#include <iostream>
using namespace std;

class CTrajectory
{

public:
	CTrajectory();
	virtual ~CTrajectory();

	bool _bool_init_time, _bool_init_state, Get_Value, Get_Value2;
	double _time_start, _time, _time_end, xd;
	double _init_pos, _init_vel, _goal_pos, _goal_vel;	
	void reset_initial(double time0, double init_pos, double init_vel);
	void update_time(double time);
	void update_goal(double goal_pos, double goal_vel, double goal_time);
	double position_cubicSpline();
	double velocity_cubicSpline();


private:
	void Initialize();
};

#endif