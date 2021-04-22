#include "trajectory.h"
CTrajectory::CTrajectory()
{
	Initialize();
}

CTrajectory::~CTrajectory()
{
}

void CTrajectory::Initialize()
{
	_time_start = 0.0;
	_time = 0.0;
	_time_end = 0.0;
	_init_pos = 0;
	_init_vel = 0;
	_goal_pos = 0;
	_goal_vel = 0;

    
}


void CTrajectory::reset_initial(double time0, double init_pos, double init_vel) // input ���۽ð�, ������ġ, ���ۼӵ�
{//std::cout << init_pos << " :1" << std::endl;
		_time_start = time0;
		//std::cout << init_pos << ": 2 " << std::endl;
		_init_pos = init_pos;
		_init_vel = init_vel;
	//std::cout << init_pos << std::endl;
	//printf("%lf,%lf,%lf", _time_start, _init_pos, _init_vel);
		//cout << _time_start << endl;
}


void CTrajectory::update_time(double time)
{
	_time = time;
}

void CTrajectory::update_goal(double goal_pos, double goal_vel, double goal_time )
{
	_goal_pos = goal_pos;
	_goal_vel = goal_vel;
	_time_end = goal_time;
}

double CTrajectory::position_cubicSpline()  
{
  
	if (_time < _time_start)
	{
		xd = _init_pos;
		//std::cout << "here?1" << std::endl;
	}
	else if (_time > _time_end)
	{
		xd = _goal_pos;
	    //std::cout << "here?2" << std::endl;
	}
	else {


		xd =
			_init_pos +  // ���
			_init_vel * (_time - _time_start) //1��
			+ (3.0 * (_goal_pos - _init_pos) / ((_time_end - _time_start) * (_time_end - _time_start))  
			- 2.0 * _init_vel / (_time_end - _time_start) 
			- _goal_vel / (_time_end - _time_start)) * (_time - _time_start) * (_time - _time_start) //2��
			+ (-2.0 * (_goal_pos - _init_pos) / ((_time_end - _time_start) * (_time_end - _time_start) * (_time_end - _time_start)) 
				+ (_init_vel + _goal_vel) / ((_time_end - _time_start) * (_time_end - _time_start))) * (_time - _time_start) * (_time - _time_start) * (_time - _time_start); //3��
		//std::cout << _time_start << " " << _time_end << " " << _time << " ";
		//std::cout <<  _init_vel  << " " << _goal_vel << std::endl;
		//std::cout << _goal_pos << std::endl;
	}
	return xd;
}

double CTrajectory::velocity_cubicSpline()
{
	double xdotd;

	if (_time < _time_start)
	{
		xdotd = _init_vel;
	}
	else if (_time > _time_end)
	{
		xdotd = _goal_vel;
	}
	else {
		xdotd = _init_vel + 2.0 * (3.0 * (_goal_pos - _init_pos) / ((_time_end - _time_start) * (_time_end - _time_start)) - 2.0 * _init_vel / (_time_end - _time_start) - _goal_vel / (_time_end - _time_start)) * (_time - _time_start)
			+ 3.0 * (-2.0 * (_goal_pos - _init_pos) / ((_time_end - _time_start) * (_time_end - _time_start) * (_time_end - _time_start)) + (_init_vel + _goal_vel) / ((_time_end - _time_start) * (_time_end - _time_start))) * (_time - _time_start) * (_time - _time_start);
	}
	return xdotd;
}

