#ifndef __VICON_FILTER_HPP__
#define __VICON_FILTER_HPP__

#include <cmath>

#include "model/drone_model.hpp"


class Vicon_filter
{
private:
	double horizontal_threshold = 0.05;
	double vertical_threshold = 0.05;
	double angle_threshold = 0.5;
	double hold_multiplier = 1;

	int hold = -1; // -1 : uninit, 0...n : n steps hold (detecting incorect read)
	int a_hold = 0;
	pos_t curr_pos;


public:
	Vicon_filter(double h_thr=0.05, double v_thr=0.05, double a_thr=0.5);
	~Vicon_filter();

	double wrap_angle(double a);
	double unwrap_angle(double a, double prev);
	pos_t step(pos_t obs, bool valid);

	bool all_zero(pos_t obs);
};




#endif