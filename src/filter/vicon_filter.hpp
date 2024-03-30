#ifndef __VICON_FILTER_HPP__
#define __VICON_FILTER_HPP__

#include <cmath>

#include "model/drone_model.hpp"


class Vicon_filter
{
private:
	double threshold = 5;
	double angle_scaler = 1;
	double vertical_scaler = 1;

	int hold = -1; // -1 : uninit, 0...n : n steps hold (detecting incorect read)
	pos_t curr_pos;

public:
	Vicon_filter(double thr=5, double a_s=1, double v_s=1);
	~Vicon_filter();

	double wrap_angle(double a);
	double unwrap_angle(double a, double prev);
	pos_t step(pos_t obs, bool valid);
};




#endif