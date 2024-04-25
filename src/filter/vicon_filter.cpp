#include "vicon_filter.hpp"


Vicon_filter::Vicon_filter(double h_thr, double v_thr, double a_thr)
{
	this->hold = -1;

	this->horizontal_threshold = h_thr;
	this->vertical_threshold = v_thr;
	this->angle_threshold = a_thr;
}

Vicon_filter::~Vicon_filter()
{
}

double Vicon_filter::wrap_angle(double a)
{
	a = fmod(a + M_PI, 2*M_PI) - M_PI;

	return a;
}


double Vicon_filter::unwrap_angle(double a, double a0)
{
	return a0 + wrap_angle(a - a0);
}

pos_t Vicon_filter::step(pos_t obs, bool valid)
{
	if (this->hold == -1) {
		this->curr_pos = obs;
		
		if (valid && !(obs.x == 0 && obs.y == 0 && obs.z == 0 & obs.a == 0)) {
			hold = 0;
		}

		return this->curr_pos;

	}

	pos_t diff;
	diff.data = obs.data - this->curr_pos.data;
	diff.a = wrap_angle(diff.a);
	double lim = this->hold + 3;

	if (sqrt(pow(diff.x, 2) + pow(diff.y, 2)) < this->vertical_threshold &&
		abs(diff.z) < this->horizontal_threshold*lim &&
		valid) {

		
		if (abs(diff.a) < this->angle_threshold*lim) {
			obs.a = unwrap_angle(obs.a, this->curr_pos.a);
		}
		else {
			obs.a = this->curr_pos.a;
		}

		this->curr_pos = obs;		
		this->hold = 0;
	}
	else {
		this->hold++;
	}

	return this->curr_pos;
}