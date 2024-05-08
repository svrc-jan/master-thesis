#include "vicon_filter.hpp"


Vicon_filter::Vicon_filter(double h_thr, double v_thr, double a_thr)
{
	this->hold = -1;
	this->a_hold = 0;

	this->horizontal_threshold = h_thr;
	this->vertical_threshold = v_thr;
	this->angle_threshold = a_thr;
}

Vicon_filter::~Vicon_filter()
{
}

bool Vicon_filter::all_zero(pos_t obs)
{
	double eps = 1e-3;
	return (abs(obs.x) < eps && abs(obs.y) < eps && abs(obs.z) < eps);
}

double Vicon_filter::wrap_angle(double a)
{
	a = fmod(a + 3*M_PI, 2*M_PI) - M_PI;

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
		
		if (valid && !this->all_zero(obs)) {
			hold = 0;
		}

		return this->curr_pos;

	}

	pos_t diff;
	diff.data = obs.data - this->curr_pos.data;
	diff.a = wrap_angle(diff.a);
	double lim = this->hold + 3;

	// cout << "diff: " << diff << endl;

	if (sqrt(pow(diff.x, 2) + pow(diff.y, 2)) < this->vertical_threshold &&
		abs(diff.z) < this->horizontal_threshold*lim &&
		obs.z > 0.01 &&
		valid) {

		double a_lim = this->a_hold + 3;

		if (abs(diff.a) < this->angle_threshold*a_lim) {
			obs.a = unwrap_angle(obs.a, this->curr_pos.a);
			this->a_hold = 0;
		}
		else {
			obs.a = this->curr_pos.a;
			this->a_hold += 1;
		}

		this->curr_pos = obs;		
		this->hold = 0;
	}
	else {
		this->hold++;
	}

	return this->curr_pos;
}