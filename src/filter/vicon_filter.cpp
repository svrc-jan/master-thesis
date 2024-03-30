#include "vicon_filter.hpp"


Vicon_filter::Vicon_filter(double thr, double a_s, double v_s)
{
	this->threshold = thr;
	this->angle_scaler = angle_scaler;
	this->vertical_scaler = v_s;
}

Vicon_filter::~Vicon_filter()
{
}

double Vicon_filter::wrap_angle(double a)
{
	a = fmod(a + M_PI, 2*M_PI);
	if (a < 0) {
		a += 2*M_PI;
	}

	return a - M_PI;
}


double Vicon_filter::unwrap_angle(double a, double prev)
{
	return prev + this->wrap_angle(a - prev);
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

	obs.a = this->unwrap_angle(obs.a, this->curr_pos.a);

	pos_t diff = obs - this->curr_pos;
	double lim = this->threshold*(this->hold + 3);

	if (sqrt(pow(diff.x, 2) + pow(diff.y, 2)) < lim &&
		abs(diff.z) < this->vertical_scaler*lim &&
		valid) {

		this->curr_pos = obs;
		this->hold = 0;
	}
	else {
		this->hold++;
	}

	return this->curr_pos;
}