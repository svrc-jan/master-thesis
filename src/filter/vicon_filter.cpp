#include "vicon_filter.hpp"


Vicon_filter::Vicon_filter(double thr, double a_s, double v_s)
{
	this->hold = -1;

	this->threshold = thr;
	this->angle_scaler = a_s;
	this->vertical_scaler = v_s;

	cout << "thr " << thr <<  ", a_s" << a_s << ", v_s " << v_s << endl;
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
	double lim = this->threshold*(this->hold + 3);

	if (sqrt(pow(diff.x, 2) + pow(diff.y, 2)) < lim &&
		abs(diff.z) < this->vertical_scaler*lim &&
		abs(diff.a) < this->angle_scaler*lim &&
		valid) {

		obs.a = unwrap_angle(obs.a, this->curr_pos.a);
		this->curr_pos = obs;		
		this->hold = 0;
	}
	else {
		this->hold++;
	}

	return this->curr_pos;
}