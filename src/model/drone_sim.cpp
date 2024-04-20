#include "drone_sim.hpp"

using namespace std;

pos_t Drone_sim::get_obs()
{
	pos_t obs = this->pos;
	
	for (size_t i = 0; i < 4; i++) {
		obs[i] += this->noise_out(this->rng);
	}

	return obs;
}
pos_t Drone_sim::diff_eq(pos_t curr, input_t input)
{
	pos_t diff;

	diff.x = this->c_hor*(cos(curr.a + this->ang_err)*input.pitch - sin(curr.a + this->ang_err)*input.roll);
	diff.y = this->c_hor*(sin(curr.a + this->ang_err)*input.pitch + cos(curr.a + this->ang_err)*input.roll);
	diff.z = this->c_ver*input.throttle;
	diff.a = this->c_ang*input.yaw;

	return diff;
}

pos_t Drone_sim::reset()
{
	this->pos = pos_t(0, 0, 0, 0);
	return this->get_obs();
}

pos_t Drone_sim::step(input_t input)
{
	if (this->c_mass == 0) {
		this->input_st = input;
	}
	else {
		this->input_st = this->input_st + this->dt*this->c_mass*(input - this->input_st);
	}

	pos_t dx_dt = this->diff_eq(this->pos, input_st);
	pos_t dx;
	for (size_t i = 0; i < 4; i++) {
		dx[i] = this->dt*(dx_dt[i] + this->noise_state(this->rng));
	}

	this->pos = this->pos + dx;

	return this->get_obs();
}

input_t Drone_sim::random_input()
{
	input_t x;
	
	for (int i = 0; i < 4; i++)
		x[i] = this->rand_input(this->rng);

	return x;
}