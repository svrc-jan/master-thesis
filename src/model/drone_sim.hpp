#ifndef __DRONE_SIM_HPP__
#define __DRONE_SIM_HPP__

#include <random>

#include "model/drone_model.hpp"

class Drone_sim {
public:
	Drone_sim(double dt=0.02, double c_mass=10, double c_hor=0.1, double c_ver=0.4, 
		double c_ang=0.05, double ang_err=0.0, double n_sta=0.01, double n_out=0.001) :
		dt(dt), ang_err(ang_err), c_mass(c_mass), c_hor(c_hor), c_ver(c_ver), c_ang(c_ang)
		{
			this->noise_state = normal_distribution<double>(0, n_sta);
			this->noise_out = normal_distribution<double>(0, n_out);
			this->rand_input = uniform_real_distribution<double>(-1, 1);
		}

	pos_t get_obs();
	pos_t diff_eq(pos_t curr, input_t input);

	pos_t reset();
	pos_t step(input_t input);

	input_t random_input();

	pos_t pos;
	input_t input_st;


private:
	double dt, ang_err, c_mass, c_hor, c_ver, c_ang;

	random_device rng;
	normal_distribution<double> noise_state, noise_out;
	uniform_real_distribution<double> rand_input;
};

#endif