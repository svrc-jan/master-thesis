#ifndef __MODEL_SIM_HPP__
#define __MODEL_SIM_HPP__

#include <random>
#include <list>

#include "model/drone_model.hpp"
#include "utils/aux.hpp"
#include "utils/json.hpp"

using namespace std;

using json = nlohmann::json;

template<typename M>
class Model_sim {
public:
	typedef typename M::s_vec s_vec;
	typedef typename M::u_vec u_vec;
	typedef typename M::o_vec o_vec;
	typedef typename M::p_vec p_vec;

	Model_sim(int seed=0)
	{
		this->uniform_dist = uniform_real_distribution<double>(0.0, 1.0);
		this->normal_dist = normal_distribution<double>(0.0, 1.0);
		this->uniform_int_dist = uniform_int_distribution<>(-1, 1);

		this->s_noise_sd.setZero();
		this->o_noise_sd.setZero();
		

		p_lb = array_to_vector<M::p_dim>(M::p_lb);
		p_ub = array_to_vector<M::p_dim>(M::p_ub);
	}
	


	o_vec obs();
	o_vec reset();
	o_vec step(u_vec input);

	p_vec random_params();
	p_vec random_params_disturbance();
	u_vec random_input();
	
	void set_config(json config);

	s_vec state;
	p_vec params;

	p_vec p_lb;
	p_vec p_ub;

	s_vec s_noise_sd;
	o_vec o_noise_sd;

	double s_noise_drift = 0;
	s_vec s_drift;

	int u_delay = 0; // expected delay
	int u_delay_curr = 0; // current delay
	int u_delay_max_diff = 0; // max_diff

	double o_miss_prob = 0;
	double dt = 1;
private:
	uniform_real_distribution<double> uniform_dist;
	normal_distribution<double> normal_dist;
	uniform_int_distribution<> uniform_int_dist;
	random_device rng;

	list<u_vec> u_buffer;
};


template<typename M>
typename Model_sim<M>::o_vec Model_sim<M>::obs()
{
	o_vec obs;
	M::output_eq(obs.data(), this->state.data());
	
	if (this->o_miss_prob > 0) {
		if (this->uniform_dist(this->rng) < this->o_miss_prob) {
			obs.setZero();
		}
	}

	for (int i = 0; i < M::o_dim; i++) {
		obs[i] += this->o_noise_sd[i]*this->normal_dist(this->rng);
	}
	
	

	return obs;
}

template<typename M>
typename Model_sim<M>::o_vec Model_sim<M>::reset()
{
	this->u_buffer.clear();
	this->state.setZero();
	this->s_drift.setZero();

	this->u_delay_curr = u_delay;

	return this->obs();
}

template<typename M>
typename Model_sim<M>::o_vec Model_sim<M>::step(u_vec input)
{
	u_vec u;
	if (this->u_delay == 0) {
		u = input;
	}
	else {
		this->u_buffer.push_front(input);
		if (this->u_buffer.size() > this->u_delay_curr) {
			auto u_it = this->u_buffer.begin();
			advance(u_it, this->u_delay_curr);
			u = *u_it;

			if (this->u_delay_max_diff > 0) {
				this->u_delay_curr += this->uniform_int_dist(this->rng);
				this->u_delay_curr = max(this->u_delay_curr, 
					max(0, this->u_delay - this->u_delay_max_diff));

				this->u_delay_curr = min(this->u_delay_curr, 
					this->u_delay + this->u_delay_max_diff);
			}

			while (this->u_buffer.size() > this->u_delay + this->u_delay_max_diff+1) {
				this->u_buffer.pop_back();
			}
		}
		else {
			u.setZero();
		}
		
	}

	s_vec ds;
	M::state_eq(ds.data(), this->state.data(), u.data(), this->params.data());

	if (this->s_noise_drift > 0) {
		for (int i = 0; i < M::s_dim; i++) {
			this->s_drift[i] -= this->dt*(1 - this->s_noise_drift)*this->s_drift[i];
			this->s_drift[i] += this->s_noise_sd[i]*this->normal_dist(this->rng);
			
			ds[i] += this->s_drift[i];
		}
	}
	else {
		for (int i = 0; i < M::s_dim; i++) {
			ds[i] += this->s_noise_sd[i]*this->normal_dist(this->rng);
		}
	}

	this->state += this->dt*ds;
	
	return this->obs();
}

template<typename M>
typename Model_sim<M>::p_vec Model_sim<M>::random_params_disturbance()
{
	p_vec params;

	for (int i = 0; i < M::p_dim; i++) {
		params[i] = (this->uniform_dist(this->rng) - 0.5)*(this->p_ub[i] - this->p_lb[i]);
	}

	return params;
}

template<typename M>
typename Model_sim<M>::p_vec Model_sim<M>::random_params()
{
	p_vec params;
	
	params = (this->p_lb + this->p_ub)/2 + this->random_params_disturbance();

	return params;
}


template<typename M>
typename Model_sim<M>::u_vec Model_sim<M>::random_input()
{
	u_vec input;

	for (int i = 0; i < M::u_dim; i++) {
		input[i] = M::u_lb[i] + this->uniform_dist(this->rng)*(M::u_ub[i] - M::u_lb[i]);
	}

	return input;
}

template<typename M>
void Model_sim<M>::set_config(json config)
{
	this->dt = config["dt"];

	if (!config["u_delay"].is_null()) {
		this->u_delay = config["u_delay"];
	}

	if (!config["u_delay_max_diff"].is_null()) {
		this->u_delay_max_diff = config["u_delay_max_diff"];
	}

	if (!config["o_miss_prob"].is_null()) {
		this->o_miss_prob = config["o_miss_prob"];
	}

	if (!config["p_lb"].is_null()) {
		this->p_lb = array_to_vector(config["p_lb"]);
	}

	if (!config["p_ub"].is_null()) {
		this->p_ub = array_to_vector(config["p_ub"]);
	}

	if (!config["s_noise_sd"].is_null()) {
		this->s_noise_sd = array_to_vector(config["s_noise_sd"]);
	}

	if (!config["o_noise_sd"].is_null()) {
		this->o_noise_sd = array_to_vector(config["o_noise_sd"]);
	}

	if (!config["s_noise_drift"].is_null()) {
		this->s_noise_drift = config["s_noise_drift"];
	}

	if (config["p"].is_string()) {
		if (string(config["p"]).compare("middle") == 0) {
			this->params = (this->p_lb + this->p_ub)/2;
		}

		else {
			this->params = this->random_params();
		}
	}
	else {
		if (config["p"].is_array()) {
			this->params = array_to_vector(config["p"]);
		}
		else {
			this->params = this->random_params();
		}
	}
}

#endif