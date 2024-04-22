#ifndef __MODEL_SIM_HPP__
#define __MODEL_SIM_HPP__

#include <random>
#include <list>

#include "model/drone_model.hpp"
#include "utils/aux.hpp"
#include "utils/json.hpp"

using namespace std;

using json = nlohmann::json;

template<class M>
class Model_sim {
public:
	typedef typename M::s_vec s_vec;
	typedef typename M::u_vec u_vec;
	typedef typename M::o_vec o_vec;
	typedef typename M::p_vec p_vec;

	Model_sim(double dt) : dt(dt)
	{
		this->uniform_dist = uniform_real_distribution<double>(0.0, 1.0);
		this->normal_dist = normal_distribution<double>(0.0, 1.0);

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

	int u_delay = 0;
	double o_miss_prob = 0;
	double dt;
private:
	uniform_real_distribution<double> uniform_dist;
	normal_distribution<double> normal_dist;
	random_device rng;

	list<u_vec> u_buffer;
};


template<class M>
typename Model_sim<M>::o_vec Model_sim<M>::obs()
{
	o_vec obs;
	M::output_eq(obs.data(), this->state.data());
	
	if (o_miss_prob > 0) {
		if (this->uniform_dist(this->rng) < o_miss_prob) {
			obs.setZero();
		}
	}

	for (int i = 0; i < M::o_dim; i++) {
		obs[i] += this->o_noise_sd[i]*this->normal_dist(this->rng);
	}
	
	

	return obs;
}

template<class M>
typename Model_sim<M>::o_vec Model_sim<M>::reset()
{
	this->u_buffer.clear();
	this->state.setZero();
	return this->obs();
}

template<class M>
typename Model_sim<M>::o_vec Model_sim<M>::step(u_vec input)
{
	u_vec u;
	if (this->u_delay == 0) {
		u = input;
	}
	else {
		if (this->u_buffer.size() == this->u_delay) {
			u = this->u_buffer.back();
			this->u_buffer.pop_back();
		}
		else {
			u.setZero();
		}
		this->u_buffer.push_front(input);
	}

	s_vec ds;
	M::state_eq(ds.data(), this->state.data(), u.data(), this->params.data());

	for (int i = 0; i < M::s_dim; i++) {
		ds[i] += this->s_noise_sd[i]*this->normal_dist(this->rng);
	}

	this->state += this->dt*ds;
	
	return this->obs();
}

template<class M>
typename Model_sim<M>::p_vec Model_sim<M>::random_params_disturbance()
{
	p_vec params;

	for (int i = 0; i < M::p_dim; i++) {
		params[i] = (this->uniform_dist(this->rng) - 0.5)*(this->p_ub[i] - this->p_lb[i]);
	}

	return params;
}

template<class M>
typename Model_sim<M>::p_vec Model_sim<M>::random_params()
{
	p_vec params;
	
	params = (this->p_lb + this->p_ub)/2 + this->random_params_disturbance();

	return params;
}


template<class M>
typename Model_sim<M>::u_vec Model_sim<M>::random_input()
{
	u_vec input;

	for (int i = 0; i < M::u_dim; i++) {
		input[i] = M::u_lb[i] + this->uniform_dist(this->rng)*(M::u_ub[i] - M::u_lb[i]);
	}

	return input;
}

template<class M>
void Model_sim<M>::set_config(json config)
{
	if (!config["u_delay"].is_null()) {
		this->u_delay = config["u_delay"];
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