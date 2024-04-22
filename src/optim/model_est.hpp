#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>

#include "utils/aux.hpp"
#include "model/drone_model.hpp"

// using AutoDiffCostFunction;
// using CostFunction;
// using Problem;
// using Solve;
// using Solver;

using namespace  ceres;

template<typename M>
struct ObsRes
{
	ObsRes(const double *obs, const double *C) :
		obs(obs), C(C) {}
	
	template <typename T>
	bool operator()(const T* const s, T* residual) const 
	{
		T o[M::o_dim];
		M::output_eq(o, s);

		for (int i = 0; i < M::o_dim; i++) {
			residual[i] = this->C[i]*(o[i] - this->obs[i]);
		}

		return true;
	}

	static CostFunction* Create(const double *obs, const double *C) {
		return (new AutoDiffCostFunction<ObsRes, M::o_dim, M::s_dim>(new ObsRes(obs, C)));
	}

	const double *obs;
	const double *C; // cost multipliers
};

template<typename M>
struct StateRes
{
	StateRes(const double *u, const double dt, const double *C) :
		u(u), dt(dt), C(C) {}
	
	template <typename T>
	bool operator()(const T* const s_curr, const T* const s_next,
		const T* const p, T* res) const
	{
		T ds[M::s_dim];
		M::state_eq(ds, s_curr, this->u, p);

		for(int i = 0; i < M::s_dim; i++) {
			res[i] = this->C[i]*(s_curr[i] - s_next[i] + this->dt*ds[i]);
		}

		return true;
	}


	static CostFunction* Create(const double* u, const double dt, const double *C) {
		return (new AutoDiffCostFunction<StateRes, M::s_dim, M::s_dim, M::s_dim, M::p_dim>(new StateRes(u, dt, C)));
	}

	const double *u;
	const double dt;
	const double *C; // cost multiplier
};

template<typename M>
struct ParamStateRes
{
	ParamStateRes(const double *s_curr, const double *s_next, 
		const double *u, const double dt, const double *C) :
		s_curr(s_curr), s_next(s_next), u(u), dt(dt), C(C) {}
	
	template <typename T>
	bool operator()(const T* const p, T* res) const
	{
		T ds[M::s_dim];
		M::state_eq(ds, this->s_curr, this->u, p);

		for(int i = 0; i < M::s_dim; i++) {
			res[i] = this->C[i]*(this->s_curr[i] - this->s_next[i] + this->dt*ds[i]);
		}

		return true;
	}


	static CostFunction* Create(const double *s_curr, const double *s_next, 
		const double *u, const double dt, const double *C) {
		return (new AutoDiffCostFunction<ParamStateRes, M::s_dim, M::p_dim>(new ParamStateRes(s_curr, s_next, u, dt, C)));
	}

	const double *s_curr;
	const double *s_next;
	const double *u;
	const double dt;
	const double *C; // cost multiplier
};


template<int S>
struct Prior
{
	Prior(const double *x_p, const double *C) :
		x_p(x_p), C(C) {}
	
	template <typename T>
	bool operator()(const T* const x, T* residual) const
	{
		
		for (int i = 0; i < S; i++)
			residual[i] = C[i]*(x[i] - this->x_p[i]);

		return true;
	}

	static CostFunction* Create(const double *x_p, const double *C)
	{
		return (new AutoDiffCostFunction<Prior<S>, S, S>(new Prior<S>(x_p, C)));
	}

	const double *x_p;
	const double *C; // cost multiplier
};


template<typename M>
class Model_est {
public:

	typedef typename M::s_vec s_vec;
	typedef typename M::u_vec u_vec;
	typedef typename M::o_vec o_vec;
	typedef typename M::p_vec p_vec;

	typedef Eigen::Matrix<double, -1, M::s_dim, Eigen::RowMajor> s_mat;
	typedef Eigen::Matrix<double, -1, M::o_dim, Eigen::RowMajor> o_mat;
	typedef Eigen::Matrix<double, -1, M::u_dim, Eigen::RowMajor> u_mat;

	Model_est()
	{
		this->C_o.setZero();
		this->C_s.setZero();
		this->C_prior.setZero();
		this->u_0.setZero();

		this->pos_data.clear();
		this->input_data.clear();
		this->state_est.clear();

		this->param_lb = array_to_vector<M::p_dim>(M::p_lb);
		this->param_ub = array_to_vector<M::p_dim>(M::p_ub);

		this->param_prior = (this->param_lb + this->param_ub)/2;
	}

	~Model_est()
	{
		
		this->clear_trajectories();

		delete this->problem;
	}

	void set_config(json config);

	void add_obs(double *est, const double *obs)
	{
		CostFunction *cost_fun = ObsRes<M>::Create(obs, this->C_o.data());		
		problem->AddResidualBlock(cost_fun, this->obs_loss, est);
	}

	void add_state(double *curr_state, double *next_state, double *model_par,
		const double *input, double dt)
	{
		CostFunction *cost_fun = StateRes<M>::Create(input, dt, this->C_s.data());
		problem->AddResidualBlock(cost_fun, this->state_loss, curr_state, next_state, model_par);
	}

	template<int S>
	void add_prior(double *x, const double *x_p, const double *C)
	{
		CostFunction *cost_fun = Prior<S>::Create(x_p, C);
		problem->AddResidualBlock(cost_fun, nullptr, x);
	}

	void add_trajectory(o_mat &pos, u_mat &input)
	{
		o_mat *o_data = new o_mat;
		u_mat *u_data = new u_mat;

		this->pos_data.push_back(o_data);
		this->input_data.push_back(u_data);

		o_data->operator=(pos);
		u_data->operator=(input);

		s_mat *s_est = new s_mat;
		this->state_est.push_back(s_est);

		int N = pos.rows();

		s_est->conservativeResize(N, M::s_dim);
		s_est->setZero();

		// prep values in the last state estimate
		this->prep_values(this->state_est.size()-1);
	}

	void prep_values(int k) {
		// prepare values, dependant on model, not generic
		int N = this->state_est[k]->rows();

		for (int t = 0; t < N; t++) {
			for (int i = 0; i < M::o_dim; i++) {
				this->state_est[k]->operator()(t, i) = this->pos_data[k]->operator()(t, i);

				if (t < N - 1) {
					if (i < M::s_dim - M::o_dim) {
						this->state_est[k]->operator()(t, i + M::o_dim) = 
							this->pos_data[k]->operator()(t+1, i) - 
							this->pos_data[k]->operator()(t, i);
					}
				}
			}
		}
	}

	void clear_trajectories()
	{
		for (int i; i < this->state_est.size(); i++)
			delete this->state_est[i];

		for (int i; i < this->pos_data.size(); i++)
			delete this->pos_data[i];
		
		for (int i; i < this->input_data.size(); i++)
			delete this->input_data[i];
	}

	void build_problem(double dt, int u_delay)
	{
		assert(this->state_est.size() == this->pos_data.size() && this->state_est.size() == this->pos_data.size());
		delete this->problem; // nothing happens for fresh, nullptr

		this->problem = new Problem();

		this->set_loss();
		this->set_model_par_prior(this->param_prior);
		this->set_model_par_bounds();

		// for (int k = 0; k < this->state_est.size(); k++)
		// 	this->prep_values(k);

		double *s;
		double *s_next;
		double *u;
		double *o;

		for (int k = 0; k < this->pos_data.size(); k++) {
			int N = this->pos_data[k]->rows();
			for (int t = 0; t < N; t++) {
				o = this->pos_data[k]->row(t).data();
				s = this->state_est[k]->row(t).data();

				this->add_obs(s, o);

				if (t == N - 1) continue; // cant res for last state
				if (t < u_delay)
					u = this->u_0.data();

				else
					u = this->input_data[k]->row(t-u_delay).data();

				s_next = this->state_est[k]->row(t+1).data();
				this->add_state(s, s_next, this->param_est.data(), u, dt);
			}
		}
	}

	p_vec solve_par_only(double dt, int u_delay, Solver::Options options, Solver::Summary *summary)
	{
		Problem par_only_prob;

		p_vec par_cand = this->param_est;

		double *o;
		double *u;
		double *s;
		double *s_next;

		for (int k = 0; k < this->pos_data.size(); k++) {
			int N = this->pos_data[k]->rows();
			for (int t = u_delay; t < N-1; t++) {
				o = this->pos_data[k]->row(t).data();
				u = this->input_data[k]->row(t-u_delay).data();

				s = this->state_est[k]->row(t).data();
				s_next = this->state_est[k]->row(t+1).data();

				CostFunction *cost_fun = ParamStateRes<M>::Create(s, s_next, u, dt, this->C_s.data());
				problem->AddResidualBlock(cost_fun, nullptr, par_cand.data());
			}
		}

		Solve(options, &par_only_prob, summary);

		return par_cand;
	}

	s_vec calculate_state_equation_missmatch(p_vec &par_est, double dt, int u_delay)
	{
		s_mat res;
		int S = 0;

		for (int k = 0; k < this->state_est.size(); k++) {
			int S_k = this->state_est[k]->rows() - u_delay - 1;
			res.conservativeResize(S + S_k, M::s_dim);
			
			s_vec ds;

			for (int t = 0; t < S_k; t++) {
				M::state_eq(ds.data(), 
					this->state_est[k]->row(t+u_delay).data(), 
					this->input_data[k]->row(t).data(),
					par_est.data());
				
				for (int i = 0; i < M::s_dim; i++) {
					res(S + t, i) = dt*ds[i] + 
						this->state_est[k]->operator()(t+u_delay, i) -
						this->state_est[k]->operator()(t+u_delay+1, i);
				}
			}
			
			S += S_k;
		}

		res = res.cwiseAbs2();
		s_vec stat = res.colwise().mean();

		return stat;
	}

	void set_model_par_bounds()
	{
		for (int i = 0; i < M::p_dim; i++) {
			this->problem->SetParameterLowerBound(this->param_est.data(), i, this->param_lb[i]);
			this->problem->SetParameterUpperBound(this->param_est.data(), i, this->param_ub[i]);
		}
	}

	void set_model_par_prior(p_vec &prior)
	{
		this->param_prior = prior;
		this->add_prior<M::p_dim>(this->param_est.data(), this->param_prior.data(), this->C_prior.data());
	}


	void solve(Solver::Options options, Solver::Summary *summary) {
		Solve(options, this->problem, summary);
	}

	void set_loss()
	{
		if (this->obs_loss_s > 0) {
			this->obs_loss = new LossFunctionWrapper(new TukeyLoss(this->obs_loss_s), ceres::TAKE_OWNERSHIP);
		}

		if (this->state_loss_s > 0) {
			this->state_loss = new LossFunctionWrapper(new HuberLoss(this->state_loss_s), ceres::TAKE_OWNERSHIP);
		}
	}

	LossFunctionWrapper* obs_loss = nullptr;
	LossFunctionWrapper* state_loss = nullptr;

	double state_loss_s = 0;
	double obs_loss_s = 0;
	
	Problem* problem = nullptr;

	vector<o_mat *> pos_data;
	vector<u_mat *> input_data;
	

	vector<s_mat *> state_est;
	p_vec param_est;
	p_vec param_prior;

	p_vec param_lb;
	p_vec param_ub;

	p_vec C_prior;
	o_vec C_o;
	s_vec C_s;

	u_vec u_0;


};

template<class M>
void Model_est<M>::set_config(json config)
{
	if (!config["p_lb"].is_null()) {
		this->param_lb = array_to_vector(config["p_lb"]);
	}

	if (!config["p_ub"].is_null()) {
		this->param_ub = array_to_vector(config["p_ub"]);
	}

	p_vec param_prior = (this->param_lb + this->param_ub)/2;

	if (!config["p_prior"].is_null()) {
		this->param_prior = array_to_vector(config["p_prior"]);
	}


	this->C_o = array_to_vector(config["C_o"]);
	this->C_s = array_to_vector(config["C_s"]);

	if (config["C_prior"].is_array()) {
		this->C_prior = array_to_vector(config["C_prior"]);
	}
	else {
		this->C_prior.setConstant(config["C_prior"]);
	}

	if (!config["obs_loss_s"].is_null()) {
		this->obs_loss_s = config["obs_loss_s"];
	}

	if (!config["state_loss_s"].is_null()) {
		this->state_loss_s = config["state_loss_s"];
	}
}


// struct StateMassRes
// {
// 	StateMassRes(const double dt, const double C) :
// 		dt(dt), C(C) {}
	
// 	template <typename T>
// 	bool operator()(const T* const model_params, const T* const state_curr, 
// 		const T* const state_next, const T* const input, T* residual) const
// 	{
		
// 		residual[0] = sqrt(this->C)*(state_curr[0] - state_next[0] + // x
// 			this->dt*model_params[0]*(cos(state_curr[3] + model_params[3])*input[1] - // c_hor*(cos(a)*pitch - sin(a)*roll)
// 									  sin(state_curr[3] + model_params[3])*input[0]));

// 		residual[1] = sqrt(this->C)*(state_curr[1] - state_next[1] + // y
// 			this->dt*model_params[0]*(sin(state_curr[3] + model_params[3])*input[1] + // c_hor*(sin(a)*pitch + cos(a)*roll)
// 									  cos(state_curr[3] + model_params[3])*input[0]));

// 		residual[2] = sqrt(this->C)*(state_curr[2] - state_next[2] + // z
// 			this->dt*model_params[1]*input[3]); // c_ver*throttle

// 		residual[3] = sqrt(this->C)*(state_curr[3] - state_next[3] + // a 
// 			this->dt*model_params[2]*input[2]); // c_ang*yaw


// 		return true;
// 	}


// 	static CostFunction* Create(double dt, double C) {
// 		return (new AutoDiffCostFunction<StateMassRes, 4, 4, 4, 4, 4>(new StateMassRes(dt, C)));
// 	}

// 	const double dt;
// 	const double C; // cost multiplier
// };

// struct InputMassRes
// {
// 	InputMassRes(const double *input, const double dt, const double C) :
// 		input(input), dt(dt), C(C) {}
	
// 	template <typename T>
// 	bool operator()(const T* const c_mass, const T* const input_prev, const T* const input_curr,
// 		T* residual) const {
		
// 		for (int i = 0; i < 4; i++) {
// 			residual[i] = sqrt(this->C)*(input_prev[i] - input_curr[i] +
// 				this->dt*exp(c_mass[0])*(this->input[i] - input_prev[i]));
// 		}

// 		return true;
// 	}


// 	static CostFunction* Create(const double *input, const double dt, const double C) {
// 		return (new AutoDiffCostFunction<InputMassRes, 4, 1, 4, 4>(new InputMassRes(input, dt, C)));
// 	}


// 	const double *input;
// 	const double dt;
// 	const double C; // cost multiplier
// };
