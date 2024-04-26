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
struct Obs_res
{
	Obs_res(const double *obs, const double *C) :
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
		return (new AutoDiffCostFunction<Obs_res, M::o_dim, M::s_dim>(new Obs_res(obs, C)));
	}

	const double *obs;
	const double *C; // cost multipliers
};

template<typename M>
struct State_res
{
	State_res(const double *u, const double dt, const double *C) :
		u(u), dt(dt), C(C) {}
	
	template <typename T>
	bool operator()(const T* const s_curr, const T* const s_next,
		const T* const p, T* res) const
	{
		T ds[M::s_dim];
		M::state_eq(ds, s_curr, this->u, p);

		for(int i = 0; i < M::s_dim; i++) {
			res[i] = this->C[i]*((s_curr[i] - s_next[i])/this->dt + ds[i]);
		}

		return true;
	}


	static CostFunction* Create(const double* u, const double dt, const double *C) {
		return (new AutoDiffCostFunction<State_res, M::s_dim, M::s_dim, M::s_dim, M::p_dim>(new State_res(u, dt, C)));
	}

	const double *u;
	const double dt;
	const double *C; // cost multiplier
};

template<int S>
struct Diff_res
{
	Diff_res(const double *C) :
		C(C) {}
	
	template <typename T>
	bool operator()(const T* a, const T* b, T* res) const
	{
		for(int i = 0; i < S; i++) {
			res[i] = this->C[i]*(a[i] - b[i]);
		}

		return true;
	}


	static CostFunction* Create(const double *C) {
		return (new AutoDiffCostFunction<Diff_res, S, S, S>(new Diff_res(C)));
	}

	const double *C; // cost multiplier
};



template<int S>
struct Prior_res
{
	Prior_res(const double *x_p, const double *C) :
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
		return (new AutoDiffCostFunction<Prior_res<S>, S, S>(new Prior_res<S>(x_p, C)));
	}

	const double *x_p;
	const double *C; // cost multiplier
};


template<typename M>
class Model_ident {
public:

	typedef typename M::s_vec s_vec;
	typedef typename M::u_vec u_vec;
	typedef typename M::o_vec o_vec;
	typedef typename M::p_vec p_vec;

	typedef Eigen::Matrix<double, -1, M::s_dim, Eigen::RowMajor> s_mat;
	typedef Eigen::Matrix<double, -1, M::o_dim, Eigen::RowMajor> o_mat;
	typedef Eigen::Matrix<double, -1, M::u_dim, Eigen::RowMajor> u_mat;
	typedef Eigen::Matrix<double, -1, M::p_dim, Eigen::RowMajor> p_mat;

	Model_ident()
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

		this->C_shift_diff.setZero();
		this->C_shift_global.setZero();
	}

	~Model_ident()
	{
		
		this->clear_trajectories();

		delete this->problem;
	}

	void set_config(json config);

	void add_obs(int k, int t)
	{
		assert(k < this->state_est.size());
		assert(t < this->state_est[k]->rows());
	
		double *o = this->pos_data[k]->row(t).data();
		double *s = this->state_est[k]->row(t).data();

		CostFunction *cost_fun = Obs_res<M>::Create(o, this->C_o.data());		
		this->problem->AddResidualBlock(cost_fun, this->obs_loss, s);
	}

	void add_state(int k, int t, int u_delay)
	{
		assert(k < this->state_est.size());
		assert(t < this->state_est[k]->rows() - 1);

		double *s = this->state_est[k]->row(t).data();
		double *s_next = this->state_est[k]->row(t+1).data();
		double *u;
		if (t >= u_delay)
			u = this->input_data[k]->row(t).data();
		else
			u = this->u_0.data();

		double *p = this->param_est.data();
		if (this->use_param_shift)
			p = this->param_shift_est[k]->row(t).data();

		CostFunction *cost_fun = State_res<M>::Create(u, this->dt, this->C_s.data());
		this->problem->AddResidualBlock(cost_fun, this->state_loss, s, s_next, p);
	}

	void add_param_shift_global(int k, int t)
	{
		assert(k < this->param_shift_est.size());
		assert(t < this->param_shift_est[k]->rows());

		double *p = this->param_shift_est[k]->row(t).data();
		double *p_global = this->param_est.data();

		CostFunction *cost_fun = Diff_res<M::p_dim>::Create(this->C_shift_global.data());
		this->problem->AddResidualBlock(cost_fun, nullptr, p, p_global);

		for (int i = 0; i < M::p_dim; i++) {
			this->problem->SetParameterLowerBound(p, i, this->param_lb[i]);
			this->problem->SetParameterUpperBound(p, i, this->param_ub[i]);
		}


	}

	void add_param_shift_diff(int k, int t)
	{
		assert(k < this->param_shift_est.size());
		assert(t < this->param_shift_est[k]->rows() - 1);

		double *p = this->param_shift_est[k]->row(t).data();
		double *p_next = this->param_shift_est[k]->row(t+1).data();

		CostFunction *cost_fun = Diff_res<M::p_dim>::Create(this->C_shift_diff.data());
		this->problem->AddResidualBlock(cost_fun, nullptr, p, p_next);
	}

	template<int S>
	void add_prior(double *x, const double *x_p, const double *C)
	{
		CostFunction *cost_fun = Prior_res<S>::Create(x_p, C);
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

		int N = pos.rows();

		s_mat *s_est = new s_mat;
		this->state_est.push_back(s_est);
		s_est->conservativeResize(N, M::s_dim);
		s_est->setZero();

		if (this->use_param_shift) {
			p_mat *p_shift_est = new p_mat;
			this->param_shift_est.push_back(p_shift_est);
			p_shift_est->conservativeResize(N-1, M::s_dim);
			for (int t = 0; t < N-1; t++) {
				p_shift_est->row(t) = this->param_prior;
			}
		}

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

		for (int i; i < this->param_shift_est.size(); i++)
			delete this->param_shift_est[i];
	}

	void build_problem(int u_delay)
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

				this->add_obs(k, t);
				if (t == N - 1) continue; // cant res for last state
				this->add_state(k, t, u_delay);
				if (this->use_param_shift)
					this->add_param_shift_global(k, t);

				if (t == N - 2) continue;
				if (this->use_param_shift)
					this->add_param_shift_diff(k, t);
			}
		}
	}

	s_vec calculate_state_equation_corr(p_vec &par_est, double dt, int u_delay)
	{
		s_mat s_diff;
		s_mat s_eq;
		
		s_vec stat;

		int S = 0;

		for (int k = 0; k < this->state_est.size(); k++) {
			int S_k = this->state_est[k]->rows() - u_delay - 1;
			s_diff.conservativeResize(S + S_k, M::s_dim);
			s_eq.conservativeResize(S + S_k, M::s_dim);
			

			for (int t = 0; t < S_k; t++) {
				M::state_eq(s_eq.row(S+t).data(), 
					this->state_est[k]->row(t+u_delay).data(), 
					this->input_data[k]->row(t).data(),
					par_est.data());

				s_diff.row(S+t) = 
					this->state_est[k]->row(t+u_delay+1) - 
					this->state_est[k]->row(t+u_delay);

				// for (int i = 0; i < M::s_dim; i++) {
				// 	state_diff(S + t, i) = dt*ds[i] + 
				// 		this->state_est[k]->operator()(t+u_delay, i) -
				// 		this->state_est[k]->operator()(t+u_delay+1, i);
				// }
			}
			
			S += S_k;
		}

		assert(is_nan(s_diff) && is_nan(s_eq));

		normalize_cols(s_diff);
		normalize_cols(s_eq);

		assert(is_nan(s_diff) && is_nan(s_eq));

		stat = (s_diff.cwiseProduct(s_eq)).colwise().mean();

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
		this->param_est = prior;
	}


	void solve(Solver::Summary *summary) {
		Solve(this->solver_options, this->problem, summary);
	}

	void set_loss()
	{
		if (this->obs_loss_s > 0) {
			this->obs_loss = new LossFunctionWrapper(new HuberLoss(this->obs_loss_s), ceres::TAKE_OWNERSHIP);
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
	Solver::Options solver_options;

	vector<o_mat *> pos_data;
	vector<u_mat *> input_data;
	

	vector<s_mat *> state_est;
	vector<p_mat *> param_shift_est;
	p_vec C_shift_global;
	p_vec C_shift_diff;


	p_vec param_est;
	p_vec param_prior;

	p_vec param_lb;
	p_vec param_ub;

	p_vec C_prior;
	o_vec C_o;
	s_vec C_s;

	u_vec u_0; //zero u always


	double dt;
	bool use_param_shift = false;
};

template<typename M>
void Model_ident<M>::set_config(json config)
{
	this->dt = config["dt"];

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
	this->C_prior = array_to_vector(config["C_prior"]);

	if (!config["use_param_shift"].is_null()) {
		this->use_param_shift = config["use_param_shift"];
		if (this->use_param_shift) {
			this->C_o = array_to_vector(config["C_shift_global"]);
			this->C_s = array_to_vector(config["C_shift_diff"]);
		}
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

	if (!config["solver_max_iter"].is_null()) {
		this->solver_options.max_num_iterations = config["solver_max_iter"];
	}

	if (!config["solver_tol"].is_null()) {
		this->solver_options.function_tolerance = config["solver_tol"];
	}

	if (!config["solver_max_time"].is_null()) {
		this->solver_options.max_solver_time_in_seconds = config["solver_max_time"];
	}


	if (!config["solver_stdout"].is_null()) {
		this->solver_options.minimizer_progress_to_stdout = config["solver_stdout"];
	}

	if (!config["solver_threads"].is_null()) {
		this->solver_options.num_threads = config["solver_threads"];
		cerr << "MPC using " << config["solver_threads"] << " threads" << endl;
	}

	if (!config["solver_linear_solver_type"].is_null()) {
		if (string(config["solver_linear_solver_type"]).compare("qr") == 0) {
			this->solver_options.linear_solver_type = ceres::DENSE_QR;
			cerr << "using dense qr" << endl;
		}
		else if (string(config["solver_linear_solver_type"]).compare("cholesky") == 0) {
			this->solver_options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
			cerr << "using dense cholesky" << endl;
		}
		else if (string(config["solver_linear_solver_type"]).compare("schur") == 0) {
			this->solver_options.linear_solver_type = ceres::DENSE_SCHUR;
			cerr << "using dense schur" << endl;
		}
		else if (string(config["solver_linear_solver_type"]).compare("sparse_cholesky") == 0) {
			this->solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
			cerr << "using sparse cholesky" << endl;
		}
		else if (string(config["solver_linear_solver_type"]).compare("sparse_schur") == 0) {
			this->solver_options.linear_solver_type = ceres::SPARSE_SCHUR;
			cerr << "using sparse schur" << endl;
		}
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
