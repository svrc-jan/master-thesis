#include <vector>
#include <list>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>

#include "utils/aux.hpp"
#include "utils/json.hpp"
#include "optim/model_ident.hpp"

using namespace std;
using namespace ceres;
using json = nlohmann::json;


template<typename M>
struct State_prior_res
{
	State_prior_res(const double *s0, const double *u, const double dt, const double *C) :
		s0(s0), u(u), dt(dt), C(C) {}
	
	template <typename T>
	bool operator()(const T* const s_next,	const T* const p, T* res) const
	{
		T ds[M::s_dim];
		M::state_eq(ds, this->s0, this->u, p);

		for(int i = 0; i < M::s_dim; i++) {
			res[i] = this->C[i]*(this->s0[i] - s_next[i] + this->dt*ds[i]);
		}

		return true;
	}


	static CostFunction* Create(const double *s0, const double* u, const double dt, const double *C) {
		return (new AutoDiffCostFunction<State_prior_res, M::s_dim, M::s_dim, M::p_dim>(new State_prior_res(s0, u, dt, C)));
	}

	const double *s0;
	const double *u;
	const double dt;
	const double *C; // cost multiplier
};


template<typename M>
class MHE_estimator
{
public:
	typedef typename M::s_vec s_vec;
	typedef typename M::u_vec u_vec;
	typedef typename M::o_vec o_vec;
	typedef typename M::p_vec p_vec;

	MHE_estimator()
	{
		
	}

	~MHE_estimator()
	{
		this->delete_all();
	}

	void delete_all()
	{
		delete this->obs_loss;
		this->obs_loss = nullptr;

		// delete this->s_arr;
		// this->s_arr = nullptr;
		// // delete this->o_arr; // possible memory leak
		// this->o_arr = nullptr;
		// delete this->u_arr;
		// this->u_arr = nullptr;

		// delete this->p_est;
		// this->p_est = nullptr;

		delete this->problem;
		this->problem = nullptr;
	}

	void shift_arr(int t)
	{
		if (t <= 0)
			return;

		if (t > this->h-1)
			t = this->h-1;

		memmove((void *)this->s[0], (void *)this->s[t], M::s_dim*(this->h+1-t)*sizeof(double));
		memmove((void *)this->o[0], (void *)this->o[t], M::o_dim*(this->h-t)*sizeof(double));
		memmove((void *)this->u[0], (void *)this->u[t], M::u_dim*(this->h-t)*sizeof(double));
	}

	void zero_arr()
	{
		memset(this->s_arr, 0, M::s_dim*(this->h+1)*sizeof(double));
		memset(this->o_arr, 0, M::o_dim*this->h*sizeof(double));
		memset(this->u_arr, 0, M::u_dim*this->h*sizeof(double));

		memset(this->p_est, 0, M::p_dim*sizeof(double));
	}

	void build_problem()
	{
		this->problem = new Problem();

		this->p_est = new double[M::p_dim];

		this->s_arr = new double[M::s_dim*this->h+1]; // we are gonna use the last estimate as prior so h+1
		this->o_arr = new double[M::o_dim*this->h];
		this->u_arr = new double[M::u_dim*this->h];

		this->zero_arr();
		
		this->s.clear();
		this->o.clear();
		this->u.clear();

		for (int t = 0; t <= this->h; t++) {
			this->s.push_back(this->s_arr + t*M::s_dim);
			if (t == this-> h) break; // only s has h+1 size;
			this->o.push_back(this->o_arr + t*M::o_dim);
			this->u.push_back(this->u_arr + t*M::u_dim);
		}

		for (int t = 0; t < this->h; t++) {
			CostFunction *obs_cost_fun = Obs_res<M>::Create(this->o[t], this->C_o.data());
			problem->AddResidualBlock(obs_cost_fun, this->obs_loss, this->s[t+1]); // we have h obs but h+1 states
		}

		CostFunction *state_prior_cost_fun = State_prior_res<M>::Create(
			this->s[0], this->u[0], dt, this->C_s.data());
		problem->AddResidualBlock(state_prior_cost_fun, nullptr, this->s[1], this->p_est);


		for (int t = 1; t < this->h; t++) {
			CostFunction *state_cost_fun = State_res<M>::Create(this->u[t], this->dt, this->C_s.data());
			problem->AddResidualBlock(state_cost_fun, nullptr, this->s[t], this->s[t+1], this->p_est);
		}

		CostFunction *param_prior_cost_fun = Prior_res<M::p_dim>::Create(this->p_prior.data(), this->C_p.data());
		problem->AddResidualBlock(param_prior_cost_fun, nullptr, this->p_est);
	}

	void solve_problem()
	{
		Solve(this->solver_options, this->problem, &(this->solver_summary));
		if (this->solver_options.minimizer_progress_to_stdout) {
			cout << this->solver_summary.BriefReport() << endl;
		}
	}

	p_vec p_vector() { return array_to_vector<M::p_dim>(this->p_est); }
	s_vec s_vector() { return array_to_vector<M::s_dim>(this->s[this->h]); }

	void set_config(json config);

	double dt;
	int h; // horizon
	
	o_vec C_o;
	s_vec C_s;
	p_vec C_p;

	p_vec p_prior;

	LossFunctionWrapper* obs_loss = nullptr;

	double *s_arr = nullptr;
	vector<double *>s;

	double *o_arr = nullptr;
	vector<double *>o;
	
	double *u_arr = nullptr;
	vector<double *>u;

	double *p_est = nullptr;
	

	Problem *problem = nullptr;
	Solver::Summary solver_summary;
	Solver::Options solver_options;
};


template<typename M>
void MHE_estimator<M>::set_config(json config)
{
	this->dt = config["dt"];
	this->h = config["h"];

	this->C_s = array_to_vector(config["C_s"]);
	this->C_o = array_to_vector(config["C_o"]);
	this->C_p = array_to_vector(config["C_prior"]);

	if (!config["solver_max_iter"].is_null()) {
		this->solver_options.max_num_iterations = config["solver_max_iter"];
	}

	if (!config["solver_tol"].is_null()) {
		this->solver_options.function_tolerance = config["solver_tol"];
	}


	if (!config["solver_stdout"].is_null()) {
		this->solver_options.minimizer_progress_to_stdout = config["solver_stdout"];
	}

	if (!config["solver_threads"].is_null()) {
		this->solver_options.num_threads = config["solver_threads"];
		cerr << "MHE using " << config["solver_threads"] << " threads" << endl;
	}

	if (!config["solver_linear_solver_type"].is_null()) {
		if (string(config["solver_linear_solver_type"]).compare("qr") == 0) {
			this->solver_options.linear_solver_type = ceres::DENSE_QR;
			cerr << "MHE using dense qr" << endl;
		}
		else if (string(config["solver_linear_solver_type"]).compare("cholesky") == 0) {
			this->solver_options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
			cerr << "MHE using dense cholesky" << endl;
		}
		else if (string(config["solver_linear_solver_type"]).compare("schur") == 0) {
			this->solver_options.linear_solver_type = ceres::DENSE_SCHUR;
			cerr << "MHE using dense schur" << endl;
		}
		else if (string(config["solver_linear_solver_type"]).compare("sparse_cholesky") == 0) {
			this->solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
			cerr << "MHE using sparse cholesky" << endl;
		}
		else if (string(config["solver_linear_solver_type"]).compare("sparse_schur") == 0) {
			this->solver_options.linear_solver_type = ceres::SPARSE_SCHUR;
			cerr << "MHE using sparse schur" << endl;
		}
	}
}


template<typename M>
class MHE_handler
{
public:
	typedef typename M::s_vec s_vec;
	typedef typename M::u_vec u_vec;
	typedef typename M::o_vec o_vec;
	typedef typename M::p_vec p_vec;

	struct request
	{
		int ts; // timestep
		
		vector<o_vec> o;
		vector<u_vec> u;

		mutex mtx;
		condition_variable cv;
	};

	struct solution
	{
		int ts; // timestep

		s_vec s;
		p_vec p;

		mutex mtx;
	};
	
	MHE_handler()
	{

	}

	~MHE_handler()
	{
		this->done = true;
		this->rqst.cv.notify_one();
		this->hndl_thread.join();
	}

	void get_est(s_vec &s_, p_vec &p_) 
	{
		unique_lock<mutex> sol_lck(this->sol.mtx);
		s_ = this->sol.s;
		p_ = this->sol.p;
	}

	void post_request(const int ts, const o_vec &o_, const u_vec &u_)
	{
		unique_lock<mutex> rqst_lck(this->rqst.mtx);
		assert(ts == rqst.ts + 1);
		this->rqst.ts = ts;

		this->rqst.o.push_back(o_);
		this->rqst.u.push_back(u_);
		
		this->rqst.cv.notify_one();
	}

	void start();


	void reset()
	{

		unique_lock<mutex> rqst_lck(this->rqst.mtx);
		this->rqst.ts = -1;
		this->rqst.o.clear();
		this->rqst.u.clear();
		rqst_lck.unlock();

		unique_lock<mutex> sol_lck(this->sol.mtx);
		this->sol.ts = -1;
		this->estim.zero_arr();
		sol_lck.unlock();

		
	}

	void set_config(json config);

	int h;
	
	solution sol;
	request rqst;
	
	atomic<bool> done = false;
	thread hndl_thread;

	MHE_estimator<M> estim;
};

template<typename M>
void mhe_handler_func(MHE_handler<M> * hndl)
{
	cerr << "starting mhe handler thread" << endl;

	int ts, time_shift;
	while (!hndl->done)
	{
		unique_lock<mutex> rqst_lck(hndl->rqst.mtx);
		if (hndl->rqst.ts <= hndl->sol.ts && !hndl->done) {
			hndl->rqst.cv.wait(rqst_lck);
		}
		if (hndl->done) {
			rqst_lck.unlock();
			break;
		}
		time_shift = hndl->rqst.ts - hndl->sol.ts;
		assert(time_shift == hndl->rqst.o.size() && time_shift == hndl->rqst.u.size());
		hndl->estim.shift_arr(time_shift);

		for (int t = 0; t < time_shift; t++) {

			memcpy(hndl->estim.o[hndl->h - time_shift + t], hndl->rqst.o[t].data(), M::o_dim*sizeof(double));
			memcpy(hndl->estim.u[hndl->h - time_shift + t], hndl->rqst.u[t].data(), M::u_dim*sizeof(double));
		}

		hndl->estim.p_prior = hndl->sol.p;

		ts = hndl->rqst.ts;

		hndl->rqst.o.clear();
		hndl->rqst.u.clear();

		rqst_lck.unlock();

		// hndl->ctrl.shift_u_arr(ts - hndl->sol.ts);
		auto start = chrono::high_resolution_clock::now();
		hndl->estim.solve_problem();
		auto end = chrono::high_resolution_clock::now();
		
		
		unique_lock<mutex> sol_lck(hndl->sol.mtx);
		hndl->sol.ts = ts;
		hndl->sol.p = hndl->estim.p_vector();
		hndl->sol.s = hndl->estim.s_vector();
		sol_lck.unlock();

		auto duration = chrono::duration_cast<chrono::microseconds>(end - start);
		cerr << "mhe ts " << ts << " duration " << duration.count() << " us" << endl;
	}


	cerr << "ending mhe handler thread" << endl;
}

template<typename M>
void MHE_handler<M>::start()
{	
	this->reset();
	this->hndl_thread = thread(mhe_handler_func<M>, this);
}

template<typename M>
void MHE_handler<M>::set_config(json config)
{
	this->estim.set_config(config);
	this->h = config["h"];
}