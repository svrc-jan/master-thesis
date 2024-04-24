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

using namespace std;
using namespace ceres;
using json = nlohmann::json;


template<typename M>
struct Target_term {
	using Target_cost_fun = 
		DynamicAutoDiffCostFunction<Target_term, M::s_dim>;

	Target_term(double *s0, double *s_tar, double *p, 
		const int h, const double dt, const double *C) :
		s0(s0), s_tar(s_tar), p(p), h(h), dt(dt), C(C) {}

	template <typename T>
	bool operator()(T const * const *u, T* res)
	{
		T s[M::s_dim];
		T ds[M::s_dim];

		for (int i = 0; i < M::s_dim; i++) {
			s[i] = T(this->s0[i]);
		}

		for (int t = 0; t < this->h; t++) {
			M::state_eq(ds, s, u[t], p);
			for (int i = 0; i < M::s_dim; i++) {
				s[i] = s[i] + this->dt*ds[i];
			}
		}

		for (int i = 0; i < M::s_dim; i++) {
			res[i] = this->C[i]*(s[i] - this->s_tar[i]);
		}

		return true;
	}

	static Target_cost_fun *Create(double *s0, double *s_tar, double *p, 
		const int h, const double dt, const double *C,
		vector<double *> *u, vector<double *> *parameter_blocks)
	{
		Target_term *term = new Target_term (s0, s_tar, p, h, dt, C);
		Target_cost_fun *cost_fun = new Target_cost_fun(term);
		
		parameter_blocks->clear();

		for (int t = 0; t < h; t++) {
			parameter_blocks->push_back(u->operator[](t));
			cost_fun->AddParameterBlock(M::u_dim);
		}

		cost_fun->SetNumResiduals(M::s_dim);

		return cost_fun;
 	}

	double *s0;
	double *s_tar;
	double *p;
	const int h;
	const double dt;
	const double *C;
};


template<typename M>
struct Action_term
{
	Action_term(const double *C) : C(C) {}
	
	template <typename T>
	bool operator()(const T* const u, T* residual) const 
	{
		for (int i = 0; i < M::o_dim; i++) {
			residual[i] = this->C[i]*u[i];
		}

		return true;
	}

	static CostFunction* Create(const double *C) {
		return (new AutoDiffCostFunction<Action_term, M::u_dim, M::u_dim>(new Action_term(C)));
	}

	const double *C; // cost multipliers
};




template<typename M>
class MPC_controller
{
public:
	typedef typename M::s_vec s_vec;
	typedef typename M::u_vec u_vec;
	typedef typename M::o_vec o_vec;
	typedef typename M::p_vec p_vec;

	MPC_controller()
	{
		this->C_s.setZero();
		this->C_s_end.setZero();
		this->C_u.setZero();

		this->s0.setZero();
		this->s_tar.setZero();

		for (int i = 0; i < M::u_dim; i++) {
			this->u_lb[i] = M::u_lb[i];
			this->u_ub[i] = M::u_ub[i];
		}
	}

	~MPC_controller()
	{
		delete this->u_arr;
		delete this->problem;
	}
	void shift_u_arr(int t)
	{
		memmove((void *)this->u[0], (void *)this->u[t], M::u_dim*(this->h-t)*sizeof(double));
	}

	void zero_u_arr()
	{
		memset(this->u_arr, 0, M::u_dim*this->h*sizeof(double));
	}

	void build_problem()
	{
		delete this->problem; // nothing happens for fresh, nullptr
		this->problem = new Problem();


		this->u_arr = new double[M::u_dim*this->h];
		this->zero_u_arr();
		
		this->u.clear();
		for (int t = 0; t < this->h; t++) {
			this->u.push_back(this->u_arr + t*M::u_dim);
		}

		double *C_ptr;
		vector<double*> parameter_blocks;

		for (int t = 0; t < this->h; t++) {
			CostFunction *action_cost_fun = Action_term<M>::Create(this->C_u.data());
			problem->AddResidualBlock(action_cost_fun, nullptr, this->u[t]);

			for (int i = 0; i < M::u_dim; i++) {
				problem->SetParameterLowerBound(this->u[t], i, this->u_lb[i]);
				problem->SetParameterUpperBound(this->u[t], i, this->u_ub[i]);
			}
		}

		for (int t = 0; t < this->h; t++) {
			
			if (t < this->h - 1) {
				C_ptr = this->C_s.data();
			}
			else {
				C_ptr = this->C_s_end.data();
			}

			typename Target_term<M>::Target_cost_fun *target_cost_fun = Target_term<M>::Create(
				this->s0.data(), 
				this->s_tar.data(), 
				this->p.data(),
				t, dt, C_ptr,
				&this->u,
				&parameter_blocks);

			problem->AddResidualBlock(target_cost_fun, nullptr, parameter_blocks);				
		}


	}

	void solve_problem(s_vec &s0_, s_vec &s_tar_, p_vec &p_)
	{
		this->s0 = s0_;
		this->s_tar = s_tar_;
		this->p = p_;

		Solve(this->solver_options, this->problem, &(this->solver_summary));
		if (this->solver_options.minimizer_progress_to_stdout) {
			cout << this->solver_summary.BriefReport() << endl;
		}
	}

	u_vec u_vector(int t) 
	{
		return array_to_vector<M::u_dim>(this->u[t]);
	}

	void set_config(json config);

	double dt;
	int h; // horizon

	s_vec s0;
	s_vec s_tar;
	p_vec p;

	s_vec C_s;
	s_vec C_s_end;
	u_vec C_u;

	u_vec u_lb;
	u_vec u_ub;
	
	double *u_arr = nullptr;
	vector<double *>u;

	Problem *problem;
	Solver::Summary solver_summary;
	Solver::Options solver_options;
};


template<typename M>
void MPC_controller<M>::set_config(json config)
{
	this->dt = config["dt"];
	this->h = config["h"];

	this->C_s = array_to_vector(config["C_s"]);
	this->C_s_end = array_to_vector(config["C_s_end"]);
	this->C_u = array_to_vector(config["C_u"]);


	if (!config["u_lb"].is_null()) {
		this->u_lb = array_to_vector(config["u_lb"]);
	}

	if (!config["u_ub"].is_null()) {
		this->u_ub = array_to_vector(config["u_ub"]);
	}

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
		cerr << "MPC using " << config["solver_threads"] << " threads" << endl;
	}

	if (!config["solver_linear_solver_type"].is_null()) {
		if (string(config["solver_linear_solver_type"]).compare("qr") == 0) {
			this->solver_options.linear_solver_type = ceres::DENSE_QR;
			cerr << "MPC using dense qr" << endl;
		}
		else if (string(config["solver_linear_solver_type"]).compare("cholesky") == 0) {
			this->solver_options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
			cerr << "MPC using dense cholesky" << endl;
		}
		else if (string(config["solver_linear_solver_type"]).compare("schur") == 0) {
			this->solver_options.linear_solver_type = ceres::DENSE_SCHUR;
			cerr << "MPC using dense schur" << endl;
		}
		else if (string(config["solver_linear_solver_type"]).compare("sparse_cholesky") == 0) {
			this->solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
			cerr << "MPC using sparse cholesky" << endl;
		}
		else if (string(config["solver_linear_solver_type"]).compare("sparse_schur") == 0) {
			this->solver_options.linear_solver_type = ceres::SPARSE_SCHUR;
			cerr << "MPC using sparse schur" << endl;
		}
	}
}


template<typename M>
class MPC_handler
{
public:
	typedef typename M::s_vec s_vec;
	typedef typename M::u_vec u_vec;
	typedef typename M::o_vec o_vec;
	typedef typename M::p_vec p_vec;

	struct request
	{
		int ts; // timestep

		s_vec s0;
		s_vec s_tar;
		p_vec p;

		mutex mtx;
		condition_variable cv;
	};

	struct solution
	{
		int ts; // timestep

		double *u_arr = nullptr;
		vector<double *> u;
		
		mutex mtx;
	};
	
	MPC_handler()
	{

	}

	~MPC_handler()
	{
		this->done = true;
		this->rqst.cv.notify_one();
		this->hndl_thread.join();

		delete this->sol.u_arr;
	}

	void post_request(int ts, s_vec s0, s_vec s_tar, p_vec p)
	{
		unique_lock<mutex> rqst_lck(this->rqst.mtx);
		this->rqst.ts = ts;
		this->rqst.s0 = s0;
		this->rqst.s_tar = s_tar;
		this->rqst.p = p;
		
		this->rqst.cv.notify_one();
		rqst_lck.unlock();
	}

	void start();

	void reset()
	{

		unique_lock<mutex> rqst_lck(this->rqst.mtx);
		this->rqst.ts = -1;
		rqst_lck.unlock();

		unique_lock<mutex> sol_lck(this->sol.mtx);
		this->sol.ts = -1;
		memset(this->sol.u_arr, 0, M::u_dim*this->h*sizeof(double));
		this->ctrl.zero_u_arr();
		sol_lck.unlock();


	}

	u_vec u_vector(int ts) 
	{
		u_vec result;
		
		unique_lock<mutex> sol_lck(this->sol.mtx);
		int idx = ts - this->sol.ts;
		if (idx < 0) {
			result = array_to_vector<M::u_dim>(this->sol.u[0]);
		}
		else if (idx >= this->h) {
			result = array_to_vector<M::u_dim>(this->sol.u[this->h-1]);
		}
		else {
			result = array_to_vector<M::u_dim>(this->sol.u[idx]);
		}
		cout << "mpc lag " << idx << " ";
		sol_lck.unlock();
		return result;
	}

	void set_config(json config);

	int h;
	int u_delay = 0;
	MPC_controller<M> ctrl;
	
	solution sol;
	request rqst;
	
	atomic<bool> done = false;
	thread hndl_thread;
};

template<typename M>
void mpc_handler_func(MPC_handler<M> * hndl)
{
	cerr << "starting mpc handler thread" << endl;

	int ts;
	typename M::s_vec s0;
	typename M::s_vec s_tar;
	typename M::p_vec p;

	while (!hndl->done)
	{
		unique_lock<mutex> rqst_lck(hndl->rqst.mtx);
		if (hndl->rqst.ts <= hndl->sol.ts && !hndl->done) {
			hndl->rqst.cv.wait(rqst_lck);
		}
		ts = hndl->rqst.ts;
		s0 = hndl->rqst.s0;
		s_tar = hndl->rqst.s_tar;
		p = hndl->rqst.p;

		rqst_lck.unlock();

		if (hndl->done) break;

		// hndl->ctrl.shift_u_arr(ts - hndl->sol.ts);
		auto start = chrono::high_resolution_clock::now();
		hndl->ctrl.solve_problem(s0, s_tar, p);
		auto end = chrono::high_resolution_clock::now();
		
		unique_lock<mutex> sol_lck(hndl->sol.mtx);

		memcpy(hndl->sol.u_arr, hndl->ctrl.u_arr, M::u_dim*hndl->h*sizeof(double));
		hndl->sol.ts = ts;

		sol_lck.unlock();

		auto duration = chrono::duration_cast<chrono::microseconds>(end - start);
	}


	cerr << "ending mpc handler thread" << endl;
}

template<typename M>
void MPC_handler<M>::start()
{
	this->sol.u_arr = new double[M::u_dim*this->h];
	
	
	this->sol.u.clear();
	for (int t = 0; t < this->h; t++) {
		this->sol.u.push_back(this->sol.u_arr + t*M::u_dim);
	}

	this->reset();
	this->hndl_thread = thread(mpc_handler_func<M>, this);
}

template<typename M>
void MPC_handler<M>::set_config(json config)
{
	this->ctrl.set_config(config);
	this->h = config["h"];
}