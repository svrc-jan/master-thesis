#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>

// using AutoDiffCostFunction;
// using CostFunction;
// using Problem;
// using Solve;
// using Solver;

using namespace  ceres;

struct ObsRes
{
	ObsRes(const double *obs, const double C) :
		idx(idx), obs(obs), C(C) {}
	
	template <typename T>
	bool operator()(const T* const state, T* residual) const 
	{
		for (int i = 0; i < 4; i++) {
			residual[i] = sqrt(this->C)*(state[i] - obs[i]);
		}

		return true;
	}


	static CostFunction* Create(const double *obs, const double C) {
		return (new AutoDiffCostFunction<ObsRes, 4, 4>(new ObsRes(obs, C)));
	}


	const int idx;
	const double *obs;
	const double C; // cost multiplier
};

struct StateRes
{
	StateRes(const int h, const double *input, const double dt, const double C) :
		h(h), input(input), dt(dt), C(C) {}
	
	template <typename T>
	bool operator()(const T* c_mass, const T* const model_params, const T* const state_curr,
		const T* const state_next, T* residual) const
	{
		T input_st[4];
		T alpha = (1.0 -dt*exp(c_mass[0]));

		for (int k = h - 1; k >= 0; k--) {
			for (int i = 0; i < 4; i++) {
				input_st[i] = input[i-4*k] + alpha*input_st[i];
			}
		}

		for (int i = 0; i < 4; i++) {
			input_st[i] = dt*exp(c_mass[0])*input_st[i];
		}



		residual[0] = sqrt(this->C)*(state_curr[0] - state_next[0] + // x
			this->dt*model_params[0]*(cos(state_curr[3] + model_params[3])*input_st[1] - // c_hor*(cos(a)*pitch - sin(a)*roll)
									  sin(state_curr[3] + model_params[3])*input_st[0]));

		residual[1] = sqrt(this->C)*(state_curr[1] - state_next[1] + // y
			this->dt*model_params[0]*(sin(state_curr[3] + model_params[3])*input_st[1] + // c_hor*(sin(a)*pitch + cos(a)*roll)
									  cos(state_curr[3] + model_params[3])*input_st[0]));

		residual[2] = sqrt(this->C)*(state_curr[2] - state_next[2] + // z
			this->dt*model_params[1]*this->input[3]); // c_ver*throttle

		residual[3] = sqrt(this->C)*(state_curr[3] - state_next[3] + // a 
			this->dt*model_params[2]*this->input[2]); // c_ang*yaw


		return true;
	}


	static CostFunction* Create(int h, double* input, double dt, double C) {
		return (new AutoDiffCostFunction<StateRes, 4, 1, 4, 4, 4>(new StateRes(h, input, dt, C)));
	}


	const int h;
	const double *input;
	const double dt;
	const double C; // cost multiplier
};


struct StateMassRes
{
	StateMassRes(const double dt, const double C) :
		dt(dt), C(C) {}
	
	template <typename T>
	bool operator()(const T* const model_params, const T* const state_curr, 
		const T* const state_next, const T* const input, T* residual) const
	{
		
		residual[0] = sqrt(this->C)*(state_curr[0] - state_next[0] + // x
			this->dt*model_params[0]*(cos(state_curr[3] + model_params[3])*input[1] - // c_hor*(cos(a)*pitch - sin(a)*roll)
									  sin(state_curr[3] + model_params[3])*input[0]));

		residual[1] = sqrt(this->C)*(state_curr[1] - state_next[1] + // y
			this->dt*model_params[0]*(sin(state_curr[3] + model_params[3])*input[1] + // c_hor*(sin(a)*pitch + cos(a)*roll)
									  cos(state_curr[3] + model_params[3])*input[0]));

		residual[2] = sqrt(this->C)*(state_curr[2] - state_next[2] + // z
			this->dt*model_params[1]*input[3]); // c_ver*throttle

		residual[3] = sqrt(this->C)*(state_curr[3] - state_next[3] + // a 
			this->dt*model_params[2]*input[2]); // c_ang*yaw


		return true;
	}


	static CostFunction* Create(double dt, double C) {
		return (new AutoDiffCostFunction<StateMassRes, 4, 4, 4, 4, 4>(new StateMassRes(dt, C)));
	}

	const double dt;
	const double C; // cost multiplier
};

struct InputMassRes
{
	InputMassRes(const double *input, const double dt, const double C) :
		input(input), dt(dt), C(C) {}
	
	template <typename T>
	bool operator()(const T* const c_mass, const T* const input_prev, const T* const input_curr,
		T* residual) const {
		
		for (int i = 0; i < 4; i++) {
			residual[i] = sqrt(this->C)*(input_prev[i] - input_curr[i] +
				this->dt*exp(c_mass[0])*(this->input[i] - input_prev[i]));
		}

		return true;
	}


	static CostFunction* Create(const double *input, const double dt, const double C) {
		return (new AutoDiffCostFunction<InputMassRes, 4, 1, 4, 4>(new InputMassRes(input, dt, C)));
	}


	const double *input;
	const double dt;
	const double C; // cost multiplier
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
			residual[i] = sqrt(C[i])*(x[i] - this->x_p[i]);

		return true;
	}

	static CostFunction* Create(const double *x_p, const double *C)
	{
		return (new AutoDiffCostFunction<Prior<S>, S, S>(new Prior<S>(x_p, C)));
	}

	const double *x_p;
	const double *C; // cost multiplier
};


class Model_est {
public:
	Model_est(const double C_obs, const double C_state, const double C_input, 
		const double obs_loss_s=0, const double input_loss_s=0) :
		C_obs(C_obs), C_state(C_state), C_input(C_input)
	{
		this->problem = new Problem();

		if (obs_loss_s > 0)
			this->obs_loss = new LossFunctionWrapper(new HuberLoss(obs_loss_s), TAKE_OWNERSHIP);
		
		if (input_loss_s > 0)
			this->input_loss = new LossFunctionWrapper(new TukeyLoss(input_loss_s), TAKE_OWNERSHIP);

		
	}

	~Model_est()
	{
		delete this->problem;
	}

	void add_obs(const double *obs, double *est)
	{
		CostFunction *cost_fun = ObsRes::Create(obs, this->C_obs);		
		problem->AddResidualBlock(cost_fun, this->obs_loss, est);
	}

	void add_state_mass(double *model_par, double *curr_state, double *next_state, 
		double *input, const double dt)
	{
		CostFunction *cost_fun = StateMassRes::Create(dt, this->C_state);
		problem->AddResidualBlock(cost_fun, nullptr, model_par, curr_state, next_state, input);
	}

	void add_input_mass(const double *input_obs, double *c_mass, double *input_prev, double *input_curr, double const dt)
	{
		CostFunction *cost_fun = InputMassRes::Create(input_obs, dt, this->C_input);
		problem->AddResidualBlock(cost_fun, this->input_loss, c_mass, input_prev, input_curr);
	}

	template<int S>
	void add_prior(const double *x_p, double *x, const double *C)
	{
		CostFunction *cost_fun = Prior<S>::Create(x_p, C);
		problem->AddResidualBlock(cost_fun, nullptr, x);
	}

	void solve(Solver::Options options, Solver::Summary *summary) {
		Solve(options, this->problem, summary);
	}


	LossFunctionWrapper* obs_loss = nullptr;
	LossFunctionWrapper* input_loss = nullptr;
	const double C_obs, C_state, C_input;
	Problem* problem;
};
