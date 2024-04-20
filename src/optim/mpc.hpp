#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>

using namespace ceres;




struct Target_term {
	using Target_cost_fun = 
		DynamicAutoDiffCostFunction<Target_term, 4>;

	Target_term(const int t, const double *x0, const double *u0, c_mass ) :
		t(t), x0(x0), u0(u0) {}

	template <typename T>
	bool operator()(T const * const *u_in, T* res)
	{
		T x[4] = {0, 0, 0, 0};
		T u[4] = {0, 0, 0, 0};

		for (int i = 0; i < 4; i++) {
			x[i] += x0[i];
			x[i] += u0[i];
		}


	}

	const int t;
	const double *x0;
	const double *u0;
}