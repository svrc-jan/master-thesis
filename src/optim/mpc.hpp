#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>

using namespace ceres;



template<typename M>
struct Target_term {
	using Target_cost_fun = 
		DynamicAutoDiffCostFunction<Target_term, M::s_dim>;

	Target_term(const double *s0, const double *s_tar, const double *p, 
		const int h, const double dt, const double *C) :
		s0(s0), s_tar(s_tar), p(p), h(h), dt(dt), C(C) {}

	template <typename T>
	bool operator()(T const * const *u, T* res)
	{
		T s[M::s_dim];
		T ds[M::s_dim];

		for (int i = 0; i < M::s_dim; i++) {
			s[i] = this->s0[i];
		}

		for (int t = 0; t < this->h; t++) {
			ds = M::state_eq(ds, s, u[t], p);
			for (int i = 0; i < M::s_dim; i++) {
				s[i] = s[i] + this->dt*ds[i];
			}
		}

		for (int i = 0; i < M::s_dim; i++) {
			res[i] = this->C[i]*(s[i] - this->s_tar[i]);
		}
	}

	const double *s0;
	const double *s_tar;
	const double *p;
	const int h;
	const double dt;
	const double *C;
};
