#ifndef __DRONE_MODEL_HPP__
#define __DRONE_MODEL_HPP__

#include <cstddef>
#include <iostream>
#include <cmath>
#include <list>
#include <string>

#include <eigen3/Eigen/Dense>
#include <vicon/vicon_client.h>

using namespace std;

struct pos_t 
{
	Eigen::Vector4d data = {0, 0, 0, 0};

	double &operator[] (size_t i) { return data[i]; }
	double &x; double &y; double &z; double& a;
	pos_t() : x(data(0)), y(data(1)), z(data(2)), a(data(3)) {}
	
	pos_t(pos_t& p) : x(data(0)), y(data(1)), z(data(2)), a(data(3))
	{
		data = p.data;
	}

	pos_t(Eigen::Vector4d v) : 
		x(data(0)), y(data(1)), z(data(2)), a(data(3))
	{
		data = v;
	}

	pos_t(double *arr) : 
		x(data(0)), y(data(1)), z(data(2)), a(data(3))
	{
		pos_t();
		data = {arr[0], arr[1], arr[2], arr[3]};
	}

	pos_t(double x_, double y_, double z_, double a_) : 
		x(data(0)), y(data(1)), z(data(2)), a(data(3))
	{
		pos_t();
		data = {x_, y_, z_, a_};
	}


	void operator=(const pos_t &p) {
		data = p.data;
	}

	void operator=(const Eigen::Vector4d v) {
		data = v;
	}

	operator Eigen::Vector4d() {
		return data;
	}

	void operator=(const CViconObject object) {
		this->x = object.translation[0];
		this->y = object.translation[1];
		this->z = object.translation[2];
		this->a = object.rotation[2];
	}



	string to_str() const {
		char buff[256];
		snprintf(buff, sizeof(buff), "(x=%5.2f, y=%5.2f, z=%5.2f, a=%5.2f)",
			this->x, this->y, this->z, this->a);

		return string(buff);
	}

	friend ostream & operator<<(ostream &os, const pos_t& x) {
    	return os << x.to_str();
	}
};


struct input_t 
{
	Eigen::Vector4d data = {0, 0, 0, 0};

	double &operator[] (size_t i) { return data[i]; }
	double &roll; double &pitch; double &yaw; double& throttle;
	input_t() : roll(data(0)), pitch(data(1)), yaw(data(2)), throttle(data(3)) {}
	
	input_t(input_t& p) : roll(data(0)), pitch(data(1)), yaw(data(2)), throttle(data(3))
	{
		data = p.data;
	}

	input_t(Eigen::Vector4d v) : 
		roll(data(0)), pitch(data(1)), yaw(data(2)), throttle(data(3))
	{
		data = v;
	}

	input_t(double *arr) : 
		roll(data(0)), pitch(data(1)), yaw(data(2)), throttle(data(3))
	{
		input_t();
		data = {arr[0], arr[1], arr[2], arr[3]};
	}

	input_t(double roll_, double pitch_, double yaw_, double throttle_) : 
		roll(data(0)), pitch(data(1)), yaw(data(2)), throttle(data(3))
	{
		input_t();
		data = {roll_, pitch_, yaw_, throttle_};
	}

	friend pos_t operator+(const pos_t &a, const pos_t &b)
		{ return pos_t(a.data + b.data); }

	friend pos_t operator-(const pos_t &a, const pos_t &b)
		{ return pos_t(a.data - b.data); }

	void operator=(const input_t &p) {
		data = p.data;
	}

	void operator=(const Eigen::Vector4d v) {
		data = v;
	}

	operator Eigen::Vector4d() {
		return data;
	}
	
	string to_str() const {
		char buff[256];
		snprintf(buff, sizeof(buff), "(roll=%5.2f, pitch=%5.2f, throttle=%5.2f, yaw=%5.2f)",
			this->roll, this->pitch, this->throttle, this->yaw);

		return string(buff);
	}

	friend ostream & operator<<(ostream &os, const input_t& x) {
    	return os << x.to_str();
	}
};


class Base_model
{
/* base model for polymorphysm
 */
public:
	template<typename Tds, typename Ts, typename Tu, typename Tp>
	static bool state_eq(Tds *ds, const Ts *s, const Tu *u, const Tp *p) { return true; }

	template<typename To, typename Ts>
	static bool output_eq(To *o, const Ts *s)  { return true; }

	static const int s_dim = 0;
	static const int u_dim = 0;
	static const int o_dim = 0;
	static const int p_dim = 0;

	typedef Eigen::Vector<double, s_dim> s_vec;
	typedef Eigen::Vector<double, u_dim> u_vec;
	typedef Eigen::Vector<double, o_dim> o_vec;
	typedef Eigen::Vector<double, p_dim> p_vec;

	// parameter upper and lower bounds
	static constexpr double p_lb[] = {};
	static constexpr double p_ub[] = {};

	// action upper and lower bounds
	static constexpr double u_lb[] = {};
	static constexpr double u_ub[] = {};

	template<typename M>
	static typename M::s_vec predict_state(
		const typename M::s_vec s0, const list<typename M::u_vec> u_list, 
		const typename M::p_vec p, double dt);
};

class Simple_drone_model : Base_model
{
/* simple drone mode:
 * s = (x, y, z, a)
 * u = (pitch, roll, yaw, throttle)
 * o = (x, y, z, a)
 * p = (c_h, c_v, c_a, e_a)
 * 
 * 
 * state eq:
 * dx = c_hor*(cos(a+e_a)*pitch - sin(a + e_a)*roll)
 * dy = c_hor*(sin(a+e_a)*pitch + cos(a + e_a)*roll)
 * dz = c_ver*throttle
 * da = c_ang*yaw
 */
public:
	static const int s_dim = 4;
	static const int u_dim = 4;
	static const int o_dim = 4;
	static const int p_dim = 4;


	typedef Eigen::Vector<double, s_dim> s_vec;
	typedef Eigen::Vector<double, u_dim> u_vec;
	typedef Eigen::Vector<double, o_dim> o_vec;
	typedef Eigen::Vector<double, p_dim> p_vec;

	// parameter upper and lower bounds
	static constexpr double p_lb[] = {0.1, 0.1, 0.1, -3};
	static constexpr double p_ub[] = {10,  10,  10,  3};

	// action upper and lower bounds
	static constexpr double u_lb[] = {-1, -1, -1, -1};
	static constexpr double u_ub[] = {1,  1,  1,  1};


	template<typename Tds, typename Ts, typename Tu, typename Tp>
	static bool state_eq(Tds *ds, const Ts *s, const Tu *u, const Tp *p);

	template<typename To, typename Ts>
	static bool output_eq(To *o, const Ts *s);

	static s_vec predict_state(const s_vec s0, const list<u_vec> u_list, const p_vec p, double dt) 
		{ return Base_model::predict_state<Simple_drone_model>(s0, u_list, p, dt); };
};

class Drift_drone_model : Base_model
{
/* simple drone mode:
 * s = (x, y, z, a)
 * u = (pitch, roll, yaw, throttle)
 * o = (x, y, z, a)
 * p = (c_h, c_v, c_a, e_a)
 * 
 * 
 * state eq:
 * dx = c_hor*(cos(a+e_a)*pitch - sin(a + e_a)*roll) + dr_x
 * dy = c_hor*(sin(a+e_a)*pitch + cos(a + e_a)*roll) + dr_y
 * dz = c_ver*throttle + dr_z
 * da = c_ang*yaw + dr_a
 */
public:
	static const int s_dim = 8;
	static const int u_dim = 4;
	static const int o_dim = 4;
	static const int p_dim = 4;


	typedef Eigen::Vector<double, s_dim> s_vec;
	typedef Eigen::Vector<double, u_dim> u_vec;
	typedef Eigen::Vector<double, o_dim> o_vec;
	typedef Eigen::Vector<double, p_dim> p_vec;

	// parameter upper and lower bounds
	static constexpr double p_lb[] = {0.1, 0.1, 0.1, -3};
	static constexpr double p_ub[] = {10,  10,  10,  3};

	// action upper and lower bounds
	static constexpr double u_lb[] = {-1, -1, -1, -1};
	static constexpr double u_ub[] = {1,  1,  1,  1};


	template<typename Tds, typename Ts, typename Tu, typename Tp>
	static bool state_eq(Tds *ds, const Ts *s, const Tu *u, const Tp *p);

	template<typename To, typename Ts>
	static bool output_eq(To *o, const Ts *s);

	static s_vec predict_state(const s_vec s0, const list<u_vec> u_list, const p_vec p, double dt) 
		{ return Base_model::predict_state<Drift_drone_model>(s0, u_list, p, dt); };
};

class Innertia_drone_model : Base_model
{
/* innertia drone mode:
 * s = (x, y, z, a, dx, dy, dz, da)
 * u = (pitch, roll, yaw, throttle)
 * o = (x, y, z, a)
 * p = (c_h, c_v, c_a, e_a, b_h, b_v, b_a)
 * 
 * 
 * state eq:
 * dx = dx
 * dy = dy
 * dz = dz
 * da = da
 * d_dx = c_hor*(cos(a+e_a)*pitch - sin(a + e_a)*roll) - b_h*dx
 * d_dy = c_hor*(sin(a+e_a)*pitch + cos(a + e_a)*roll) - b_h*dy
 * d_dz = c_ver*throttle - b_v*dz
 * d_da = c_ang*yaw  - b_a*da
 */
public:
	template<typename Tds, typename Ts, typename Tu, typename Tp>
	static bool state_eq(Tds *ds, const Ts *s, const Tu *u, const Tp *p);

	template<typename To, typename Ts>
	static bool output_eq(To *o, const Ts *s);

	static const int s_dim = 8;
	static const int u_dim = 4;
	static const int o_dim = 4;
	static const int p_dim = 7;

	typedef Eigen::Vector<double, s_dim> s_vec;
	typedef Eigen::Vector<double, u_dim> u_vec;
	typedef Eigen::Vector<double, o_dim> o_vec;
	typedef Eigen::Vector<double, p_dim> p_vec;

	// parameter upper and lower bounds
	static constexpr double p_lb[] = {2,   2,   2,  -0.3, 0.2, 0.2, 0.2};
	static constexpr double p_ub[] = {100, 100, 100, 0.3, 20,  20,  20};

	// action upper and lower bounds
	static constexpr double u_lb[] = {-1, -1, -1, -1};
	static constexpr double u_ub[] = {1,  1,  1,  1};

	static s_vec predict_state(const s_vec s0, const list<u_vec> u_list, const p_vec p, double dt) 
		{ return Base_model::predict_state<Innertia_drone_model>(s0, u_list, p, dt); };
};


template<typename M>
typename M::s_vec Base_model::predict_state(
	const typename M::s_vec s0, const list<typename M::u_vec> u_list, 
	const typename M::p_vec p, double dt)
{
	typename M::s_vec ds;
	typename M::s_vec s;
	typename M::u_vec u;
	s = s0;

	for (auto u_it = u_list.begin(); u_it != u_list.end(); ++u_it) {
		u = *(u_it);
		M::state_eq(ds.data(), s.data(), u.data(), p.data());
		s = s + dt*ds;
	}

	return s;
}

/* simple drone mode:
 * s = (x, y, z, a)
 * u = (pitch, roll, yaw, throttle)
 * p = (c_h, c_v, c_a, e_a)
 * 
 * 
 * state eq: // roll inversed in real model
 * dx = c_hor*(cos(a+e_a)*pitch - sin(a + e_a)*-roll)
 * dy = c_hor*(sin(a+e_a)*pitch + cos(a + e_a)*-roll)
 * dz = c_ver*throttle
 * da = c_ang*yaw
 */
template<typename Tds, typename Ts, typename Tu, typename Tp>
bool Simple_drone_model::state_eq(Tds *ds, const Ts *s, const Tu *u, const Tp *p)
{
	ds[0] = Tds(p[0]*(cos(s[3]+p[3])*u[1] - sin(s[2]+p[3])*-u[0]));
	ds[1] = Tds(p[0]*(sin(s[3]+p[3])*u[1] + cos(s[2]+p[3])*-u[0]));
	ds[2] = Tds(p[1]*u[3]);
	ds[3] = Tds(p[2]*u[2]);

	return true;
}

/* simple drone mode:
 * s = (x, y, z, a)
 * o = (x, y, z, a)
 */
template<typename To, typename Ts>
bool Simple_drone_model::output_eq(To *o, const Ts *s)
{
	for (int i = 0; i < 4; i++)
		o[i] = s[i];

	return true;
}

/* innertia drone mode:
 * s = (x, y, z, a, dx, dy, dz, da)
 * u = (pitch, roll, yaw, throttle)
 * o = (x, y, z, a)
 * p = (c_h, c_v, c_a, e_a, b_h, b_v, b_a)
 * 
 * 
 * state eq:
 * dx = dx
 * dy = dy
 * dz = dz
 * da = da
 * d_dx = b_h*(c_hor*(cos(a+e_a)*pitch - sin(a + e_a)*roll) - dx)
 * d_dy = b_h*(c_hor*(sin(a+e_a)*pitch + cos(a + e_a)*roll) - dy)
 * d_dz = b_v*(c_ver*throttle - dz)
 * d_da = b_a*(c_ang*yaw  - da)
 */
template<typename Tds, typename Ts, typename Tu, typename Tp>
bool Innertia_drone_model::state_eq(Tds *ds, const Ts *s, const Tu *u, const Tp *p)
{
	ds[0] = Tds(s[4]); 
	ds[1] = Tds(s[5]);
	ds[2] = Tds(s[6]);
	ds[3] = Tds(s[7]);

	ds[4] = Tds(p[0]*(cos(s[2]+p[3])*u[1] - sin(s[2]+p[3])*u[0]) - p[4]*(s[4]));
	ds[5] = Tds(p[0]*(sin(s[2]+p[3])*u[1] + cos(s[2]+p[3])*u[0]) - p[4]*(s[5]));
	ds[6] = Tds(p[1]*u[3] - p[5]*(s[6]));
	ds[7] = Tds(p[2]*u[2] - p[6]*(s[7]));
	
	return true;
}
/* innertia drone mode:
 * s = (x, y, z, a, dx, dy, dz, da)
 * o = (x, y, z, a)
 */
template<typename To, typename Ts>
bool Innertia_drone_model::output_eq(To *o, const Ts *s)
{
	for (int i = 0; i < 4; i++)
		o[i] = s[i];

	return true;
}

template<typename Tds, typename Ts, typename Tu, typename Tp>
bool Drift_drone_model::state_eq(Tds *ds, const Ts *s, const Tu *u, const Tp *p)
{
	ds[0] = Tds(p[0]*(cos(s[3]+p[3])*u[1] - sin(s[2]+p[3])*-u[0])) + s[3];
	ds[1] = Tds(p[0]*(sin(s[3]+p[3])*u[1] + cos(s[2]+p[3])*-u[0])) + s[4];
	ds[2] = Tds(p[1]*u[3]) + s[5];
	ds[3] = Tds(p[2]*u[2]) + s[6];
	ds[4] = Tds(0);
	ds[5] = Tds(0);
	ds[6] = Tds(0);
	ds[7] = Tds(0);


	return true;
}

/* simple drone mode:
 * s = (x, y, z, a)
 * o = (x, y, z, a)
 */
template<typename To, typename Ts>
bool Drift_drone_model::output_eq(To *o, const Ts *s)
{
	for (int i = 0; i < 4; i++)
		o[i] = s[i];

	return true;
}

#endif