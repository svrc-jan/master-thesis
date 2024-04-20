#ifndef __DRONE_MODEL_HPP__
#define __DRONE_MODEL_HPP__

#include <cstddef>
#include <iostream>

#include <eigen3/Eigen/Dense>

#include <vicon/vicon_client.h>

using namespace std;

struct pos_t 
{
	double arr[4] = {0};

	double &operator[] (size_t i) { return arr[i]; }
	double &x; double &y; double &z; double& a;
	pos_t() : x(arr[0]), y(arr[1]), z(arr[2]), a(arr[3]) {}
	pos_t(Eigen::Vector<double, 4> v) : x(arr[0]), y(arr[1]), z(arr[2]), a(arr[3])
	{
		x = v[0]; y=v[1]; z=v[2]; a=v[3];
	}
	pos_t(double x_, double y_, double z_, double a_) : 
		x(arr[0]), y(arr[1]), z(arr[2]), a(arr[3])
	{
		x = x_; y = y_; z = z_; a = a_;
	}
	
	friend pos_t operator+(pos_t a, pos_t b) {
		pos_t rv;
		for (size_t i = 0; i < 4; i++) rv[i] = a[i] + b[i];
		return rv;
	}

	friend pos_t operator*(double f, pos_t p) {
		pos_t rv;
		for (size_t i = 0; i < 4; i++) rv[i] = f*p[i];
		return rv;
	}

	friend pos_t operator*(pos_t p, double f) {
		return f*p;
	}

	friend pos_t operator-(pos_t a, pos_t b) {
		pos_t rv;
		for (size_t i = 0; i < 4; i++) rv[i] = a[i] + -1*b[i];
		return rv;
	}

	pos_t operator+=(pos_t b) {
		pos_t rv;
		for (size_t i = 0; i < 4; i++) rv[i] = this->arr[i] + b[i];
		return rv;
	}

	void copy(const pos_t &p) {
		this->arr[0] = p.arr[0];
		this->arr[1] = p.arr[1];
		this->arr[2] = p.arr[2];
		this->arr[3] = p.arr[3];
	}


	void operator=(const pos_t &p) {
		this->copy(p);
	}

	void operator=(const CViconObject object) {
		this->x = object.translation[0];
		this->y = object.translation[1];
		this->z = object.translation[2];
		this->a = object.rotation[2];
	}

	Eigen::Vector<double, 4> to_vec() {
		Eigen::Vector<double, 4> v;
		v[0] = x; v[1] = y; v[2] = z; v[3] = a;
		return v;
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
	double arr[4] = {0};

	double &operator[] (int i) { return arr[i]; }
	double &roll; double &pitch; double &throttle; double& yaw;

	input_t() : roll(arr[0]), pitch(arr[1]), yaw(arr[2]), throttle(arr[3]) {}
	input_t(Eigen::Vector<double, 4> v) : roll(arr[0]), pitch(arr[1]), yaw(arr[2]), throttle(arr[3]) {
		roll = v[0], pitch = v[1], yaw = v[2], throttle = arr[3];
	}
	input_t(double roll_, double pitch_, double yaw_, double throttle_) : 
		roll(arr[0]), pitch(arr[1]), yaw(arr[2]), throttle(arr[3])
	{
		pitch = roll_; pitch = pitch_; yaw = yaw_; throttle = throttle_;
	}

	Eigen::Vector<double, 4> to_vec() {
		Eigen::Vector<double, 4> v;
		v[0] = roll; v[1] = pitch; v[2] = yaw; v[3] = throttle;
		return v;
	}

	void copy(const input_t &i) {
		this->arr[0] = i.arr[0];
		this->arr[1] = i.arr[1];
		this->arr[2] = i.arr[2];
		this->arr[3] = i.arr[3];
	}

	friend input_t operator+(input_t a, input_t b) {
		input_t rv;
		for (size_t i = 0; i < 4; i++) rv[i] = a[i] + b[i];
		return rv;
	}

	friend input_t operator*(double f, input_t p) {
		input_t rv;
		for (size_t i = 0; i < 4; i++) rv[i] = f*p[i];
		return rv;
	}

	friend input_t operator*(input_t p, double f) {
		return f*p;
	}

	friend input_t operator-(input_t a, input_t b) {
		input_t rv;
		for (size_t i = 0; i < 4; i++) rv[i] = a[i] + -1*b[i];
		return rv;
	}

	input_t operator+=(input_t b) {
		input_t rv;
		for (size_t i = 0; i < 4; i++) rv[i] = this->arr[i] + b[i];
		return rv;
	}
	void operator=(const input_t &i) {
		this->copy(i);
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


struct state_t 
{
	double arr[8];

	double &operator[] (int i) { return arr[i]; };
	double &x; double &y; double &z; double& a;
	double &roll; double &pitch; double &throttle; double& yaw;

	state_t() : 
		x(arr[0]), y(arr[1]), z(arr[2]), a(arr[3]),
		roll(arr[4]), pitch(arr[5]), yaw(arr[6]), throttle(arr[7]) {}


	friend state_t operator+(state_t a, state_t b) {
		state_t rv;
		for (size_t i = 0; i < 8; i++) rv[i] = a[i] + b[i];
		return rv;
	}

	friend state_t operator*(double f, state_t p) {
		state_t rv;
		for (size_t i = 0; i < 8; i++) rv[i] = f*p[i];
		return rv;
	}

	friend state_t operator*(state_t p, double f) {
		return f*p;
	}

};

class Model {
/* base Model for polymorphysm
 */
public:
	template<typename Tp, typename Ts, typename Tu>
	static bool state_eq(Ts *dx, const Ts *s, const Tu *u, const Tp *Tp) { return true; }

	template<typename T>
	static bool output_eq(T *o, const T *s)  { return true; }

	static int s_size() { return 0; }
	static int u_size() { return 0; }
	static int o_size() { return 0; }
	static int p_size() { return 0; }
};

class Simple_drone_model : Model{
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
	template<typename Tp, typename Ts, typename Tu>
	static bool state_eq(Ts *ds, const Ts *s, const Tu *u, const Tp *Tp);

	template<typename T>
	static bool output_eq(T *o, const T *s);

	static int s_size() { return 4; }
	static int u_size() { return 4; }
	static int o_size() { return 4; }
	static int p_size() { return 4; }

};

class Innertia_drone_model : Model{
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
 * d_da = c_ang*yaw  - b_a*dz
 */

	template<typename Tp, typename Ts, typename Tu>
	static bool state_eq(Ts *ds, const Ts *s, const Tu *u, const Tp *Tp);

	template<typename T>
	static bool output_eq(T *o, const T *s);

	static int s_size() { return 8; }
	static int u_size() { return 4; }
	static int o_size() { return 4; }
	static int p_size() { return 7; }
};

#endif