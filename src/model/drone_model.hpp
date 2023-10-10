#ifndef __DRONE_MODEL_HPP__
#define __DRONE_MODEL_HPP__

#include <cstddef>

struct pos_t 
{
	double arr[4];

	double &operator[] (size_t i) { return arr[i]; }
	double &x; double &y; double &z; double& a;
	pos_t() : x(arr[0]), y(arr[1]), z(arr[2]), a(arr[3]) {}

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
};


struct input_t 
{
	double arr[4];

	double &operator[] (int i) { return arr[i]; }
	double &roll; double &pitch; double &throttle; double& yaw;

	input_t() : roll(arr[0]), pitch(arr[1]), yaw(arr[2]), throttle(arr[3]) {}
};


struct state_t 
{
	double arr[4];

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




class Drone_model
{
private:
	double roll_pitch_par = 20;
	double throttle_par = 20;
	double yaw_par = 20;

public:
	Drone_model();
	~Drone_model();

	state_t state_diff_eq(state_t curr_state, input_t input);
};

#endif