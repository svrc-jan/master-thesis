#include <chrono>
#include <iostream>
#include <thread>
#include <cstdlib>
#include <thread>
#include <algorithm>

#include <termios.h>

#include "model/drone_model.hpp"
#include "optim/mhe.hpp"
#include "optim/mpc.hpp"
#include "tello/tello.h"
#include "vicon/vicon_handler.hpp"
#include "filter/vicon_filter.hpp"
#include "utils/keyboard_handler.hpp"
#include "utils/logger.hpp"
#include "utils/aux.hpp"


using namespace std;

string io_config_file = "/home/jsv/CVUT/master-thesis/config/io_real.json";
string target_file = "/home/jsv/CVUT/master-thesis/config/tar_square.json";
string log_name = "default";


Eigen::Vector<double, 4> get_traj_correction(Eigen::Vector<double, 4> pos, 
	Eigen::Vector<double, 4> target, Eigen::Vector<double, 4> prev_target, double coef)
{
	Eigen::Vector<double, 4> result;

	Eigen::Vector3d p;
	Eigen::Vector3d a;
	Eigen::Vector3d b;
	Eigen::Vector3d r;

	for (int i = 0; i < 3; i++) {
		p[i] = pos[i];
		a[i] = target[i];
		b[i] = prev_target[i];
	}

	auto ap = p - a;
	auto ab = (b - a).normalized();

	r = a + coef*ap.norm()*(a + ap.dot(ab)*ab - p);

	for (int i = 0; i < 3; i++) {
		result[i] = r[i];
	}
	result[3] = target[3];

	return result;
}

string get_new_log_name(string folder, string name)
{
	int index = 0;
	char buffer[256];
	do {
		index += 1;
		sprintf(buffer, "%s/%s_%d.log", folder.c_str(), name.c_str(), index);
	} while (file_exists(buffer));

	return string(buffer);
}

template<int S>
vector<Eigen::Vector<double, S>> get_targets(json target_json)
{
	Eigen::Vector<double, S> ref;
	Eigen::Vector<double, S> t;


	vector<Eigen::Vector<double, S>> targets;
	
	if (!target_json["ref"].is_array()) {
		cerr << "Target file invalid ref!" << endl;
		exit(EXIT_FAILURE);
	}

	if (!target_json["targets"].is_array()) {
		cerr << "Target file invalid targets!" << endl;
		exit(EXIT_FAILURE);
	}

	if (!target_json["targets"][0].is_array()) {
		cerr << "Target file invalid target format!" << endl;
		exit(EXIT_FAILURE);
	}

	
	ref = array_to_vector(target_json["ref"]);
	for (int i = 0; i < target_json["targets"].size(); i++) {
		t = ref + array_to_vector(target_json["targets"][i]);
		targets.push_back(t);
		cout << "target " << i << " : " << t.transpose() << endl;
	}

	return targets;
}

int main(int argc, char const *argv[])
{
	
	typedef Simple_drone_model M;
	using namespace std::chrono;

	json io_config = get_json_config(io_config_file);

	char buffer[256];

	if (argc > 1) {
		log_name = string(argv[1]);
	}
	if (argc > 2) {
		target_file = string(argv[2]);
	}
	

	json target_json = get_json_config(target_file);
	auto targets = get_targets<M::s_dim>(target_json);
	double target_tol = target_json["tol"];

	CTello tello1;
	const string tello_net_interface(io_config["tello_net_interface"]);
    tello1.init(
		io_config["tello_port"], 
		tello_net_interface.c_str());    // "wlxd0374576c511" or "wlxd0374543f3a6";

	Keyboard_handler keyboard_hndl(io_config["keyboard_device"]);
	Vicon_handler vicon_hndl(io_config["vicon_ip"]);

	Vicon_filter vicon_filter(
		io_config["filter_horizontal_threshold"],
		io_config["filter_vertical_threshold"],
		io_config["filter_angle_threshold"]);

	MHE_handler<M> mhe;
	json mhe_config = get_json_config(io_config["mhe_config"]); 
	mhe.set_config(mhe_config);
	mhe.estim.build_problem();
	mhe.start();

	MPC_handler<M> mpc;
	json mpc_config = get_json_config(io_config["mpc_config"]); 
	mpc.set_config(mpc_config);
	mpc.ctrl.build_problem();
	mpc.start();

	Logger logger;

	auto timestep = 20ms;

	input_t input, input_target;
	pos_t raw_pos, filt_pos;

	M::s_vec s_est, s_predict, s_target, target_diff, s_mpc_tar;
	M::p_vec p_est;
	list<M::u_vec> u_buffer;
	M::u_vec u_mpc;
	int u_delay = io_config["u_delay"]; 
	double traj_correction = io_config["traj_correction"];

	auto start = steady_clock::now();
	auto next = start;
	
	char c;
	bool done = false;
	bool log_running = false;
	int log_timestep = 0;

	double input_c = io_config["input_c"];

	input.data.setZero();
	input_target.data.setZero();
	int ts = 0;
	int ctrl_step = -2;


	// -2 : startup, takeoff
	// -1 : manual
	// 0 : init target
	// 1-n : targets  

	while (!done)
	{

		input_target.pitch = keyboard_hndl['w'] - keyboard_hndl['s'];
		input_target.roll = keyboard_hndl['d'] - keyboard_hndl['a'];
		input_target.throttle = keyboard_hndl['i'] - keyboard_hndl['k'];
		input_target.yaw = keyboard_hndl['j'] - keyboard_hndl['l'];	

		raw_pos = vicon_hndl();

		if (keyboard_hndl['e'] & ctrl_step != -1) { // manual control
			cout << "\nMANUAL CONTROL" << endl;
			ctrl_step = -1;
		}
		if (keyboard_hndl['c'] & ctrl_step < 0) { // get to init target
			cout << "\nMPC CONTROL" << endl;
			ctrl_step = 0;
		}

		if (ctrl_step > -2) {
			filt_pos = vicon_filter.step(raw_pos, 1);
			mhe.get_est(s_est, p_est, ts);
			u_mpc = mpc.u_vector(ts);
		}


		if (ctrl_step == -1) {// manual
			input.data = input_c*input_target.data + (1 - input_c)*input.data;
			memcpy(s_target.data(), s_est.data(), sizeof(double)*M::s_dim);
		}
		else if (ctrl_step >= 0) {
			memcpy(s_target.data(), targets[ctrl_step].data(), sizeof(double)*M::s_dim);
			memcpy(input.data.data(), u_mpc.data(), sizeof(double)*M::u_dim);
		}


		if (keyboard_hndl['q']) {
			done = true;
			logger.close();
			mhe.end();
			mpc.end();

		}
		if (keyboard_hndl['t']) {
			tello1.takeOff();
			cout << "tello taking off" << endl;
		}
		else if (keyboard_hndl['r'] || keyboard_hndl['q']) {
			tello1.land();
			cout << "tello landing" << endl;
		}
		else {
			tello1.setStickData(false, input.roll, input.pitch, input.throttle, input.yaw);
		}

		u_buffer.push_back(input.data);
		while (u_buffer.size() > u_delay + 1)
		{
			u_buffer.pop_front();
		}

		if (ctrl_step > -2) {
			mhe.post_request(ts, filt_pos.data, u_buffer.front());
			s_predict = M::predict_state(s_est, u_buffer, p_est, 0.02);
			target_diff = s_target - s_est;
			mpc.post_request(ts+1, s_predict, u_buffer.back(), s_target, p_est);
		}	

		if (ctrl_step >= 0) {
			double target_dist = target_diff.norm();
			if (target_dist <= target_tol) {
				cout << '\n' << "Target " << ctrl_step << " reached. " << endl;
				
				if (ctrl_step == 0) {
					log_timestep = 0;
					log_running = true;
					string log_file = get_new_log_name(io_config["log_dir"], log_name);
					cout << "Opening log " << log_file << endl;
					logger.open(log_file);
				}

				ctrl_step += 1;

				if (ctrl_step >= targets.size()) {
					ctrl_step = 0;
					logger.close();
					log_running = false;
				}
			}
		}

		if (log_running) {
			logger << "pos" << log_timestep << s_est << '\n';
			logger << "input" << log_timestep << input.data << '\n';
			logger << "target" << log_timestep << s_target << '\n';
			logger << "param" << log_timestep << p_est << '\n';

			log_timestep += 1;
		}

		if (ctrl_step > -2) {
			ts += 1;
			cout << "ts: " << ts << ", tar diff: " << pos_t(target_diff) << ", input:" << input << "\r" << flush;
		}

		next += timestep;
		std::this_thread::sleep_until(next);
	}


	turn_on_console_echo();



	return 0;
}