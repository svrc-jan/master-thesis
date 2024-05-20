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
string log_num = "default";


int main(int argc, char const *argv[])
{
	
	using namespace std::chrono;

	json io_config = get_json_config(io_config_file);

	char buffer[256];

	if (argc > 1) {
		log_num = string(argv[1]);
	}
	
	sprintf(buffer, "%s/raw/%s.log", string(io_config["log_dir"]).c_str(), log_num.c_str());
	Logger raw_logger(buffer);
	cout << "raw log " << buffer << endl;
	
	sprintf(buffer, "%s/filt/%s.log", string(io_config["log_dir"]).c_str(), log_num.c_str());
	Logger filt_logger(buffer);
	cout << "filt log " << buffer << endl;

	sprintf(buffer, "%s/mhe/%s.log", string(io_config["log_dir"]).c_str(), log_num.c_str());
	Logger mhe_logger(buffer);
	cout << "mhe log " << buffer << endl;
	

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

	typedef Simple_drone_model M;

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

	auto timestep = 20ms;

	input_t input, input_target;
	pos_t raw_pos, filt_pos;

	M::s_vec s_est, s_predict, s_target;
	M::p_vec p_est;
	list<M::u_vec> u_buffer;
	M::u_vec u_mpc;
	int u_delay = io_config["u_delay"]; 

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
	int ctrl_mode = 0; //0  1 is manual 2 is mpc

	while (!done)
	{

		input_target.pitch = keyboard_hndl['w'] - keyboard_hndl['s'];
		input_target.roll = keyboard_hndl['d'] - keyboard_hndl['a'];
		input_target.throttle = keyboard_hndl['i'] - keyboard_hndl['k'];
		input_target.yaw = keyboard_hndl['j'] - keyboard_hndl['l'];	

		raw_pos = vicon_hndl();

		if (keyboard_hndl['e']) {
			ctrl_mode = 1;
		}
		if (keyboard_hndl['c'] && ctrl_mode == 1) {
			ctrl_mode = 2;
		}

		if (ctrl_mode > 0) {
			filt_pos = vicon_filter.step(raw_pos, 1);
			mhe.get_est(s_est, p_est, ts);
			mpc.u_vector(ts);
		}


		if (ctrl_mode == 1) {// manual
			input.data = input_c*input_target.data + (1 - input_c)*input.data;
			memcpy(s_target.data(), s_est.data(), sizeof(double)*M::s_dim);
		}
		else if (ctrl_mode == 2) {//
			s_target[0] = s_target[0] + input_c*input_target.pitch;
			s_target[1] = s_target[1] - input_c*input_target.roll;
			s_target[2] = s_target[2] + input_c*input_target.throttle;
			s_target[3] = s_target[3] + input_c*input_target.yaw;
		}


		u_mpc = mpc.u_vector(ts);


		if (keyboard_hndl['q']) {
			done = true;
			raw_logger.flush();
			filt_logger.flush();
			mhe_logger.flush();
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
			if (ctrl_mode == 2) {
				memcpy(input.data.data(), u_mpc.data(), sizeof(double)*M::u_dim);
			}			
			tello1.setStickData(false, input.roll, input.pitch, input.throttle, input.yaw);
	
		}

		u_buffer.push_back(input.data);
		while (u_buffer.size() > u_delay + 1)
		{
			u_buffer.pop_front();
		}

		if (ctrl_mode > 0) {
			mhe.post_request(ts, filt_pos.data, u_buffer.front());
			s_predict = M::predict_state(s_est, u_buffer, p_est, 0.02);
			mpc.post_request(ts+1, s_predict, u_buffer.back(), s_target, p_est);
		}	

		if (keyboard_hndl['f'] && !log_running) {
			log_running = true;
			log_timestep = 0;
			cout << "starting log" << endl;
		}

		if (keyboard_hndl['h'] && log_running) {
			ctrl_mode = 1;
			log_running = false;
			cout << "stopping log" << endl;
		}

		if (log_running) {

			raw_logger << "pos" << log_timestep << raw_pos.data << '\n';
			raw_logger << "input" << log_timestep << input.data << '\n';
			raw_logger << "target" << log_timestep << s_target << '\n';
			
			filt_logger << "pos" << log_timestep << filt_pos.data << '\n';
			filt_logger << "input" << log_timestep << input.data << '\n';
			filt_logger << "target" << log_timestep << s_target << '\n';

			mhe_logger << "pos" << log_timestep << s_est << '\n';
			mhe_logger << "input" << log_timestep << input.data << '\n';
			mhe_logger << "target" << log_timestep << s_target << '\n';
			mhe_logger << "param" << log_timestep << p_est << '\n';
			

			pos_t target_diff = pos_t(s_target - s_est);
			cout << "log timestep: " << log_timestep << ", target_diff: " << target_diff << ", input:" << input << "    \r" << flush;
			log_timestep += 1;
		}

		if (ctrl_mode > 0) {
			ts += 1;
		}
		next += timestep;
		std::this_thread::sleep_until(next);
	}

	turn_on_console_echo();



	return 0;
}