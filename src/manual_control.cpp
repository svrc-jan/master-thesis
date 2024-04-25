#include <chrono>
#include <iostream>
#include <thread>
#include <cstdlib>
#include <thread>
#include <algorithm>

#include <termios.h>

#include "model/drone_model.hpp"
#include "tello/tello.h"
#include "vicon/vicon_handler.hpp"
#include "filter/vicon_filter.hpp"
#include "utils/keyboard_handler.hpp"
#include "utils/logger.hpp"
#include "utils/aux.hpp"


using namespace std;

string io_config_file = "config/real_io.json";


int main(int argc, char const *argv[])
{
	
	using namespace std::chrono;

	json io_config = get_json_config(io_config_file);

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


	auto timestep = 20ms;

	input_t input, input_target;
	pos_t raw_pos, filt_pos;


	auto start = steady_clock::now();
	auto next = start;

	string log_file;
	char buffer[256];

	assert(argc > 1);
	
	sprintf(buffer, "%s/raw/%s.log", string(io_config["log_dir"]).c_str(), argv[1]);
	Logger raw_logger(buffer);
	cout << "raw log " << buffer << endl;
	
	sprintf(buffer, "%s/filt/%s.log", string(io_config["log_dir"]).c_str(), argv[1]);
	Logger filt_logger(buffer);
	cout << "filt log " << buffer << endl;
	
	
	char c;
	bool done = false;
	bool log_running = false;
	int log_timestep = 0;

	double input_c = io_config["input_c"];

	input.data.setZero();
	input_target.data.setZero();

	while (!done)
	{

		input_target.pitch = keyboard_hndl['w'] - keyboard_hndl['s'];
		input_target.roll = keyboard_hndl['d'] - keyboard_hndl['a'];
		input_target.throttle = keyboard_hndl['i'] - keyboard_hndl['k'];
		input_target.yaw = keyboard_hndl['j'] - keyboard_hndl['l'];	

		raw_pos = vicon_hndl();

		input.data = input_c*input_target.data + (1 - input_c)*input.data;

		if (keyboard_hndl['q']) done = true;
		if (keyboard_hndl['t']) {
			tello1.takeOff();
			cout << "tello taking off" << endl;
		}
		if (keyboard_hndl['r'] || keyboard_hndl['q']) {
			tello1.land();
			cout << "tello landing" << endl;
		}
		else {
			tello1.setStickData(true, input.roll, input.pitch, input.throttle, input.yaw);
		}

		if (keyboard_hndl['f'] && !log_running) {
			log_running = true;
			log_timestep = 0;
			cout << "starting log" << endl;
		}

		if (keyboard_hndl['h'] && log_running) {
			log_running = false;
			cout << "stopping log" << endl;
		}

		if (log_running) {
			filt_pos = vicon_filter.step(raw_pos, 1);

			raw_logger << "pos" << log_timestep << raw_pos.data << '\n';
			raw_logger << "input" << log_timestep << input.data << '\n';
			
			filt_logger << "pos" << log_timestep << filt_pos.data << '\n';
			filt_logger << "input" << log_timestep << input.data << '\n';

			cout << "log timestep: " << log_timestep << ", pos: " << raw_pos << ", input:" << input << "    \r" << flush;
			log_timestep += 1;
		}
 

		next += timestep;
		std::this_thread::sleep_until(next);
	}

	turn_on_console_echo();



	return 0;
}