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
#include "utils/keyboard_handler.hpp"
#include "utils/aux.hpp"

using namespace std;

static string keyboard_device = "/dev/input/by-path/platform-i8042-serio-0-event-kbd";
static string vicon_ip = "192.168.2.100";


int main(int argc, char const *argv[])
{
	
	using namespace std::chrono;

	CTello tello1;
    tello1.init(8889, "wlxd0374543f3a6");    // tello1.init(8889, "wlxd0374576c511");

	Keyboard_handler keyboard_hndl(keyboard_device);
	Vicon_handler vicon_hndl(vicon_ip);

	auto timestep = 20ms;

	char c;
	bool done = false;

	input_t input;
	pos_t pos;


	auto start = steady_clock::now();
	auto next = start;

	ofstream log_file;
	log_file.open("data/" + string(argv[1]));
	char buffer[256];

	while (!done)
	{

		input.pitch = keyboard_hndl['w'] - keyboard_hndl['s'];
		input.roll = keyboard_hndl['d'] - keyboard_hndl['a'];
		input.throttle = keyboard_hndl['i'] - keyboard_hndl['k'];
		input.yaw = keyboard_hndl['j'] - keyboard_hndl['l'];

		pos = vicon_hndl();

		cout << pos << "\r" << endl;



		if (keyboard_hndl['q']) done = true;
		if (keyboard_hndl['t']) {
			tello1.takeOff();
		}
		else {
			tello1.setStickData(true, input.roll, input.pitch, input.throttle, input.yaw);
		}

		sprintf(buffer, "%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f\n",
			pos.x, pos.y, pos.z, pos.a,
			input.roll, input.pitch, input.throttle, input.yaw);

		log_file << buffer;

		next += timestep;
		std::this_thread::sleep_until(next);
	}

	turn_on_console_echo();


	tello1.land();



	return 0;
}
