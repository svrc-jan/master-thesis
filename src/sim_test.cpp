#include <chrono>
#include <iostream>
#include <fstream>

#include <cstdlib>
#include <cstdio>
#include <sys/stat.h>

#include <termios.h>

#include <eigen3/Eigen/Dense>

#include "model/drone_model.hpp"
#include "model/drone_sim.hpp"
#include "utils/keyboard_handler.hpp"


//static string keyboard_device = "/dev/input/by-path/platform-i8042-serio-0-event-kbd";
static string keyboard_device = "/dev/input/event9";


using namespace std;

int main(int argc, char const *argv[])
{
	
	using namespace std::chrono;

	Keyboard_handler keyboard_hndl(keyboard_device);
	
	ofstream log_file;
	log_file.open("data/" + string("sim"));

	mkfifo("/tmp/drone_data", 0777);
	system("python3 tools/monitor.py &");

	int pipe_fd = open("/tmp/drone_data", O_WRONLY);
	char buffer[256];

	input_t input;
	pos_t pos;

	Drone_sim sim(0.02);
	sim.reset();
	
	auto timestep = 20ms;
	auto start = steady_clock::now();
	auto next = start;

	bool done = false;
	while (!done)
	{

		input.pitch = keyboard_hndl['w'] - keyboard_hndl['s'];
		input.roll = keyboard_hndl['d'] - keyboard_hndl['a'];
		input.throttle = keyboard_hndl['i'] - keyboard_hndl['k'];
		input.yaw = keyboard_hndl['j'] - keyboard_hndl['l'];

		pos = sim.get_obs();

		cout << pos << " " << input << "\r" << endl;

		if (keyboard_hndl['q']) done = true;
		sim.step(input);

		sprintf(buffer, "%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f\n",
			pos.x, pos.y, pos.z, pos.a,
			input.roll, input.pitch, input.throttle, input.yaw);
		
		log_file << buffer;
		log_file.flush();

		next += timestep;
		std::this_thread::sleep_until(next);
	}

	turn_on_console_echo();
	log_file.close();
	close(pipe_fd);

	return 0;
}
