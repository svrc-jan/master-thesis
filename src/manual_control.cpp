#include <chrono>
#include <iostream>
#include <thread>
#include <cstdlib>
#include <thread>

#include <curses.h>

#include "model/drone_model.hpp"
#include "tello/tello.h"


int main(int argc, char const *argv[])
{
	
	using namespace std::chrono;

	WINDOW *win;
	initscr();
	
    clear();
    noecho();
    cbreak();
	nodelay(stdscr, true);
	
	CTello tello1;
    tello1.init(8889, "wlxd0374543f3a6");
    // tello1.init(8889, "wlxd0374576c511");


	auto timestep = 50ms;
	auto next = steady_clock::now();

	char c;
	bool done = false;

	while (getch() != 't');


	while (!done)
	{
		auto now = steady_clock::now();
		bool take_off = false;
		input_t input;

		while ((c = getch()) != ERR) {
			switch (c) {
			case 'w':
				input.pitch = 1;
				break;

			case 's':
				input.pitch = -1;
				break;

			case 'd':
				input.roll = 1;
				break;

			case 'a':
				input.roll = -1;
				break;

			case 'i':
				input.throttle = 1; 
				break;

			case 'k':
				input.throttle = -1;
				break;

			case 'l':
				input.yaw = 1;
				break;

			case 'j':
				input.yaw = -1;
				break;

			case 't':
				take_off = true;
				break;

			case 'q':
				done = true;
				break;


			default:
				break;
			}
		}
		
		cout << input << "\r" << endl;

		if (take_off) {
			tello1.takeOff();
		}
		else {
			tello1.setStickData(true, input.roll, input.pitch, input.throttle, input.yaw);
		}

		next += timestep;
		std::this_thread::sleep_until(next);
	}




	tello1.land();

	nodelay(stdscr, false);
	echo();
	endwin();	

	return 0;
}
