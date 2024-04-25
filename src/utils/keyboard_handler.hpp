#ifndef __KEYBOARD_HANDLER_HPP__
#define __KEYBOARD_HANDLER_HPP__

#include <string>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>
#include <iostream>

#include <fcntl.h>
#include <unistd.h>

#include <linux/input.h>
#include <linux/input-event-codes.h>

#include <termios.h>


using namespace std;

static map<char, size_t> key_map = {
	{ 'w', KEY_W},
	{ 'a', KEY_A},
	{ 's', KEY_S},
	{ 'd', KEY_D},
	{ 'i', KEY_I},
	{ 'j', KEY_J},
	{ 'k', KEY_K},
	{ 'l', KEY_L},
	{ 'q', KEY_Q},
	{ 't', KEY_T},
	{ 'f', KEY_F},
	{ 'h', KEY_H},
	{ 't', KEY_T},
	{ 'r', KEY_R}
};

class Keyboard_handler
{
private:
	thread handler_thread;
	string device_name;

	atomic<bool> done = false;

	mutex array_mtx;
	bool array[256] = {0};

	friend void keyboard_handler_func(Keyboard_handler *hndl);

public:

	Keyboard_handler(string device);
	~Keyboard_handler();

	bool operator[](char c) { return this->get_val(key_map[c]); }
	bool operator[](size_t code) { return this->get_val(code); }

	bool get_val(size_t code);
};

void turn_off_console_echo();
void turn_on_console_echo();


#endif
