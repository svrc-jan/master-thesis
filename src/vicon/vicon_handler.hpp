#ifndef __VICON_HANDLER_HPP__
#define __VICON_HANDLER_HPP__

#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>

#include "vicon_client.h"

using namespace std;

class Vicon_handler
{
private:
	thread handler_thread;
	atomic<bool> done = false;

	string host_ip = "127.0.0.1";
	int host_port = 51602;

	CViconClient client;

	mutex object_mtx;
	CViconObject object;

	friend void vicon_handler_func(Vicon_handler *hndl);

public:
	Vicon_handler(string ip);	
	~Vicon_handler();

	CViconObject get_val();

	CViconObject operator()() { return this->get_val(); }
};



#endif