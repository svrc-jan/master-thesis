#include "vicon_handler.hpp"

void vicon_handler_func(Vicon_handler *hndl)
{
	if (!hndl->client.connect(hndl->host_ip.c_str(), hndl->host_port)) {
		cerr << "Cannot connect to vicon server: " << hndl->host_ip << ":" << hndl->host_port << endl;
		exit(EXIT_FAILURE);
	}

	CViconObject obj;
	while (hndl->client.waitForData(obj) && !hndl->done)
	{
		hndl->object_mtx.lock();
		hndl->object = obj;
		hndl->object_mtx.unlock();
	}

	hndl->client.close();
}

CViconObject Vicon_handler::get_val()
{
	CViconObject obj;
	
	this->object_mtx.lock();
	obj = this->object;
	this->object_mtx.unlock();

	return obj;
}


Vicon_handler::Vicon_handler(string ip)
{
	this->host_ip = ip;
	this->handler_thread = thread(vicon_handler_func, this);
}

Vicon_handler::~Vicon_handler()
{
	this->done = true;
	this->handler_thread.join();
}
