
#include <iostream>


#include "model/drone_model.hpp"
#include "vicon/vicon_client.h"

using namespace std;

int main(int argc, char ** argv)
{
	const char * hostIp = "127.0.0.1";
	int hostPort = 51602;

	int a = 1;
	if (a < argc) {
		hostIp = argv[a++];
	}

	CViconClient client;
	if (!client.connect(hostIp, hostPort)) {
		return 1;
	}
	fprintf(stderr, "Connected\n");

	CViconObject object;
	
	pos_t pos_abs;
	pos_t pos_rel;
	pos_t pos_init;

	client.waitForData(object);
	pos_init = object;
	pos_init.a = 0;

	while (client.waitForData(object)) {
		pos_abs = object;
		pos_rel = pos_abs - pos_init;
	}

	return 0;
}
