
#include <iostream>

#include "utils/matplotlibcpp.h"
#include "model/drone_model.hpp"
#include "vicon/vicon_client.h"


using namespace std;
namespace plt = matplotlibcpp;

#define PLOT_N 1000

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


	vector<double> vec_i = {0};
	vector<double> vec_x = {0};
	vector<double> vec_y = {0};
	vector<double> vec_z = {0};
	vector<double> vec_a = {0};


	vec_i.reserve(PLOT_N);
	vec_x.reserve(PLOT_N);
	vec_y.reserve(PLOT_N);
	vec_z.reserve(PLOT_N);
	vec_a.reserve(PLOT_N);

	vector<pos_t> vec;
	vec.reserve(10000);

	plt::Plot plot_x("plot");

	client.waitForData(object);
	pos_init = object;
	pos_init.a = 0;

	int step = 0;
	while (client.waitForData(object)) {
		step++;

		pos_abs = object;
		pos_rel = pos_abs - pos_init;

		vec.push_back(pos_rel);

		vec_i.clear();
		vec_x.clear();
		vec_y.clear();
		vec_z.clear();
		vec_a.clear();

	
		long n = min(vec.size(), (size_t)PLOT_N);
		for (size_t i = 0; i < n; i++) {
			pos_t *p = &(vec[vec.size() - n + i]);

			vec_i.push_back(vec.size() - n + i);
			vec_x.push_back(p->x);
			vec_y.push_back(p->y);
			vec_z.push_back(p->z);
			vec_a.push_back(p->a);
		
		}

		plot_x.update(vec_i, vec_x);

		// plt::subplot(2, 2, 1);
		// plt::plot(vec_x);

		// plt::subplot(2, 2, 2);
		// plt::plot(vec_y);

		// plt::subplot(2, 2, 3);
		// plt::plot(vec_z);

		// plt::subplot(2, 2, 4);
		// plt::plot(vec_a);

		plt::pause(0.001);


	}

	plt::show();

	return 0;
}
