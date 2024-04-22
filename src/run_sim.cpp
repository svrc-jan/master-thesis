#include <chrono>
#include <iostream>
#include <fstream>

#include <cstdlib>
#include <cstdio>
#include <sys/stat.h>

#include "model/drone_model.hpp"
#include "model/model_sim.hpp"
#include "logging/logger.hpp"
#include "logging/parser.hpp"

//static string keyboard_device = "/dev/input/by-path/platform-i8042-serio-0-event-kbd";
//static string keyboard_device = "/dev/input/event9";


using namespace std;

using json = nlohmann::json;

string default_config = "/home/jsv/CVUT/master-thesis/config/sim.json";

json get_config(string file_name)
{
	if (!file_exists(file_name)) {
			cerr << "config file not found" << endl;
			exit(1);
		}

	ifstream config_file(file_name);
	json config = json::parse(config_file);

	return config;
}

int main(int argc, char const *argv[])
{
	typedef Innertia_drone_model M;

	json config;
	if (argc > 1) 
		config = get_config(argv[1]);

	else{
		config = get_config(default_config);
	}	

	double dt = config["dt"];
	int T_max = config["T_max"];
	int T_u_change = config["T_u_change"];
	int N_sim = config["N_sim"];

	Model_sim<M> sim(dt);
	sim.set_config(config);

	cout << "o noise sd: " << sim.o_noise_sd.transpose() << endl;


	string log_dir = config["log_dir"];
	if (!config["clear_log_dir"].is_null()) {
		if ((bool)config["clear_log_dir"]) {
			delete_dir_content(log_dir);
		}
	}

	char buffer[256];

	pos_t pos;
	input_t input;
	M::u_vec target;

	double tar_ratio = 0.1;

	for (int sim_i = 0; sim_i < N_sim; sim_i++) {
		sprintf(buffer, "%s/%02d.log", log_dir.c_str(), sim_i+1);
		string log_name(buffer);

		Logger logger(log_name);

		logger << "params" << sim.params << '\n';

		pos = sim.reset();
		int t = 0;
		while (true) {
			logger << "pos" << t << pos.data << '\n';
			if (t % T_u_change == 0) {
				target = sim.random_input();
				input.data.setZero();
			}

			input = (1 - tar_ratio)*input.data + tar_ratio*target;
			

			logger << "input" << t << input.data << '\n';
			pos = sim.step(input);
			logger << "curr_delay" << sim.u_delay_curr << '\n';
			

			t += 1;
			if (t >= T_max) break;
		}

		logger.close();
	}
	

	return 0;
}
