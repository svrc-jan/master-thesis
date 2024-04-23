#include <chrono>
#include <iostream>
#include <fstream>
#include <chrono>

#include <cstdlib>
#include <cstdio>
#include <sys/stat.h>

#include "model/drone_model.hpp"
#include "model/model_sim.hpp"
#include "optim/mpc.hpp"
#include "logging/logger.hpp"
#include "logging/parser.hpp"

//static string keyboard_device = "/dev/input/by-path/platform-i8042-serio-0-event-kbd";
//static string keyboard_device = "/dev/input/event9";


using namespace std;

using json = nlohmann::json;

string default_sim_config = "/home/jsv/CVUT/master-thesis/config/sim_mpc.json";
string default_mpc_config = "/home/jsv/CVUT/master-thesis/config/mpc.json";


int main(int argc, char const *argv[])
{
	typedef Innertia_drone_model M;

	json sim_config, mpc_config;
    string sim_config_file = default_sim_config;
    string mpc_config_file = default_mpc_config;
	if (argc > 1) 
		mpc_config_file = argv[1];

	if (argc > 2)
        sim_config_file = argv[2];

    sim_config = get_json_config(sim_config_file);
    mpc_config = get_json_config(mpc_config_file);

	double dt = sim_config["dt"];
	int T_max = sim_config["T_max"];
	int N_sim = sim_config["N_sim"];

	Model_sim<M> sim;
	sim.set_config(sim_config);

    MPC_controller<M> mpc;
    mpc.set_config(mpc_config);
    mpc.build_problem();

	string log_dir = sim_config["log_dir"];
	if (!sim_config["clear_log_dir"].is_null()) {
		if ((bool)sim_config["clear_log_dir"]) {
			delete_dir_content(log_dir);
		}
	}

	char buffer[256];

	pos_t pos;
	input_t input;

    M::s_vec target;
    target = target.Random();
    for (int i = 4; i < 8; i++) target[i] = 0;

    double target_threshold = mpc_config["target_threshold"];

    M::p_vec mpc_p = sim.params + 0.05*sim.random_params_disturbance();

	for (int sim_i = 0; sim_i < N_sim; sim_i++) {
		sprintf(buffer, "%s/%02d.log", log_dir.c_str(), sim_i+1);
		string log_name(buffer);

		Logger logger(log_name);

		logger << "delay" << sim.u_delay << '\n';
		logger << "params" << sim.params << '\n';
        logger << "target" << 0 << target << '\n';

		pos = sim.reset();
		int t = 0;
		while (true) {
            logger << "pos" << t << pos.data << '\n';

            auto start = chrono::high_resolution_clock::now();
            mpc.shift_u_arr();
            mpc.solve_problem(sim.state, target, mpc_p);
            auto stop = chrono::high_resolution_clock::now();
            auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);

            cout << "t:" << t << ", solve time: " << duration.count() << "us" << endl;

            input = mpc.u_vector(0);
			logger << "input" << t << input.data << '\n';
			pos = sim.step(input);
			
			t += 1;
			if (t >= T_max) break;
            double max_s_diff = (sim.state - target).cwiseAbs().maxCoeff();
            cout << "max state diff " << max_s_diff << endl;
            if (max_s_diff <= target_threshold) break;
		}

		logger.close();
	}
	

	return 0;
}
