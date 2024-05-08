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
#include "utils/logger.hpp"
#include "utils/parser.hpp"

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

    MPC_handler<M> mpc;
    mpc.set_config(mpc_config);
    mpc.ctrl.build_problem();
	mpc.start();

	int mpc_u_delay = mpc_config["u_delay"];

	char buffer[256];
	string log_dir = sim_config["log_dir"];
	if (!sim_config["clear_log_dir"].is_null()) {
		if ((bool)sim_config["clear_log_dir"]) {
			delete_dir_content(log_dir);
			sprintf(buffer, "%s/mpc", log_dir.c_str());
			delete_dir_content(buffer);
		}
	}

	

	pos_t pos;
	input_t input;

    M::s_vec target;
    M::s_vec pos_pred;

	list<M::u_vec> u_buffer;

    double target_threshold = sim_config["target_threshold"];

    M::p_vec mpc_p;

	for (int sim_i = 0; sim_i < N_sim; sim_i++) {
		sprintf(buffer, "%s/%02d.log", log_dir.c_str(), sim_i+1);
		Logger logger(buffer);

		sprintf(buffer, "%s/mpc/%02d.log", log_dir.c_str(), sim_i+1);
		Logger mpc_logger(buffer);

		srand(time(nullptr));
		target = target.Random();
	    for (int i = M::o_dim; i < M::s_dim; i++) target[i] = 0;


		logger << "delay" << sim.u_delay << '\n';
		logger << "params" << sim.params << '\n';
        logger << "target" << 0 << target << '\n';

		mpc_p = sim.params + 0.05*sim.random_params_disturbance();
		// mpc_p = sim.params;

		mpc.reset();
		pos = sim.reset();
		int t = 0;


		auto timestep = int(dt*1000)*1ms;
		auto start = chrono::steady_clock::now();
		auto next = start;
		while (true) {
            logger << "pos" << t << pos.data << '\n';

            input = mpc.u_vector(t);
			logger << "input" << t << input.data << '\n';
			pos = sim.step(input);

			u_buffer.push_back(input);
			if (u_buffer.size() > mpc_u_delay+1) {
				u_buffer.pop_front();
			}

			pos_pred = M::predict_state(sim.state, u_buffer, mpc_p, dt);
			mpc.post_request(t + 1, pos_pred, u_buffer.back(), target, mpc_p);
			mpc_logger << "pos" << t << pos_pred << '\n';
			// auto mpc_start = chrono::high_resolution_clock::now();
			// mpc.ctrl.solve_problem(pos_pred, target, mpc_p);
			// auto mpc_end = chrono::high_resolution_clock::now();


			// auto duration = chrono::duration_cast<chrono::microseconds>(mpc_end - mpc_start);
			// cerr << "solution of request " << t << " took " << duration.count() << "us" << endl;
			
			t += 1;
			if (t >= T_max) break;
            double max_s_diff = (sim.state - target).cwiseAbs().maxCoeff();
            cout << "max state diff " << max_s_diff << endl;
            if (max_s_diff <= target_threshold) break;

			next += timestep;
			std::this_thread::sleep_until(next);
		}

		logger.close();
	}
	

	return 0;
}
