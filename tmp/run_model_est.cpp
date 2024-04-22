#include <chrono>
#include <iomanip>
#include <fstream>

#include <cstdlib>
#include <cstdio>
#include <sys/stat.h>


#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <ceres/ceres.h>

#include "model/model_sim.hpp"
#include "optim/model_est.hpp"
#include "logging/parser.hpp"
#include "logging/logger.hpp"
#include "utils/json.hpp"

using namespace std;

using json = nlohmann::json;

string default_config = "/home/jsv/CVUT/master-thesis/config/est_sim.json";

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

	json config;
	if (argc > 1)
		config = get_config(argv[1]);

	else
		config = get_config(default_config);

	typedef Innertia_drone_model M;

	Model_est<M> model_est;
	model_est.set_config(config);
	
	Model_est<M>::o_mat pos;
	Model_est<M>::u_mat input;
	
	double dt = config["dt"];	
	int u_delay = config["u_delay"];


	int max_models = 1000;
	if (!config["max_models"].is_null()) {
		max_models = config["max_models"];
	}
	if (argc > 2) {
		max_models = atoi(argv[2]);
	}

	string log_dir(config["log_dir"]);
	vector<string> log_files = list_files_in_dir(log_dir);

	cout << "using files:" << endl;


	char buffer[256];
	for (int i = 0; i < min((int)log_files.size(), max_models); i++) {
		sprintf(buffer, "%s/%s", log_dir.c_str(), log_files[i].c_str());
		cout << buffer << endl;
		
		auto data = Parser::parse_log(buffer, 
			{"input", "pos"},
			{1, 1});

		Parser::fill_matrix<M::o_dim>(pos, data["pos"]);
		input.conservativeResize(pos.rows(), input.cols());
		Parser::fill_matrix<M::u_dim>(input, data["input"]);

		model_est.add_trajectory(pos, input);
	}

	ceres::Solver::Options options;
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	options.max_num_iterations = 100;
	options.num_threads = 6;
	options.minimizer_progress_to_stdout = true;
	ceres::Solver::Summary summary;

	int best_delay = -1;
	int model_delay = config["u_delay"];
	M::s_vec stat;

	model_est.build_problem(dt, model_delay);
	model_est.solve(options, &summary);
	
	cout << summary.BriefReport() << endl;

	double best_stat = INFINITY;
	best_delay = -1;

	for (int delay = 0; delay < 100; delay++) {
		stat = model_est.calculate_state_equation_missmatch(dt, delay);
		cout << "delay" << delay << " | " << stat.transpose() << endl;
		if (stat.sum() < best_stat) {
			best_stat = stat.sum();
			best_delay = delay;
		}
	}

	for (int i = 0; i < min((int)log_files.size(), max_models); i++) {
		sprintf(buffer, "%s/est/%s", log_dir.c_str(), log_files[i].c_str());
		Logger logger(buffer);

		M::o_vec o;
		logger << "u_delay" << model_delay << '\n';
		logger << "params" << model_est.param_est << '\n';
		for (int t = 0; t < model_est.state_est[i]->rows(); t++) {
			M::output_eq(o.data(), model_est.state_est[i]->row(t).data());
			logger << "pos" << t << o << '\n';
		}
	}

	cout << "delay est" << best_delay << endl;
	cout << "par est " << model_est.param_est.transpose() << endl;
	return 0;
}
