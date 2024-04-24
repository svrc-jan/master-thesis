#include <chrono>
#include <iomanip>
#include <fstream>

#include <cstdlib>
#include <cstdio>
#include <sys/stat.h>


#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <ceres/ceres.h>

#include "optim/model_ident.hpp"
#include "utils/parser.hpp"
#include "utils/logger.hpp"
#include "utils/json.hpp"

using namespace std;

using json = nlohmann::json;

string default_config = "/home/jsv/CVUT/master-thesis/config/est_sim.json";


int main(int argc, char const *argv[])
{
	string config_file = default_config;
	json config;
	if (argc > 1)
		config_file = string(argv[1]);


	config = get_json_config(config_file);

	typedef Innertia_drone_model M;

	Model_ident<M> model_ident;
	model_ident.set_config(config);
	
	Model_ident<M>::o_mat pos;
	Model_ident<M>::u_mat input;
	
	double dt = config["dt"];	
	int u_delay = config["u_delay"];
	int u_delay_max = config["u_delay_max"];

	int max_models = 1000;
	if (!config["max_models"].is_null()) {
		max_models = config["max_models"];
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

		model_ident.add_trajectory(pos, input);
	}

	


	ceres::Solver::Options options;
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	options.max_num_iterations = 100;
	options.num_threads = 6;
	// options.minimizer_progress_to_stdout = true;

	ceres::Solver::Summary summary;

	// model_ident.build_problem(dt, u_delay);
	// model_ident.solve(options, &summary);

	// cout << summary.BriefReport() << endl;
	
	int model_delay = u_delay;

	M::s_vec stat;
	double best_stat = -INFINITY;
	int best_delay = -1;

	while (true) {
		cout << "model delay " << model_delay << endl;
		model_ident.build_problem(dt, model_delay);
		model_ident.solve(options, &summary);
		cout << summary.BriefReport() << endl;

		best_stat = -INFINITY;
		best_delay = -1;

		for (int d = 0; d < u_delay_max; d++) {
			stat = model_ident.calculate_state_equation_corr(model_ident.param_est, dt, d);
			if (d == model_delay) {
				 cout << "state eq corr: " << stat.transpose() << endl;
			}

			if (stat.mean() > best_stat) {
				best_stat = stat.mean();
				best_delay = d;
			}
		}

		if (best_delay == model_delay) {
			break;
		}

		model_delay = best_delay;
	}
	

	// cout << "delay est" << model_delay << endl;

	if (!config["clear_log_est_dir"].is_null()) {
		if ((bool)config["clear_log_est_dir"]) {
			sprintf(buffer, "%s/est/", log_dir.c_str());
			delete_dir_content(buffer);
		}
	}

	for (int i = 0; i < min((int)log_files.size(), max_models); i++) {
		sprintf(buffer, "%s/est/%s", log_dir.c_str(), log_files[i].c_str());
		Logger logger(buffer);

		M::o_vec o;
		logger << "delay" << model_delay << '\n';
		logger << "params" << model_ident.param_est << '\n';
		for (int t = 0; t < model_ident.state_est[i]->rows(); t++) {
			M::output_eq(o.data(), model_ident.state_est[i]->row(t).data());
			logger << "pos" << t << o << '\n';
		}
	}

	cout << "par est " << model_ident.param_est.transpose() << endl;
	return 0;
}
