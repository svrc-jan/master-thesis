#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstring>

#include <eigen3/Eigen/Dense>

#include "model/drone_model.hpp"
#include "optim/mhe.hpp"
#include "filter/vicon_filter.hpp"
#include "utils/parser.hpp"
#include "utils/logger.hpp"

using namespace std;

string mhe_config_file = "/home/jsv/CVUT/master-thesis/config/mhe_real.json";

int main(int argc, char const *argv[])
{
	typedef Simple_drone_model M;

	Eigen::Matrix<double, -1, M::o_dim, Eigen::RowMajor> pos_data;
	Eigen::Matrix<double, -1, M::u_dim, Eigen::RowMajor> input_data;

	auto data = Parser::parse_log(argv[1], 
			{"input", "pos"},
			{1, 1});

	Parser::fill_matrix<4>(pos_data, data["pos"]);
	Parser::fill_matrix<4>(input_data, data["input"]);
 
	Logger logger(argv[2]);
	
	M::o_vec obs;
	M::s_vec pos_est;
	M::u_vec input;
	M::p_vec param_est;

	json mhe_config = get_json_config(mhe_config_file);

	int u_delay = mhe_config["u_delay"];
	
	MHE_handler<M> mhe;
	mhe.set_config(mhe_config);
	mhe.estim.build_problem();
	mhe.start();


	char buffer[256];

	
	input.setZero();
	list<M::u_vec> u_buffer;
	u_buffer.push_back(input);

	int t = 0;
	double dt = mhe_config["dt"];

	auto timestep = int(dt*1000/3)*1ms;
	auto start = chrono::steady_clock::now();
	auto next = start;

	while (t < pos_data.rows()) {
		obs = pos_data.row(t);
		
		if (mhe.sol.ts >= 0) {
			cout << endl << "ts " << t << "/" << pos_data.rows() << " "; 
			mhe.get_est(pos_est, param_est, t);
		}
		else {
			pos_est = obs;
			param_est = mhe.estim.p_prior;
		}

		input = input_data.row(t);

		mhe.post_request(t, obs, u_buffer.front());

		u_buffer.push_back(input);
		if (u_buffer.size() > u_delay+1) {
			u_buffer.pop_front();
		}

		logger << "pos" << t << pos_est << '\n';
		logger << "param" << t << param_est << '\n';

		t += 1;
		next += timestep;
		std::this_thread::sleep_until(next);
	}

	cout << endl << "par est " << param_est.transpose() << endl;

	return 0;
}
