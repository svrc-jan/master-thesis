#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstring>

#include <eigen3/Eigen/Dense>

#include "model/drone_model.hpp"
#include "filter/vicon_filter.hpp"
#include "utils/parser.hpp"
#include "utils/logger.hpp"

using namespace std;

int main(int argc, char const *argv[])
{

	Vicon_filter vicon_filter(atof(argv[3]), atof(argv[4]), atof(argv[5]));

	Eigen::Matrix<double, -1, 4, Eigen::RowMajor> pos_data;
	Eigen::Matrix<double, -1, 4, Eigen::RowMajor> input_data;

	auto data = Parser::parse_log(argv[1], 
			{"input", "pos"},
			{1, 1});

	Parser::fill_matrix<4>(pos_data, data["pos"]);
	Parser::fill_matrix<4>(input_data, data["input"]);
 
	Logger logger(argv[2]);
	
	pos_t raw_pos, filt_pos;
	input_t input;

	char buffer[256];

	int t = 0;
	while (t < pos_data.rows()) {
		raw_pos.data = pos_data.row(t);
		filt_pos = vicon_filter.step(raw_pos, 1);
		input = input_data.row(t);
		
		logger << "pos" << t << filt_pos.data << '\n';
		logger << "input" << t << input.data << '\n';
		
		t += 1;
	}

	return 0;
}
