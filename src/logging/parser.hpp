#ifndef __PARSER_HPP__
#define __PARSER_HPP__

#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <map>



#include "utils/aux.hpp"
#include <eigen3/Eigen/Dense>

using namespace std;



class Parser
{
public:
	static map<string, vector<vector<double>>> parse_log(string file_name, vector<string> tags, vector<bool> has_id);
	
	template<int S>
	static void fill_matrix(Eigen::Matrix<double, -1, S, Eigen::RowMajor> &mat, vector<vector<double>> &data);



	template<int S>
	static void fill_vector(Eigen::Vector<double, S> &vec, vector<double> &data);

	template<int S>
	static void fill_vector(Eigen::Vector<double, S> &vec, vector<vector<double>> &data)
	{
		fill_vector<S>(vec, data[0]);
	}

	
};

map<string, vector<vector<double>>> Parser::parse_log(string file_name, vector<string> tags, vector<bool> has_id)
{
	assert(tags.size() == has_id.size());
	ifstream file(file_name);
	assert(file.is_open());

	map<string, vector<vector<double>>> result;

	string line("");
	vector<string> line_split;

	while(getline(file, line)) {
		line_split = split_string(line, ',');

		for (int i = 0; i < tags.size(); i++) {

			if (line_split[0].compare(tags[i]) == 0) {
				if (has_id[i]) {
					int id = stoi(line_split[1]);
					for (int j = 2; j < line_split.size(); j++) {
						if (result[tags[i]].size() <= id) {
							result[tags[i]].resize(id+1);
						}
						result[tags[i]][id].push_back(stod(line_split[j]));
					}
				}

				else {
					for (int j = 1; j < line_split.size(); j++) {
						if (result[tags[i]].size() <= 1) {
							result[tags[i]].resize(1);
						}
						result[tags[i]][0].push_back(stod(line_split[j]));
					}
				}

				break;
			}
		}
	}

	return result;
}

template<int S>
void Parser::fill_matrix(Eigen::Matrix<double, -1, S, Eigen::RowMajor> &mat, vector<vector<double>> &data)
{
	if (mat.rows() < data.size()) {
		mat.conservativeResize(data.size(), mat.cols());
	}
	int i, j;
	for(i = 0; i < data.size(); i++) {
		for (j = 0; j < data[i].size(); j++) {
			mat(i,j) = data[i][j];
		}
		for (; j < mat.cols(); j++) {
			if (i == 0)
				mat(i, j) = 0;
			
			else
				mat(i, j) = mat(i-1, j);
		}


	}

	for(; i < mat.rows(); i++) {
		for (j = 0; j < mat.cols(); j++) {
			if (i == 0)
				mat(i, j) = 0;
			
			else
				mat(i, j) = mat(i-1, j);
		}
	}
}

template<int S>
void Parser::fill_vector(Eigen::Vector<double, S> &vec, vector<double> &data)
{
	for (int i; i < data.size(); i++)
		vec(i) = double(data[i]);
}


#endif