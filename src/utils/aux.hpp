#ifndef __AUX_HPP__
#define __AUX_HPP__

#include <string>
#include <iostream>
#include <sstream>
#include <ctime>
#include <chrono>
#include <vector>
#include <filesystem>

#include <eigen3/Eigen/Dense>

#include "utils/json.hpp"

using namespace std;
namespace fs = std::filesystem;

using json = nlohmann::json;

template <typename Derived>
void normalize_cols(Eigen::MatrixBase<Derived> &mat)
{
    double var;
    for (int j = 0; j < mat.cols(); j++) {
        var = mat.col(j).array().abs2().mean();
        assert(var > 0);
        mat.col(j) = mat.col(j) / sqrt(var);
    }
}

template<typename Derived>
inline bool is_nan(const Eigen::MatrixBase<Derived>& x)
{
	return ((x.array() == x.array())).all();
}

template<typename Derived1, typename Derived2>
void append_row_to_matrix(Eigen::MatrixBase<Derived1> &mat, const Eigen::MatrixBase<Derived2> &vec)
{
	assert(mat.cols() == vec.rows() && vec.cols() == 1);
	mat.conservativeResize(mat.rows()+1, mat.cols());
	mat.row(mat.rows()-1) = vec;
}

vector<string> split_string(const string &str, const char del)
{
	vector<std::string> result;
    stringstream ss (str);
    string item;

    while (getline(ss, item, del)) {
        result.push_back(item);
    }

    return result;
}

inline bool file_exists(const string& name) {
    if (FILE *file = fopen(name.c_str(), "r")) {
        fclose(file);
        return true;
    } else {
        return false;
    }   
}

bool dir_exists(const string dir_name)
{
    fs::path dir_path(dir_name);
    return fs::is_directory(dir_path);
}

void delete_dir_content(const string dir_name) {
    fs::path dir_path(dir_name);
    for (auto& path: fs::directory_iterator(dir_path)) {
        if (!path.is_directory()) {
            fs::remove_all(path);
        }
    }
}

vector<string> list_files_in_dir(const string dir_name)
{
    vector<string> result;
    
    fs::path dir_path(dir_name);
    for (auto& entry: fs::directory_iterator(dir_path)) {
        if (!entry.is_directory()) {
            result.push_back(entry.path().filename().string());
        }
    }

    return result;
}

template<int S>
Eigen::Vector<double, S> array_to_vector(const double *array)
{
    Eigen::Vector<double, S> vector;

    for(int i = 0; i < S; i++) {
        vector[i] = array[i];
    }

    return vector; 
}

Eigen::Vector<double, -1> array_to_vector(vector<double> array)
{
    Eigen::Vector<double, -1> vector;
    vector.resize(array.size());

    for(int i = 0; i < array.size(); i++) {
        vector[i] = array[i];
    }

    return vector; 
}


json get_json_config(string file_name)
{
	if (!file_exists(file_name)) {
			cerr << "config file not found" << endl;
			exit(1);
		}

	ifstream config_file(file_name);
	json config = json::parse(config_file);

	return config;
}

#endif