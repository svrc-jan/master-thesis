#ifndef __AUX_HPP__
#define __AUX_HPP__

#include <string>
#include <ctime>
#include <chrono>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

using namespace std;

string time_point_to_string(chrono::steady_clock::time_point tp);


template<int N>
void append_row_to_matrix(Eigen::Matrix<double, -1, N, 1, -1, N> *mat, Eigen::Matrix<double, N, 1, 0, N, 1> *vec);


#endif