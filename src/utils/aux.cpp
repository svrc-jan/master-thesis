#include "aux.hpp"

string time_point_to_string(chrono::_V2::system_clock::time_point tp)
{
	time_t t = chrono::system_clock::to_time_t(tp);
	return string(ctime(&t));
}

template<int N>
void append_row_to_matrix(Eigen::Matrix<double, -1, N, 1, -1, N> *mat, Eigen::Matrix<double, N, 1, 0, N, 1> *vec)
{
	mat->conservativeResize(mat.rows()+1, mat.cols());
	mat->row(mat.rows()-1) = vec;
}