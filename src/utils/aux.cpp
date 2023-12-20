#include "aux.hpp"

string time_point_to_string(chrono::_V2::system_clock::time_point tp)
{
	time_t t = chrono::system_clock::to_time_t(tp);
	return string(ctime(&t));
}