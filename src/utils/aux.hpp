#ifndef __AUX_HPP__
#define __AUX_HPP__

#include <string>
#include <ctime>
#include <chrono>

using namespace std;

string time_point_to_string(chrono::steady_clock::time_point tp);

#endif