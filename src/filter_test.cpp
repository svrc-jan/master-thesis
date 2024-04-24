#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstring>

#include "filter/vicon_filter.hpp"

using namespace std;

int main(int argc, char const *argv[])
{
	Vicon_filter *filt;

	if (argc <= 2) {
		filt = new Vicon_filter;
	}
	else if (argc == 3) {
		filt = new Vicon_filter(atof(argv[2]));
	}
	else if (argc == 4) {
		filt = new Vicon_filter(atof(argv[2]), atof(argv[3]));
	}
	else if (argc == 5) {
		filt = new Vicon_filter(atof(argv[2]), atof(argv[3]), atof(argv[4]));
	}
	// else {
	// 	cerr << "File name missing!" << endl;
	// 	return 1;
	// }

	string filename;
	if (argc > 1) 
		filename = string(argv[1]);

	else
		filename = "/home/jsv/CVUT/master-thesis/data/2f";

	fstream fin(filename, ios::in);
	if (!fin.is_open()) {
		cerr << "File not found!" << endl;
		return 1;
	}

	fstream fout(filename + "_filt", ios::out);
	if (!fout.is_open()) {
		cerr << "Cannot open file!" << endl;
		return 1;
	}

	pos_t pos, pos_f;
	input_t input;
	string line, val;

	vector<double> values;
	char buffer[256];
	
	while (!getline(fin, line).eof()) {
		values.clear();

		stringstream ss(line, ios::in);
		while (getline(ss, val, ',')) {
			values.push_back(stod(val));
		}
		
		for (size_t i = 0; i < 4; i++) {
			pos[i] = values[i];
			input[i] = values[i+4];
		}
		
		pos_f = filt->step(pos, true);
		
		sprintf(buffer, "%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f\n",
			pos_f.x, pos_f.y, pos_f.z, pos_f.a,
			input.roll, input.pitch, input.throttle, input.yaw);

		fout << buffer;
	}
	
	
	

	delete filt;

	return 0;
}
