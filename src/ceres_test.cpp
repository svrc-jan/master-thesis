#include <chrono>
#include <iostream>
#include <iomanip>
#include <fstream>

#include <cstdlib>
#include <cstdio>
#include <sys/stat.h>

#include <termios.h>

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>

#include "model/drone_model.hpp"
#include "model/drone_sim.hpp"
#include "utils/keyboard_handler.hpp"
#include "utils/aux.hpp"
#include "optim/model_est.hpp"

//static string keyboard_device = "/dev/input/by-path/platform-i8042-serio-0-event-kbd";
static string keyboard_device = "/dev/input/event9";


using namespace std;



int main(int argc, char const *argv[])
{
	// Keyboard_handler keyboard_hndl(keyboard_device);

	// logging and monotoring	
	ofstream log_file;
	log_file.open("data/" + string("sim"));

	char buffer[256];

	input_t input;
	pos_t obs;


	Eigen::Matrix<double, -1, 4, Eigen::RowMajor> states_obs; // observations
	Eigen::Matrix<double, -1, 4, Eigen::RowMajor> inputs;

	states_obs.resize(0, 4);
	inputs.resize(0, 4);
	states_obs.resize(0, 4);


	
	cout << "running" << endl;


	auto rand_c_hor = uniform_real_distribution<double>(7, 20);
	auto rand_c_ver = uniform_real_distribution<double>(7, 20);
	auto rand_c_ang = uniform_real_distribution<double>(5, 10);
	auto rand_ang_err = uniform_real_distribution<double>(-0.3, 0.3);
	random_device rng;

	double dt = 0.01;
	double c_mass = 10;
	if (argc > 2) c_mass = atof(argv[2]);
	double c_hor = rand_c_hor(rng);
	double c_ver = rand_c_ver(rng);
	double c_ang = rand_c_ang(rng);
	double ang_err = rand_ang_err(rng);

	int T_max = 1000;
	if (argc > 1) T_max = atoi(argv[1]);

	Drone_sim sim(dt, c_mass, c_hor, c_ver, c_ang, ang_err, 7, 0.05);
	sim.reset();
	input = sim.random_input();


	cout << setprecision(3);
	cout << "model_par_rea: " << c_mass << " | " << c_hor << " | " << c_ver << " | " << c_ang << " | " << ang_err << endl;
	
	using namespace std::chrono;
	duration<int, milli> timestep(int(1000*dt));
	auto start = steady_clock::now();
	auto next = start;

	int t = 0;
	bool done = false;
	while (!done)
	{
		
		// input.pitch = keyboard_hndl['w'] - keyboard_hndl['s'];
		// input.roll = keyboard_hndl['d'] - keyboard_hndl['a'];
		// input.throttle = keyboard_hndl['i'] - keyboard_hndl['k'];
		// input.yaw = keyboard_hndl['j'] - keyboard_hndl['l'];

		if (t % 100 == 0)
			input = sim.random_input();

		obs = sim.get_obs();

		inputs.conservativeResize(t+1, inputs.cols());
		inputs.row(t) = input.to_vec();

		states_obs.conservativeResize(t+1, states_obs.cols());
		states_obs.row(t) = obs.to_vec();


		// if (keyboard_hndl['q']) done = true;
		sim.step(input);

		sprintf(buffer, "%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f\n",
			obs.x, obs.y, obs.z, obs.a,
			input.roll, input.pitch, input.throttle, input.yaw);
		
		log_file << buffer;

		if ((t + 1) % 10 == 0) log_file.flush();

		next += timestep;
		
		t += 1;
		// std::this_thread::sleep_until(next);

		// cout << t << ": " << obs << input << endl;

		if (t >= T_max) break;
	}

	cout << "ending";

	turn_on_console_echo();
	log_file.close();


	Eigen::Matrix<double, -1, 4, Eigen::RowMajor> states_est; // states estimate
	Eigen::Matrix<double, -1, 4, Eigen::RowMajor> input_st_est;

	states_est = states_obs;
	input_st_est = inputs;

	double model_c_mass_est = 0;
	double model_par_est[4] = {12, 12, 5, 0};

	Model_est model_est(100, 1, 10, 0.3, 0.1);

	for (int i = 0; i < t; i++) {

		if (i <  t - 1) {
			model_est.add_state_mass(model_par_est, states_est.row(i).data(), 
				states_est.row(i+1).data(), input_st_est.row(i).data(), dt);


			model_est.add_input_mass(inputs.row(i+1).data(), &model_c_mass_est, 
				input_st_est.row(i).data(), input_st_est.row(i+1).data(), dt);
		}

		model_est.add_obs(states_obs.row(i).data(), states_est.row(i).data());

	}

	double model_c_mass_prior = model_c_mass_est;
	double model_par_prior[4] = {12, 12, 5, 0};


	double model_c_mass_C = 0.001;
	double model_par_C[4] = {0.01, 0.01, 0.01, 0.01};

	model_est.add_prior<1>(&model_c_mass_prior, &model_c_mass_est, &model_c_mass_C);
	model_est.add_prior<4>(model_par_prior, model_par_est, model_par_C);

	ceres::Solver::Options options;
	options.linear_solver_type = ceres::SPARSE_SCHUR;
	options.minimizer_progress_to_stdout = true;
	options.max_num_iterations = 100;
	options.num_threads = 6;

	ceres::Solver::Summary summary;
	model_est.solve(options, &summary);
	std::cout << summary.FullReport() << "\n";

	cout << setprecision(5);
	cout << (states_est - states_obs).cwiseAbs().colwise().mean() << endl;
	cout << setprecision(3);
	cout << "model_par_rea: " << c_mass << " | " 		   << c_hor << " | " 			<< c_ver << " | " 			 << c_ang << " | "  	      << ang_err << endl;
	cout << "model_par_est: " << exp(model_c_mass_est) << " | " << model_par_est[0] << " | " << model_par_est[1] << " | " << model_par_est[2] << " | " << model_par_est[3] << endl ;
	
	return 0;
}
