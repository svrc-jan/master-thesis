# Identification and control of robotic helicopter
by Jan Švrčina

This is a source-code repository for the master thesis project on the Faculty of Electrical Engineering at CTU Prague.

## Outline

This project is used for the identification and control of the robotic helicopter Tello using the motion tracking Vicon system. To control we are using the moving horizon estimate (MHE) and model predictive control (MPC), identification is a MAP estimate using trajectories. The code written in C++ is separeted to parts of the `src` directory:

 - `filter`: filtering algorithm for the Vicon system
 - `model`: model of the dynamic system
 - `optim`: identification method, MHE, MPC using Ceres library
 - `tello`: communication with the Tello helicopter
 - `utils`: logger, parser and auxillary methods
 - `vicon`: communication with the Vicon system

## Building the program
Commands to build the programs using the CMake tools:
```
mkdir build
cmake .
make run_model_ident
make mpc_control
```

## Running the program
### Model identification
The `run_model_ident` has one argument, the path to the configuration file, example:

```
build/run_model_ident config/ident_real
```


### MPC control
The `mpc_control` has two arguments, name of the log and target configuration file, example:

```
build/mpc_control square config/tar_square
```

The controls are using keyboard (program requires sudo permissions to access the device):
 - `Q`: quit
 - `T`: take-off
 - `R`: land
 - `E`: manual control
 - `C`: automatic control

During manual control, the commands are (positive - negative):
 - `A`-`D`: roll
 - `W`-`S`: pitch
 - `J`-`L`: yaw
 - `I`-`K`: throttle

## Configuration file
The model identification, MHE, MPC and the control program require JSON configuration files saved in the `config` folder.

### Identification configuration options
- `dt`: discretization step
- `u_delay`: input delay
- `C_o`: weighing coefficients for observations (size number of observations)
- `C_s`: weighing coefficients for state transitions (size number of states)
- `C_p`: weighing coefficients for parameter priors (size number of parameters)
- `p_prior`: prior values for parameters (size number of parameters)
- `p_lb`: lower bound values for parameters (size number of parameters)
- `p_ub`: upper bound values for parameters (size number of parameters)
- `obs_loss_s`: 1 to use Tukey loss for observations, 0 for normal loss  
- `state_loss_s`: 1 to use Tukey loss for state transitions, 0 for normal loss
- `max_models`: maximum number of logs to load
- `log_dir`: folder from where to load the trajectory logs
- `clear_log_est_dir`: boolean, if true delete contents of the estimation folder
- `solver_tol`: solver relative functional tolerance
- `solver_threads`: number of threads to use for optimization
- `solver_linear_solver_type`: what factorization the solver uses (example \texttt{sparse_cholesky})
- `solver_stdout`: boolean, if true, solver prints optimization progress


### MHE configuration options
- `dt`: discretization step
- `u_delay`: input delay
- `C_o`: weighing coefficients for observations (size number of observations)
- `C_s`: weighing coefficients for state transitions (size number of states)
- `C_p`: weighing coefficients for parameter priors (size number of parameters)
- `p_prior`: prior values for parameters (size number of parameters)
- `p_lb`: lower bound values for parameters (size number of parameters)
- `p_ub`: upper bound values for parameters (size number of parameters)
- `obs_loss_s`: 1 to use Tukey loss for observations, 0 for normal loss  
- `state_loss_s`: 1 to use Tukey loss for state transitions, 0 for normal loss
- `max_models`: maximum number of logs to load
- `clear_log_est_dir`: boolean, if true delete contents of the estimation folder
- `h` length of the horizon
- `solver_max_time` maximum time for solving in seconds
- `solver_tol`: solver relative functional tolerance
- `solver_threads`: number of threads to use for optimization
- `solver_linear_solver_type`: what factorization the solver uses (example `sparse_cholesky`)
- `solver_stdout`: boolean, if true, solver prints optimization progress

### MPC configuration options
- `input_c`: constant for manual control
- `u_delay`: input delay $D$
- `filter_horizontal_threshold`: horizontal threshold for the Vicon filter
- `filter_vertical_threshold`: horizontal threshold for the Vicon filter
- `filter_angle_threshold`: angular threshold for the Vicon filter
- `keyboard_device`: device for user input
- `vicon_ip`: IP address for Vicon communication
- `tello_port`: port for Tello communication
- `tello_net_interface`: network interface string for  Tello communication
- `mpc_config`: path to the MPC config file
- `mhe_config`: path to the MHE config file
- `log_dir`: path to the directory where the logs will be saved

### Control program configuration options
- `input_c` constant for manual control
- `u_delay` input delay $D$
- `filter_horizontal_threshold` horizontal threshold for the Vicon filter
- `filter_vertical_threshold` horizontal threshold for the Vicon filter
- `filter_angle_threshold` angular threshold for the Vicon filter
- `keyboard_device` device for user input
- `vicon_ip` IP address for Vicon communication
- `tello_port` port for Tello communication
- `tello_net_interface` network interface string for  Tello communication
- `mpc_config` path to the MPC config file
- `mhe_config` path to the MHE config file
- `log_dir` path to the directory where the logs will be saved