#include "drone_model.hpp"

#include <math.h>

Drone_model::Drone_model(/* args */)
{
}

Drone_model::~Drone_model()
{
}


state_t Drone_model::state_diff_eq(state_t curr_state, input_t input)
{
	// state derivatives based on curr state and input
	state_t state_diff;

	state_diff.x = cos(curr_state.a)*input.pitch - sin(curr_state.a)*input.roll;
	state_diff.y = sin(curr_state.a)*input.pitch + cos(curr_state.a)*input.roll;
	state_diff.z = input.throttle;
	state_diff.a = input.yaw;

	state_diff.roll = Drone_model::roll_pitch_par*(input.roll - curr_state.roll);
	state_diff.pitch = Drone_model::roll_pitch_par*(input.pitch - curr_state.pitch);
	state_diff.throttle = Drone_model::throttle_par*(input.throttle - curr_state.throttle);
	state_diff.yaw = Drone_model::yaw_par*(input.yaw - curr_state.yaw);

	return state_diff;
}