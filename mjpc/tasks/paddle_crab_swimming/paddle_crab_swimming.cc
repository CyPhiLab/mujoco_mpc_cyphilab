// Copyright 2022 DeepMind Technologies Limited

#include "mjpc/tasks/paddle_crab_swimming/paddle_crab_swimming.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>
#include "mjpc/utilities.h"

namespace mjpc {

namespace {

// Rear-biased control costs: front and center legs remain more expensive than
// the rear paddles, but not so expensive that the optimizer prefers to stay
// still. This keeps the task emergent while allowing motion to reappear.
double ControlRegularizationWeight(int actuator_index) {
	constexpr std::array<double, 18> kControlWeights = {
			3.0, 3.0, 3.0,   // R1 (front right)
			1.8, 1.8, 1.8,   // R2 (center right)
			0.7, 0.7, 0.4,   // R3 (back right, distal paddle cheapest)
			3.0, 3.0, 3.0,   // L1 (front left)
			1.8, 1.8, 1.8,   // L2 (center left)
			0.7, 0.7, 0.4    // L3 (back left, distal paddle cheapest)
	};
	if (actuator_index < 0 ||
			actuator_index >= static_cast<int>(kControlWeights.size())) {
		return 1.0;
	}
	return kControlWeights[actuator_index];
}

double ControlRateRegularizationWeight(int actuator_index) {
	constexpr std::array<double, 18> kRateWeights = {
			2.5, 2.5, 2.5,  // R1 (front right)
			1.5, 1.5, 1.5,  // R2 (center right)
			0.8, 0.8, 0.8,  // R3 (rear paddles)
			2.5, 2.5, 2.5,  // L1 (front left)
			1.5, 1.5, 1.5,  // L2 (center left)
			0.8, 0.8, 0.8   // L3 (rear paddles)
	};
	if (actuator_index < 0 ||
			actuator_index >= static_cast<int>(kRateWeights.size())) {
		return 1.0;
	}
	return kRateWeights[actuator_index];
}

}  // namespace

std::string PaddleCrabSwimming::XmlPath() const {
	return GetModelPath("paddle_crab_swimming/task.xml");
}

std::string PaddleCrabSwimming::Name() const { return "PaddleCrabSwimming"; }

PaddleCrabSwimming::PaddleCrabSwimming()
		: residual_(this), initialized_(false) {}

// Residual layout (17 + model->nu = 35 total for 18-control hexapod):
//   [0-2]   Position error (3)
//   [3]     Progress / one-sided desired speed floor (1)
//   [4-6]   Slip velocity perpendicular to target direction (3)
//   [7-9]   Align: front_point direction from base_cog to front_point (3)
//   [10-12] Trim: cross(body_up, env_normal) (3)
//   [13-15] AngVel damping: omega perpendicular to env_normal (3)
//   [16]    Control effort weighted L2 norm (1)
//   [17+i]  ControlRate: sqrt(rate_weight[i]) * (ctrl[i] - prev_ctrl[i])
// Total: 35 residuals (for 18-control hexapod)
void PaddleCrabSwimming::ResidualFn::Residual(const mjModel* model,
																							 const mjData* data,
																							 double* residual) const {
	// Sensor lookups
	double* base_pos          = SensorByName(model, data, "base_pos_task");
	double* front_pos         = SensorByName(model, data, "front_pos_task");
	double* target_pos        = SensorByName(model, data, "target_pos_task");
	double* base_vel_world    = SensorByName(model, data, "base_vel_world_task");
	double* base_angvel_world = SensorByName(model, data, "base_angvel_world_task");

	// Parameters: [0]=DesiredSpeed [1]=SlowRadius [2-4]=LocAxis [5-7]=UpAxis [8-10]=EnvNormal
	double desired_speed_param = parameters_[0];
	double loc_axis_body[3]    = {parameters_[2], parameters_[3], parameters_[4]};
	double body_up_body[3]     = {parameters_[5], parameters_[6], parameters_[7]};
	double env_normal_world[3] = {parameters_[8], parameters_[9], parameters_[10]};

	// Safety: clear all residuals if any input is bad
	auto is_bad = [](double v) { return std::isnan(v) || std::isinf(v); };
	bool bad = false;
	for (int i = 0; i < 11; ++i) if (is_bad(parameters_[i])) bad = true;
	for (int i = 0; i < 3; ++i) {
		if (is_bad(base_pos[i]) || is_bad(target_pos[i]) ||
				is_bad(base_vel_world[i]) || is_bad(base_angvel_world[i])) bad = true;
		if (front_pos && is_bad(front_pos[i])) bad = true;
	}
	if (bad) {
		int n_residuals = 17 + model->nu;
		for (int r = 0; r < n_residuals; ++r) residual[r] = 0.0;
		return;
	}

	const double eps = 1e-8;

	// Normalize locomotion axis (body frame)
	double loc_norm = mju_sqrt(loc_axis_body[0]*loc_axis_body[0] +
														 loc_axis_body[1]*loc_axis_body[1] +
														 loc_axis_body[2]*loc_axis_body[2] + eps);
	loc_axis_body[0] /= loc_norm;
	loc_axis_body[1] /= loc_norm;
	loc_axis_body[2] /= loc_norm;

	// Normalize body-up axis (body frame)
	double up_norm = mju_sqrt(body_up_body[0]*body_up_body[0] +
														body_up_body[1]*body_up_body[1] +
														body_up_body[2]*body_up_body[2] + eps);
	body_up_body[0] /= up_norm;
	body_up_body[1] /= up_norm;
	body_up_body[2] /= up_norm;

	// Normalize environment normal (world frame)
	double env_norm = mju_sqrt(env_normal_world[0]*env_normal_world[0] +
														 env_normal_world[1]*env_normal_world[1] +
														 env_normal_world[2]*env_normal_world[2] + eps);
	env_normal_world[0] /= env_norm;
	env_normal_world[1] /= env_norm;
	env_normal_world[2] /= env_norm;

	// Get rotation matrix of plate body
	int base_id = mj_name2id(model, mjOBJ_BODY, "plate");
	const double* xmat = (base_id >= 0) ? (data->xmat + 9 * base_id) : nullptr;

	// Rotate body-up axis to world frame.
	double body_up_world[3]  = {0, 0, 0};
	if (xmat) {
		mju_mulMatVec(body_up_world,  xmat, body_up_body,  3, 3);
	}

	// Direction to target
	double d[3] = {target_pos[0] - base_pos[0],
								 target_pos[1] - base_pos[1],
								 target_pos[2] - base_pos[2]};
	double dist_3d = mju_sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2] + eps);
	double dir[3]  = {d[0]/dist_3d, d[1]/dist_3d, d[2]/dist_3d};

	// [0-2] Position error
	residual[0] = d[0];
	residual[1] = d[1];
	residual[2] = d[2];

	// [3] Progress: one-sided floor penalizing insufficient forward speed
	double slow_radius = parameters_[1];
	if (is_bad(slow_radius) || slow_radius <= 1e-8) slow_radius = 1e-8;
	double desired_speed = (dist_3d > slow_radius)
													 ? desired_speed_param
													 : desired_speed_param * (dist_3d / slow_radius);
	double v_parallel = base_vel_world[0]*dir[0] +
										base_vel_world[1]*dir[1] +
										base_vel_world[2]*dir[2];
	residual[3] = mju_min(v_parallel - desired_speed, 0.0);

	// [4-6] Slip: velocity component perpendicular to target direction
	double v_par_vec[3] = {v_parallel*dir[0], v_parallel*dir[1], v_parallel*dir[2]};
	residual[4] = base_vel_world[0] - v_par_vec[0];
	residual[5] = base_vel_world[1] - v_par_vec[1];
	residual[6] = base_vel_world[2] - v_par_vec[2];

	// [7-9] Align: front-point-defined body direction only. No fallback axis is
	// used here; if the direction becomes degenerate, Align contributes zero.
	double body_forward_world[3] = {0, 0, 0};
	if (front_pos) {
		double front_delta[3] = {
				front_pos[0] - base_pos[0],
				front_pos[1] - base_pos[1],
				front_pos[2] - base_pos[2]};
		double front_norm = mju_sqrt(front_delta[0]*front_delta[0] +
										 front_delta[1]*front_delta[1] +
										 front_delta[2]*front_delta[2] + eps);
		if (front_norm > 1e-6) {
			body_forward_world[0] = front_delta[0] / front_norm;
			body_forward_world[1] = front_delta[1] / front_norm;
			body_forward_world[2] = front_delta[2] / front_norm;
		}
	}
	residual[7] = body_forward_world[1]*dir[2] - body_forward_world[2]*dir[1];
	residual[8] = body_forward_world[2]*dir[0] - body_forward_world[0]*dir[2];
	residual[9] = body_forward_world[0]*dir[1] - body_forward_world[1]*dir[0];

	// [10-12] Trim: cross(body_up, env_normal) — zero when up aligns with normal
	residual[10] = body_up_world[1]*env_normal_world[2] - body_up_world[2]*env_normal_world[1];
	residual[11] = body_up_world[2]*env_normal_world[0] - body_up_world[0]*env_normal_world[2];
	residual[12] = body_up_world[0]*env_normal_world[1] - body_up_world[1]*env_normal_world[0];

	// [13-15] AngVel damping: angular velocity perpendicular to env_normal
	double omega_par = base_angvel_world[0]*env_normal_world[0] +
										 base_angvel_world[1]*env_normal_world[1] +
										 base_angvel_world[2]*env_normal_world[2];
	residual[13] = base_angvel_world[0] - omega_par*env_normal_world[0];
	residual[14] = base_angvel_world[1] - omega_par*env_normal_world[1];
	residual[15] = base_angvel_world[2] - omega_par*env_normal_world[2];

	// [16] Control effort: morphology-aware weighted L2 norm.
	double effort_sq = 0.0;
	for (int i = 0; i < model->nu; ++i) {
		effort_sq += ControlRegularizationWeight(i) *
							 data->ctrl[i] * data->ctrl[i];
	}
	residual[16] = mju_sqrt(effort_sq);

	// [17+i] ControlRate: morphology-aware weighted control delta.
	const PaddleCrabSwimming* task =
			static_cast<const PaddleCrabSwimming*>(task_);
	if (model->nu == static_cast<int>(task->prev_ctrl_.size())) {
		for (int i = 0; i < model->nu; ++i) {
			residual[17 + i] =
					mju_sqrt(ControlRateRegularizationWeight(i)) *
					(data->ctrl[i] - task->prev_ctrl_[i]);
		}
	} else {
		for (int i = 0; i < model->nu; ++i) {
			residual[17 + i] = 0.0;
		}
	}
}

void PaddleCrabSwimming::TransitionLocked(mjModel* model, mjData* data) {
	// Initialize on first call or if model->nu changed
	if (!initialized_ || filtered_ctrl_.size() != static_cast<size_t>(model->nu)) {
		filtered_ctrl_.resize(model->nu, 0.0);
		prev_ctrl_.resize(model->nu, 0.0);
		for (int i = 0; i < model->nu; ++i) {
			filtered_ctrl_[i] = data->ctrl[i];
			prev_ctrl_[i]     = data->ctrl[i];
		}
		initialized_ = true;
	}

	int smoothing_enabled_id =
			mj_name2id(model, mjOBJ_NUMERIC, "residual_ControlSmoothingEnabled");
	int smoothing_tau_id =
			mj_name2id(model, mjOBJ_NUMERIC, "residual_ControlSmoothingTau");

	double smoothing_enabled = (smoothing_enabled_id >= 0)
			? model->numeric_data[model->numeric_adr[smoothing_enabled_id]] : 1.0;
	double tau = (smoothing_tau_id >= 0)
			? model->numeric_data[model->numeric_adr[smoothing_tau_id]] : 0.08;

	if (smoothing_enabled > 0.5) {
		double dt    = model->opt.timestep;
		double alpha = dt / (tau + dt);
		for (int i = 0; i < model->nu; ++i) {
			prev_ctrl_[i]     = filtered_ctrl_[i];
			filtered_ctrl_[i] = filtered_ctrl_[i] + alpha * (data->ctrl[i] - filtered_ctrl_[i]);
		}
		mju_copy(data->ctrl, filtered_ctrl_.data(), model->nu);
	} else {
		for (int i = 0; i < model->nu; ++i) {
			prev_ctrl_[i]     = filtered_ctrl_[i];
			filtered_ctrl_[i] = data->ctrl[i];
		}
	}
}

}  // namespace mjpc