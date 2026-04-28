// Copyright 2022 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mjpc/tasks/turtle_3dswimming/turtle_3dswimming.h"

#include <string>

#include <mujoco/mujoco.h>
#include "mjpc/utilities.h"

namespace mjpc {

std::string Turtle3DSwimming::XmlPath() const {
  return GetModelPath("turtle_3dswimming/task.xml");
}

std::string Turtle3DSwimming::Name() const { return "TurtleSwimming"; }

Turtle3DSwimming::Turtle3DSwimming() 
    : residual_(this), initialized_(false) {}

// Residuals for Turtle3DSwimming task.
//   Number of residuals: 17
//     Residual (0-2):    3D position error
//     Residual (3):      one-sided forward progress floor
//     Residual (4-6):    cross-track / slip velocity vector
//     Residual (7-9):    locomotion-axis alignment vector
//     Residual (10-12):  trim relative to environment normal
//     Residual (13-15):  angular-rate damping orthogonal to environment normal
//     Residual (16):     scalar control effort
void Turtle3DSwimming::ResidualFn::Residual(const mjModel* model,
                                            const mjData* data,
                                            double* residual) const {

  // Fetch sensor values by name
  double* head_pos       = SensorByName(model, data, "head_pos_task");
  double* target_pos     = SensorByName(model, data, "target_pos_task");
  double* base_vel_world = SensorByName(model, data, "base_vel_world_task");
  double* base_angvel_world = SensorByName(model, data, "base_angvel_world_task");

  // Read task parameters from parameters_ array.
  // Parameter layout: DesiredSpeed (1), SlowRadius (1), LocomotionAxisBody (3),
  // BodyUpAxisBody (3), EnvNormalWorld (3) = 11 total.
  double desired_speed_param = parameters_[0];

  double loc_axis_body[3] = {parameters_[2], parameters_[3], parameters_[4]};
  double body_up_body[3] = {parameters_[5], parameters_[6], parameters_[7]};
  double env_normal_world[3] = {parameters_[8], parameters_[9], parameters_[10]};

  // NaN/Inf check helper
  auto is_bad = [](double v) { return std::isnan(v) || std::isinf(v); };
  bool bad_param = false;
  for (int i = 0; i < 11; ++i) if (is_bad(parameters_[i])) bad_param = true;
  bool bad_sensor = false;
  for (int i = 0; i < 3; ++i) {
    if (is_bad(head_pos[i]) || is_bad(target_pos[i]) || is_bad(base_vel_world[i]) || is_bad(base_angvel_world[i])) bad_sensor = true;
  }
  if (bad_param || bad_sensor) {
    mju_warning("Turtle3DSwimming: NaN/Inf in parameters or sensors! param=%d sensor=%d", bad_param, bad_sensor);
  }

  // Safe-guard for slow_radius. Do not mutate parameters_ inside const Residual().
  double slow_radius = parameters_[1];
  if (is_bad(slow_radius) || slow_radius <= 1e-8) {
    mju_warning("Turtle3DSwimming: slow_radius invalid (%.6f), clamping to 1e-8", slow_radius);
    slow_radius = 1e-8;
  }

  // Normalize body-frame axes
  const double eps = 1e-8;
  double loc_norm = mju_sqrt(loc_axis_body[0] * loc_axis_body[0] +
                              loc_axis_body[1] * loc_axis_body[1] +
                              loc_axis_body[2] * loc_axis_body[2] + eps);
  loc_axis_body[0] /= loc_norm;
  loc_axis_body[1] /= loc_norm;
  loc_axis_body[2] /= loc_norm;

  double up_norm = mju_sqrt(body_up_body[0] * body_up_body[0] +
                             body_up_body[1] * body_up_body[1] +
                             body_up_body[2] * body_up_body[2] + eps);
  body_up_body[0] /= up_norm;
  body_up_body[1] /= up_norm;
  body_up_body[2] /= up_norm;

  // Normalize environment normal
  double env_norm = mju_sqrt(env_normal_world[0] * env_normal_world[0] +
                              env_normal_world[1] * env_normal_world[1] +
                              env_normal_world[2] * env_normal_world[2] + eps);
  env_normal_world[0] /= env_norm;
  env_normal_world[1] /= env_norm;
  env_normal_world[2] /= env_norm;

  // Get root body orientation matrix (body-to-world rotation)
  int base_id = mj_name2id(model, mjOBJ_BODY, "fr13_s105k");
  const double* xmat = (base_id >= 0) ? (data->xmat + 9 * base_id) : nullptr;

  // Transform body-frame axes to world frame
  // xmat is row-major: column i is at indices [i, 3+i, 6+i]
  double loc_axis_world[3] = {0, 0, 0};
  double body_up_world[3] = {0, 0, 0};
  
  if (xmat) {
    // loc_axis_world = R_body_to_world * loc_axis_body
    loc_axis_world[0] = xmat[0] * loc_axis_body[0] + xmat[3] * loc_axis_body[1] + xmat[6] * loc_axis_body[2];
    loc_axis_world[1] = xmat[1] * loc_axis_body[0] + xmat[4] * loc_axis_body[1] + xmat[7] * loc_axis_body[2];
    loc_axis_world[2] = xmat[2] * loc_axis_body[0] + xmat[5] * loc_axis_body[1] + xmat[8] * loc_axis_body[2];

    // body_up_world = R_body_to_world * body_up_body
    body_up_world[0] = xmat[0] * body_up_body[0] + xmat[3] * body_up_body[1] + xmat[6] * body_up_body[2];
    body_up_world[1] = xmat[1] * body_up_body[0] + xmat[4] * body_up_body[1] + xmat[7] * body_up_body[2];
    body_up_world[2] = xmat[2] * body_up_body[0] + xmat[5] * body_up_body[1] + xmat[8] * body_up_body[2];
  }

  // Compute target displacement and direction
  double d[3] = {target_pos[0] - head_pos[0],
                 target_pos[1] - head_pos[1],
                 target_pos[2] - head_pos[2]};
  double dist_3d = mju_sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2] + eps);
  double dir[3] = {d[0] / dist_3d, d[1] / dist_3d, d[2] / dist_3d};

  // ============================================================
  // Residual [0-2]: 3D position error
  // ============================================================
  residual[0] = d[0];
  residual[1] = d[1];
  residual[2] = d[2];

  // ============================================================
  // Residual [3]: one-sided forward progress floor
  // ============================================================
  double desired_speed = (dist_3d > slow_radius)
                            ? desired_speed_param
                            : desired_speed_param * (dist_3d / slow_radius);
  double v_parallel = base_vel_world[0] * dir[0] +
                      base_vel_world[1] * dir[1] +
                      base_vel_world[2] * dir[2];
  residual[3] = mju_min(v_parallel - desired_speed, 0.0);

  // ============================================================
  // Residual [4-6]: cross-track / slip velocity vector
  // ============================================================
  double v_parallel_vec[3] = {
      v_parallel * dir[0],
      v_parallel * dir[1],
      v_parallel * dir[2]
  };
  double v_perp[3] = {
      base_vel_world[0] - v_parallel_vec[0],
      base_vel_world[1] - v_parallel_vec[1],
      base_vel_world[2] - v_parallel_vec[2]
  };
  residual[4] = v_perp[0];
  residual[5] = v_perp[1];
  residual[6] = v_perp[2];

  // ============================================================
  // Residual [7-9]: locomotion-axis alignment vector
  // ============================================================
  // cross(loc_axis_world, dir)
    residual[7] = loc_axis_world[1] * dir[2] - loc_axis_world[2] * dir[1];
    residual[8] = loc_axis_world[2] * dir[0] - loc_axis_world[0] * dir[2];
    residual[9] = loc_axis_world[0] * dir[1] - loc_axis_world[1] * dir[0];

  // ============================================================
  // Residual [10-12]: trim relative to environment normal
  // ============================================================
  // cross(body_up_world, env_normal_world)
  residual[10] = body_up_world[1] * env_normal_world[2] -
                 body_up_world[2] * env_normal_world[1];
  residual[11] = body_up_world[2] * env_normal_world[0] -
                 body_up_world[0] * env_normal_world[2];
  residual[12] = body_up_world[0] * env_normal_world[1] -
                 body_up_world[1] * env_normal_world[0];

  // ============================================================
  // Residual [13-15]: angular-rate damping orthogonal to environment normal
  // ============================================================
  double omega_parallel = base_angvel_world[0] * env_normal_world[0] +
                          base_angvel_world[1] * env_normal_world[1] +
                          base_angvel_world[2] * env_normal_world[2];
  double omega_parallel_vec[3] = {
      omega_parallel * env_normal_world[0],
      omega_parallel * env_normal_world[1],
      omega_parallel * env_normal_world[2]
  };
  double omega_perp[3] = {
      base_angvel_world[0] - omega_parallel_vec[0],
      base_angvel_world[1] - omega_parallel_vec[1],
      base_angvel_world[2] - omega_parallel_vec[2]
  };
  residual[13] = omega_perp[0];
  residual[14] = omega_perp[1];
  residual[15] = omega_perp[2];

  // ============================================================
  // Residual [16]: scalar control effort
  // ============================================================
  double effort_sq = 0.0;
  for (int i = 0; i < model->nu; ++i) {
    effort_sq += data->ctrl[i] * data->ctrl[i];
  }
  residual[16] = mju_sqrt(effort_sq);
}

void Turtle3DSwimming::TransitionLocked(mjModel* model, mjData* data) {
  // Initialize filtered control vector on first call or if actuator count changes
  if (!initialized_ || filtered_ctrl_.size() != model->nu) {
    filtered_ctrl_.resize(model->nu, 0.0);
    for (int i = 0; i < model->nu; ++i) {
      filtered_ctrl_[i] = data->ctrl[i];
    }
    initialized_ = true;
  }

  // Read smoothing parameters from task.xml custom numerics
  // residual_ControlSmoothingEnabled: 1 = enabled, 0 = disabled
  // residual_ControlSmoothingTau: time constant in seconds
  int smoothing_enabled_id = 
      mj_name2id(model, mjOBJ_NUMERIC, "residual_ControlSmoothingEnabled");
  int smoothing_tau_id = 
      mj_name2id(model, mjOBJ_NUMERIC, "residual_ControlSmoothingTau");

  double smoothing_enabled = (smoothing_enabled_id >= 0) 
      ? model->numeric_data[smoothing_enabled_id] : 1.0;
  double tau = (smoothing_tau_id >= 0) 
      ? model->numeric_data[smoothing_tau_id] : 0.08;

  // Apply first-order control smoothing
  if (smoothing_enabled > 0.5) {
    double dt = model->opt.timestep;
    // First-order low-pass: alpha = dt / (tau + dt)
    double alpha = dt / (tau + dt);
    
    for (int i = 0; i < model->nu; ++i) {
      // filtered_ctrl[i] += alpha * (u_cmd - filtered_ctrl[i])
      filtered_ctrl_[i] = filtered_ctrl_[i] + alpha * (data->ctrl[i] - filtered_ctrl_[i]);
    }
    
    // Apply filtered controls back to data->ctrl
    mju_copy(data->ctrl, filtered_ctrl_.data(), model->nu);
  } else {
    // Smoothing disabled: pass through and keep filtered state updated
    for (int i = 0; i < model->nu; ++i) {
      filtered_ctrl_[i] = data->ctrl[i];
    }
  }
}

}  // namespace mjpc
