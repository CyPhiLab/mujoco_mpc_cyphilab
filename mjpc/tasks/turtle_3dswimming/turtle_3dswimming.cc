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

std::string Turtle3DSwimming::Name() const { return "Turtle3DSwimming"; }

// Residuals for Turtle3DSwimming task.
//   Number of residuals: 17
//     Residual (0-9):   control effort for 10 actuators
//     Residual (10):    3D distance to target
//     Residual (11):    distance-adaptive progress residual (smooth tanh, no clamp)
//     Residual (12-14): cross(body_axis, head_to_target_direction)
//     Residual (15):    weak roll/bank trim stabilizer
//     Residual (16):    reserved / disabled
//
// Actuator ctrl index mapping:
//   0=shoulderL1, 1=shoulderL2, 2=shoulderL3 (left front chain)
//   3=shoulderR1, 4=shoulderR2, 5=shoulderR3 (right front chain)
//   6=hip_L1, 7=hip_L2 (left rear; hip_L2 is rear flipper -> 70% cheaper)
//   8=hip_R1, 9=hip_R2 (right rear; hip_R2 is rear flipper -> 70% cheaper)
void Turtle3DSwimming::ResidualFn::Residual(const mjModel* model,
                                            const mjData* data,
                                            double* residual) const {
  // Residuals [0-9]: control effort. Cheap-joint scaling unchanged.
  mju_copy(residual, data->ctrl, model->nu);
  residual[2] *= 0.5;   // shoulderL3: left  front flipper stroke, 50% cheaper
  residual[5] *= 0.5;   // shoulderR3: right front flipper stroke, 50% cheaper
  residual[7] *= 0.33;  // hip_L2:     left  rear  flipper stroke, 70% cheaper
  residual[9] *= 0.33;  // hip_R2:     right rear  flipper stroke, 70% cheaper

  // Fetch sensor values by name (defined in task.xml).
  double* head_pos       = SensorByName(model, data, "head_pos_task");
  double* tail_pos       = SensorByName(model, data, "tail_pos_task");
  double* target_pos     = SensorByName(model, data, "target_pos_task");
  double* base_pos       = SensorByName(model, data, "base_pos_task");
  double* base_vel_world = SensorByName(model, data, "base_vel_world_task");

  // --- 3D body forward axis (tail -> head), normalized ---
  double body_axis_x = head_pos[0] - tail_pos[0];
  double body_axis_y = head_pos[1] - tail_pos[1];
  double body_axis_z = head_pos[2] - tail_pos[2];
  double body_length = mju_sqrt(body_axis_x * body_axis_x +
                                body_axis_y * body_axis_y +
                                body_axis_z * body_axis_z + 1e-6);
  body_axis_x /= body_length;
  body_axis_y /= body_length;
  body_axis_z /= body_length;

  // --- 3D distance from base to target (for dist residual and speed scaling) ---
  double dx = target_pos[0] - base_pos[0];
  double dy = target_pos[1] - base_pos[1];
  double dz = target_pos[2] - base_pos[2];
  double dist_3d = mju_sqrt(dx * dx + dy * dy + dz * dz + 1e-6);

  // --- Head-to-target direction (for alignment and progress) ---
  double hx = target_pos[0] - head_pos[0];
  double hy = target_pos[1] - head_pos[1];
  double hz = target_pos[2] - head_pos[2];
  double hdist = mju_sqrt(hx * hx + hy * hy + hz * hz + 1e-6);
  double look_x = hx / hdist;
  double look_y = hy / hdist;
  double look_z = hz / hdist;

  // Residual [10]: full 3D distance from base to target.
  residual[10] = dist_3d;

  // Residual [11]: distance-adaptive progress residual.
  // desired_speed scales smoothly with distance via tanh - fast when far, near-zero at goal.
  // Two-sided (no clamp): rewards going faster than desired and penalizes going slower.
  const double vmax = 0.55;
  const double d0   = 0.6;
  double desired_speed = vmax * mju_tanh(dist_3d / d0);
  double speed_toward  = base_vel_world[0] * look_x +
                         base_vel_world[1] * look_y +
                         base_vel_world[2] * look_z;
  residual[11] = desired_speed - speed_toward;

  // Residuals [12-14]: cross(body_axis, head_to_target_direction).
  // Zero when body is perfectly aligned with head-to-target vector.
  residual[12] = body_axis_y * look_z - body_axis_z * look_y;
  residual[13] = body_axis_z * look_x - body_axis_x * look_z;
  residual[14] = body_axis_x * look_y - body_axis_y * look_x;

  // Residual [15]: weak roll/bank trim stabilizer.
  // Uses the body's world-frame rotation matrix (xmat, row-major 3x3).
  // body right axis in world = column 0: (xmat[0], xmat[3], xmat[6]).
  // dot(body_right, world_up) is ~0 when not rolled, +/-1 when fully rolled.
  int base_id = mj_name2id(model, mjOBJ_BODY, "fr13_s105k");
  if (base_id >= 0) {
    const double* xmat = data->xmat + 9 * base_id;
    residual[15] = xmat[6];  // Z-component of body right axis
  } else {
    residual[15] = 0.0;
  }

  // Residual [16]: reserved / disabled.
  residual[16] = 0.0;
}

void Turtle3DSwimming::TransitionLocked(mjModel* model, mjData* data) {
  // Static target; can be extended later for moving targets.
}

}  // namespace mjpc