// Copyright 2026 CyPhiLab
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

#include "mjpc/tasks/turtle_compliant_swimming/turtle_compliant_swimming.h"

#include <cmath>
#include <string>

#include <mujoco/mujoco.h>
#include "mjpc/task.h"
#include "mjpc/utilities.h"

namespace mjpc {

namespace {
constexpr char kRootBody[] = "fr13_s105k";
constexpr char kGaitPeriod[] = "turtle_gait_period";
constexpr char kGaitPrefix[] = "turtle_gait_";

// parameter indices (order of residual_* numerics in task.xml)
constexpr int kParamSpeed = 0;
constexpr int kParamHeading = 1;
constexpr int kParamCadence = 2;
constexpr int kParamPhase = 3;
}  // namespace

std::string TurtleCompliantSwimming::XmlPath() const {
  return GetModelPath("turtle_compliant_swimming/task.xml");
}
std::string TurtleCompliantSwimming::Name() const {
  return "TurtleCompliantSwimming";
}

void TurtleCompliantSwimming::ResetLocked(const mjModel* model) {
  residual_.root_body_id_ = mj_name2id(model, mjOBJ_BODY, kRootBody);
  if (residual_.root_body_id_ < 0) {
    mju_error("TurtleCompliantSwimming: body '%s' not found", kRootBody);
  }
  residual_.gait_period_id_ = mj_name2id(model, mjOBJ_NUMERIC, kGaitPeriod);
  residual_.gait_numeric_id_.assign(model->nu, -1);
  for (int i = 0; i < model->nu; i++) {
    const char* name = mj_id2name(model, mjOBJ_ACTUATOR, i);
    if (!name) continue;
    std::string numeric = std::string(kGaitPrefix) + name;
    residual_.gait_numeric_id_[i] =
        mj_name2id(model, mjOBJ_NUMERIC, numeric.c_str());
  }
}

void TurtleCompliantSwimming::ResidualFn::ReferenceCtrl(const mjModel* model,
                                                        double time,
                                                        double* ctrl) const {
  double period = 1.0;
  if (gait_period_id_ >= 0) {
    period = model->numeric_data[model->numeric_adr[gait_period_id_]];
  }
  double cadence = parameters_[kParamCadence];
  if (!(cadence > 1e-3)) cadence = 1e-3;
  double phase = time * cadence / period + parameters_[kParamPhase];
  phase -= std::floor(phase);  // [0, 1)

  for (int i = 0; i < model->nu; i++) {
    int id = (i < static_cast<int>(gait_numeric_id_.size()))
                 ? gait_numeric_id_[i] : -1;
    if (id < 0) {
      ctrl[i] = 0.0;  // no reference for this actuator: hold zero
      continue;
    }
    const double* table = model->numeric_data + model->numeric_adr[id];
    int n = model->numeric_size[id];
    // periodic linear interpolation, sample k at phase k / n
    double x = phase * n;
    int k = static_cast<int>(x);
    double f = x - k;
    k %= n;
    ctrl[i] = (1.0 - f) * table[k] + f * table[(k + 1) % n];
  }
}

void TurtleCompliantSwimming::ResidualFn::Residual(const mjModel* model,
                                                   const mjData* data,
                                                   double* residual) const {
  int counter = 0;

  // swim direction: world -y rotated about z by Heading
  double heading = parameters_[kParamHeading];
  double dir[3] = {std::sin(heading), -std::cos(heading), 0.0};

  // shell axes in world frame (body -y is forward, +z is up)
  const double* xmat = data->xmat + 9 * root_body_id_;
  double forward[3] = {-xmat[1], -xmat[4], -xmat[7]};
  double up[3] = {xmat[2], xmat[5], xmat[8]};

  // ---------- Speed ----------
  double* com_vel = SensorByName(model, data, "com_linvel");
  double v_fwd = mju_dot3(com_vel, dir);
  residual[counter++] = mju_min(v_fwd - parameters_[kParamSpeed], 0.0);

  // ---------- Slip ----------
  for (int i = 0; i < 3; i++) {
    residual[counter++] = com_vel[i] - v_fwd * dir[i];
  }

  // ---------- Heading ----------
  mju_cross(residual + counter, forward, dir);
  counter += 3;

  // ---------- Upright ----------
  double world_z[3] = {0.0, 0.0, 1.0};
  mju_cross(residual + counter, up, world_z);
  counter += 3;

  // ---------- AngVel ----------
  double* angvel = SensorByName(model, data, "shell_angvel");
  mju_copy3(residual + counter, angvel);
  counter += 3;

  // ---------- Gait ----------
  ReferenceCtrl(model, data->time, residual + counter);
  for (int i = 0; i < model->nu; i++) {
    residual[counter + i] = data->ctrl[i] - residual[counter + i];
  }
  counter += model->nu;

  // ---------- Power ----------
  double power = 0.0;
  for (int i = 0; i < model->nu; i++) {
    power += mju_max(data->actuator_force[i] * data->actuator_velocity[i], 0.0);
  }
  residual[counter++] = std::sqrt(power);

  CheckSensorDim(model, counter);
}

}  // namespace mjpc
