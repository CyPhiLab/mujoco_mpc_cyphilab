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

#include "mjpc/tasks/spirob/spirob.h"

#include <cmath>
#include <string>

#include <mujoco/mujoco.h>
#include "mjpc/task.h"
#include "mjpc/utilities.h"

namespace mjpc {
std::string Spirob::XmlPath() const {
  return GetModelPath("spirob/task.xml");
}
std::string Spirob::Name() const { return "SpiRob"; }

// ------- Residuals for SpiRob task ------
//     Tip Position: End effector should reach target position
//     Tendon Effort: Minimize tendon forces
//     Tendon Limits: Keep tendon positions within limits
//     Velocity: Minimize joint velocities
// ------------------------------------------
namespace {
void ResidualImpl(const mjModel* model, const mjData* data,
                  const double target[3], double* residual) {
  // ----- residual (0-2): tip position error ----- //
  double* tip_pos = SensorByName(model, data, "tip_position");
  mju_sub(residual, tip_pos, target, 3);

  // ----- residual (3): tendon effort ----- //
  double tendon_effort = 0.0;
  for (int i = 0; i < model->nu; i++) {
    tendon_effort += data->ctrl[i] * data->ctrl[i];
  }
  residual[3] = std::sqrt(tendon_effort);

  // ----- residual (4): tendon limits ----- //
  double tendon_limit_violation = 0.0;
  for (int i = 0; i < model->ntendon; i++) {
    double length = data->ten_length[i];
    if (model->tendon_range) {
      double range_low = model->tendon_range[2*i];
      double range_high = model->tendon_range[2*i + 1];
      
      if (length < range_low) {
        tendon_limit_violation += (range_low - length) * (range_low - length);
      } else if (length > range_high) {
        tendon_limit_violation += (length - range_high) * (length - range_high);
      }
    }
  }
  residual[4] = std::sqrt(tendon_limit_violation);

  // ----- residual (5): joint velocity ----- //
  double vel_magnitude = 0.0;
  for (int i = 0; i < model->nv; i++) {
    vel_magnitude += data->qvel[i] * data->qvel[i];
  }
  residual[5] = std::sqrt(vel_magnitude);
}
}  // namespace

void Spirob::ResidualFn::Residual(const mjModel* model, const mjData* data,
                                 double* residual) const {
  // Fixed target position (for now - can be made dynamic later)
  double target[3]{0.3, 0.0, 0.2};
  ResidualImpl(model, data, target, residual);
}

}  // namespace mjpc