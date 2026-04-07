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

#include "mjpc/tasks/pterosaur/pterosaur.h"

#include <cmath>
#include <string>

#include <mujoco/mujoco.h>
#include "mjpc/task.h"
#include "mjpc/utilities.h"

namespace mjpc {

std::string Pterosaur::XmlPath() const {
  return GetModelPath("pterosaur/task.xml");
}
std::string Pterosaur::Name() const { return "Pterosaur"; }

// ------- Residuals for Pterosaur jump task ------
//     Velocity: COM velocity should match desired launch velocity vector
//     Effort: Minimize control effort
// ------------------------------------------
void Pterosaur::ResidualFn::Residual(const mjModel* model, const mjData* data,
                                     double* residual) const {
  // parameters: Launch Speed (m/s) and Launch Angle (radians)
  double speed = - parameters_[0];
  double angle = parameters_[1];

  // target launch velocity: +X horizontal, +Z vertical
  double target_vel[3] = {speed * mju_cos(angle), 0.0, speed * mju_sin(angle)};

  // ----- residual (0-2): COM velocity error ----- //
  double* com_vel = SensorByName(model, data, "com_velocity");
  mju_sub(residual, com_vel, target_vel, 3);

  // ----- residual (3-16): control effort ----- //
  mju_copy(&residual[3], data->ctrl, model->nu);
}

}  // namespace mjpc
