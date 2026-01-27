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
//     Ring Enclosure: Ring min-distance wrapping around cube
//     Tendon Effort: Minimize tendon forces  
//     Cube Height: Reward lifting the cube up
//     Cube XY: Keep cube centered
//     Joint Velocity: Smooth motion penalty
// ------------------------------------------
namespace {


void ResidualImpl(const mjModel* model, const mjData* data, double* residual) {
  // ----- residual (0): tip-to-cube distance ----- //
  double* cube_pos = SensorByName(model, data, "cube_position");
  double* tip_pos = SensorByName(model, data, "tip_position");
  
  double dx = tip_pos[0] - cube_pos[0];
  double dy = tip_pos[1] - cube_pos[1]; 
  double dz = tip_pos[2] - cube_pos[2];
  residual[0] = std::sqrt(dx * dx + dy * dy + dz * dz);
  
  // ----- residual (1): segment-to-cube distance penalty ----- //
  // Simple approach: penalize distances from robot segments to cube
  const int sample_step = 2; // Sample every 2nd site for performance
  double total_distance = 0.0;
  int count = 0;
  
  // Iterate through robot sites, sampling for efficiency
  for (int site_idx = 0; site_idx < model->nsite; site_idx += sample_step) {
    const double* site_pos = &data->site_xpos[3 * site_idx];
    
    double dx = site_pos[0] - cube_pos[0];
    double dy = site_pos[1] - cube_pos[1]; 
    double dz = site_pos[2] - cube_pos[2];
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    
    // Accumulate inverse exponential to reward proximity
    total_distance += -dist; // threshold for "close"
    count++;
  }
  
  // Average and negate (we want to maximize proximity by minimizing residual)
  residual[1] = -(total_distance / count);

  // ----- residual (2): tendon effort ----- //
  double tendon_effort = 0.0;
  for (int i = 0; i < model->nu; i++) {
    tendon_effort += data->ctrl[i] * data->ctrl[i];
  }
  residual[2] = std::sqrt(tendon_effort);

  // ----- residual (3): cube height (negative to reward lifting) ----- //
  residual[3] = -(cube_pos[2] - 0.2); // Reward lifting above initial height

  // ----- residual (4-5): cube x,y position (keep close to 0,0) ----- //
  residual[4] = cube_pos[0]; // x position
  residual[5] = cube_pos[1]; // y position

  // ----- residual (6): joint velocity penalty ----- //
  double joint_vel_penalty = 0.0;
  for (int i = 0; i < model->nv; i++) {
    joint_vel_penalty += data->qvel[i] * data->qvel[i];
  }
  residual[6] = std::sqrt(joint_vel_penalty);
}
}  // namespace

void Spirob::ResidualFn::Residual(const mjModel* model, const mjData* data,
                                 double* residual) const {
  ResidualImpl(model, data, residual);
}

}  // namespace mjpc