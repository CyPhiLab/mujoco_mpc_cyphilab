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

#ifndef MJPC_TASKS_CRAB_SWIMMING_CRAB_SWIMMING_H_
#define MJPC_TASKS_CRAB_SWIMMING_CRAB_SWIMMING_H_

#include <string>
#include <mujoco/mujoco.h>
#include "mjpc/task.h"

namespace mjpc {

// CrabSwimming: 3D target-tracking task using a generic body-motion locomotion
// controller for a 12-DOF crab robot swimming in fluid. This is a morphology-agnostic
// controller that can be adapted to other swimming/walking robots by changing the
// configurable body axes and parameters.
//
// Actuator order (ctrl indices, position control):
//   0-1:   Front right leg (mid, distal)
//   2-3:   Middle right leg (mid, distal)
//   4-5:   Back right leg (mid, distal)
//   6-7:   Front left leg (mid, distal)
//   8-9:   Middle left leg (mid, distal)
//   10-11: Back left leg (mid, distal)
//
// Controller: Generic body-motion MPC with 17-residual layout.
// Root body: plate (floating base)
// Target reference: 3D position sensor
class CrabSwimming : public Task {
 public:
  std::string Name() const override;
  std::string XmlPath() const override;

  class ResidualFn : public mjpc::BaseResidualFn {
   public:
    explicit ResidualFn(const CrabSwimming* task)
        : mjpc::BaseResidualFn(task) {}

    void Residual(const mjModel* model, const mjData* data,
                  double* residual) const override;
  };

  CrabSwimming();
  void TransitionLocked(mjModel* model, mjData* data) override;

 protected:
  std::unique_ptr<mjpc::ResidualFn> ResidualLocked() const override {
    return std::make_unique<ResidualFn>(this);
  }
  ResidualFn* InternalResidual() override { return &residual_; }

 private:
  ResidualFn residual_;
  std::vector<double> filtered_ctrl_;  // First-order filtered control commands
  bool initialized_;                    // Flag to track first initialization
};

}  // namespace mjpc

#endif  // MJPC_TASKS_CRAB_SWIMMING_CRAB_SWIMMING_H_
