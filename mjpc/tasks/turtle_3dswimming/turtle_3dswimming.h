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

#ifndef MJPC_TASKS_TURTLE_3DSWIMMING_TURTLE_3DSWIMMING_H_
#define MJPC_TASKS_TURTLE_3DSWIMMING_TURTLE_3DSWIMMING_H_

#include <string>
#include <mujoco/mujoco.h>
#include "mjpc/task.h"

namespace mjpc {

// Turtle3DSwimming: 3D target-tracking task using the real physical turtle robot
// model (from Onshape CAD export) with velocity-controlled actuators.
//
// Actuator order (ctrl indices):
//   0: shoulderL1  (left  front arm sweep)
//   1: shoulderL2  (left  front arm pitch)
//   2: shoulderL3  (left  front flipper)
//   3: shoulderR1  (right front arm sweep)
//   4: shoulderR2  (right front arm pitch)
//   5: shoulderR3  (right front flipper)
//   6: hip_L1      (left  rear  arm)
//   7: hip_L2      (left  rear  flipper)  <- cheaper cost
//   8: hip_R1      (right rear  arm)
//   9: hip_R2      (right rear  flipper)  <- cheaper cost
class Turtle3DSwimming : public Task {
 public:
  std::string Name() const override;
  std::string XmlPath() const override;

  class ResidualFn : public mjpc::BaseResidualFn {
   public:
    explicit ResidualFn(const Turtle3DSwimming* task)
        : mjpc::BaseResidualFn(task) {}

    void Residual(const mjModel* model, const mjData* data,
                  double* residual) const override;
  };

  Turtle3DSwimming() : residual_(this) {}
  void TransitionLocked(mjModel* model, mjData* data) override;

 protected:
  std::unique_ptr<mjpc::ResidualFn> ResidualLocked() const override {
    return std::make_unique<ResidualFn>(this);
  }
  ResidualFn* InternalResidual() override { return &residual_; }

 private:
  ResidualFn residual_;
};

}  // namespace mjpc

#endif  // MJPC_TASKS_TURTLE_3DSWIMMING_TURTLE_3DSWIMMING_H_