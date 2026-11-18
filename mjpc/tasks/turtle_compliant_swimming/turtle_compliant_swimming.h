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

#ifndef MJPC_TASKS_TURTLE_COMPLIANT_SWIMMING_TURTLE_COMPLIANT_SWIMMING_H_
#define MJPC_TASKS_TURTLE_COMPLIANT_SWIMMING_TURTLE_COMPLIANT_SWIMMING_H_

#include <memory>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>
#include "mjpc/task.h"

namespace mjpc {

// Mechanical power at the output shaft of an actuator [W]: actuator torque
// minus joint damping (back-EMF when the motor model is on), times velocity.
double ShaftPower(const mjModel* model, const mjData* data, int actuator);

// Motor torque-speed envelope for design variants (design_study.py). An actuator with a
// numeric motor_<actuator> = "tau_peak tau_stall w0 eta [km]" (at the motor, before any
// external stage of efficiency eta) gets its position-servo force limited to
//   motoring:  |force| <= eta min(tau_peak, tau_stall (1 - |w| / w0))
//   braking:   |force| <= eta tau_peak
// with w the actuator velocity, by moving ctrl to the value that gives the limiting force
// (MuJoCo cannot clip a position actuator's force by speed). InitMotorEnvelope caches the
// numerics (returns how many actuators have one); ClampToMotorEnvelope is meant to run as
// mjcb_control, so it applies in every planner rollout as well as in the simulation.
int InitMotorEnvelope(const mjModel* model);
void ClampToMotorEnvelope(const mjModel* model, mjData* data);

// Straight, level swimming for the sea-turtle robot with compliant front
// flippers (turtle_simulator's turtle_compliant.xml, default blade stiffness).
//
// All 10 actuators are joint-target servos. The planner starts from the
// turtle_simulator reference gait: the Gait term pulls every actuator towards
// that gait's target at the current stroke phase, and the runner seeds the
// nominal plan with it. Lowering the Gait weight lets the optimizer move away
// from the reference to swim faster, straighter or more efficiently.
//
// Residuals (14 + nu = 24 for nu = 10):
//   [0]          Speed    one-sided: min(v_fwd - v_desired, 0), v_fwd = COM
//                         velocity along the swim direction
//   [1-3]        Slip     COM velocity perpendicular to the swim direction
//                         (sideways and vertical drift)
//   [4-6]        Heading  cross(shell forward axis, swim direction)
//   [7-9]        Upright  cross(shell up axis, world z)
//   [10-12]      AngVel   shell angular velocity (world frame)
//   [13, 13+nu)  Gait     ctrl - reference gait target at the current phase
//   [13+nu]      Power    sqrt(sum of positive shaft power, see ShaftPower);
//                         with the quadratic norm the cost is
//                         0.5 * weight * power [W]
//
// Parameters (residual_* numerics in task.xml, in this order):
//   Speed    desired forward speed [m/s]
//   Heading  swim direction, yaw about world z [rad]; 0 = world -y, which is
//            the shell's forward axis in the home keyframe
//   Cadence  stroke-frequency multiplier on the reference gait (1 = as in
//            turtle_simulator)
//   Phase    offset added to the stroke phase [fraction of a stroke]
class TurtleCompliantSwimming : public Task {
 public:
  std::string Name() const override;
  std::string XmlPath() const override;

  class ResidualFn : public mjpc::BaseResidualFn {
   public:
    explicit ResidualFn(const TurtleCompliantSwimming* task)
        : mjpc::BaseResidualFn(task) {}
    void Residual(const mjModel* model, const mjData* data,
                  double* residual) const override;

    // Reference gait target for every actuator at the given time.
    void ReferenceCtrl(const mjModel* model, double time, double* ctrl) const;

   private:
    friend class TurtleCompliantSwimming;
    // ids, cached in ResetLocked
    int root_body_id_ = -1;
    int gait_period_id_ = -1;
    std::vector<int> gait_numeric_id_;  // per actuator, -1 if missing
  };

  TurtleCompliantSwimming() : residual_(this) {}

  // Reference gait target for every actuator at the given time, using the
  // task's current parameters (cadence, phase).
  void ReferenceCtrl(const mjModel* model, double time, double* ctrl) const {
    residual_.ReferenceCtrl(model, time, ctrl);
  }

 protected:
  void ResetLocked(const mjModel* model) override;
  std::unique_ptr<mjpc::ResidualFn> ResidualLocked() const override {
    return std::make_unique<ResidualFn>(residual_);
  }
  ResidualFn* InternalResidual() override { return &residual_; }

 private:
  ResidualFn residual_;
};

}  // namespace mjpc

#endif  // MJPC_TASKS_TURTLE_COMPLIANT_SWIMMING_TURTLE_COMPLIANT_SWIMMING_H_
