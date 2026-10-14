// Copyright 2022 DeepMind Technologies Limited

#ifndef MJPC_TASKS_PADDLE_CRAB_SWIMMING_PADDLE_CRAB_SWIMMING_H_
#define MJPC_TASKS_PADDLE_CRAB_SWIMMING_PADDLE_CRAB_SWIMMING_H_

#include <fstream>
#include <memory>
#include <string>
#include <mujoco/mujoco.h>
#include "mjpc/task.h"

namespace mjpc {

// PaddleCrabSwimming: straight-line swimming for the cross-domain hexapod with
// paddle-like rear leg tips. 18 controls (6 legs x 3 joints), ordered
// R1 R2 R3 L1 L2 L3 with (proximal, mid, distal) per leg.
//
// There is no scripted gait. The cost describes only where the robot should
// end up - moving forward, on the start line, level and on depth - and the
// planner has to discover a stroke that gets it there. Forward uses the Null
// (linear) norm so speed is a reward whose gradient never vanishes.
//
// The servos are dyntype="user" and the task supplies their dynamics through
// mjcb_act_dyn: a first-order lag da/dt = (ctrl - a) / tau on the setpoint,
// with tau taken from residual_ControlSmoothingTau in task.xml.
//
// Residual layout (56). Each steering term is error + lead * rate-of-error, and
// the rhythm terms treat (qdot/omega, q) as a phasor per joint:
//   [0]      -(forward speed)                      (Forward, linear reward)
//   [1]      lateral offset from the start line    (Lateral)
//   [2]      yaw of the body x-axis                (Heading)
//   [3-4]    tilt of the body z-axis               (Level)
//   [5]      depth deviation from the spawn height (Depth)
//   [6-23]   actuator commands                     (Effort)
//   [24-41]  left/right phasor mismatch            (Symmetry)
//   [42-49]  front/centre vs rear phasor mismatch  (RowPhase)
//   [50-55]  mid joint phasor radius - StrokeAmp   (Periodicity)
class PaddleCrabSwimming : public Task {
 public:
	std::string Name() const override;
	std::string XmlPath() const override;

	class ResidualFn : public mjpc::BaseResidualFn {
	 public:
		explicit ResidualFn(const PaddleCrabSwimming* task)
				: mjpc::BaseResidualFn(task) {}

		void Residual(const mjModel* model, const mjData* data,
									double* residual) const override;
	};

	PaddleCrabSwimming();
	void TransitionLocked(mjModel* model, mjData* data) override;

 protected:
	void ResetLocked(const mjModel* model) override;
	std::unique_ptr<mjpc::ResidualFn> ResidualLocked() const override {
		return std::make_unique<ResidualFn>(this);
	}
	ResidualFn* InternalResidual() override { return &residual_; }

 private:
	// Copies the smoothing slider where the global actuator dynamics callback can
	// see it. Must be called with the task lock held.
	void PublishSmoothingTau();

	ResidualFn residual_;
	// Logs "time,cost" for every real physics step; read by plot_cost.py.
	std::ofstream cost_log_;
	// Logs "time,x,y,z" for every real physics step; read by plot_position.py.
	std::ofstream pos_log_;
};

}  // namespace mjpc

#endif  // MJPC_TASKS_PADDLE_CRAB_SWIMMING_PADDLE_CRAB_SWIMMING_H_