// Copyright 2022 DeepMind Technologies Limited

#ifndef MJPC_TASKS_PADDLE_CRAB_SWIMMING_PADDLE_CRAB_SWIMMING_H_
#define MJPC_TASKS_PADDLE_CRAB_SWIMMING_PADDLE_CRAB_SWIMMING_H_

#include <string>
#include <mujoco/mujoco.h>
#include "mjpc/task.h"

namespace mjpc {

// PaddleCrabSwimming: 3D target-tracking task for the cross-domain hexapod
// robot with paddle-like back leg tips. 18 controls (6 legs x 3 joints).
// Residual layout (17 + model->nu = 35 total for 18-control hexapod):
//   [0-2]   Position error
//   [3]     Progress (one-sided desired speed floor)
//   [4-6]   Slip velocity (perpendicular to target direction)
//   [7-9]   Align using front_point direction from base_cog to front_point
//   [10-12] Trim (body-up stabilization)
//   [13-15] AngVel (angular velocity damping)
//   [16]    Control effort (morphology-aware weighted L2 norm)
//   [17-34] ControlRate for all 18 controls (model->nu)
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

	std::vector<double> prev_ctrl_;

 protected:
	std::unique_ptr<mjpc::ResidualFn> ResidualLocked() const override {
		return std::make_unique<ResidualFn>(this);
	}
	ResidualFn* InternalResidual() override { return &residual_; }

 private:
	ResidualFn residual_;
	std::vector<double> filtered_ctrl_;
	bool initialized_;
};

}  // namespace mjpc

#endif  // MJPC_TASKS_PADDLE_CRAB_SWIMMING_PADDLE_CRAB_SWIMMING_H_