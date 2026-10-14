// Copyright 2022 DeepMind Technologies Limited

#include "mjpc/tasks/paddle_crab_swimming/paddle_crab_swimming.h"

#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <absl/strings/str_cat.h>
#include <absl/strings/str_split.h>

#include <mujoco/mujoco.h>
#include "mjpc/utilities.h"

namespace mjpc {

// A periodic servo command from gait_reference.xml, made offline by
// analysis/export_gaits.py: per actuator a Fourier series in phase,
//   u_i(phase) = c_i + sum_k a_ik cos(2 pi k phase) + b_ik sin(2 pi k phase)
// in radians. The numeric "gait_<name>" holds
//   period, K, speed, c (nu), a (nu x K), b (nu x K).
struct PaddleCrabGait {
	std::string name;
	double period = 1.0;
	int nu = 0;
	int k = 0;
	double speed = 0.0;
	std::vector<double> c, a, b;

	double Command(int i, double phase) const {
		double u = c[i];
		const double w = 2.0 * mjPI * phase;
		for (int j = 0; j < k; ++j) {
			u += a[i * k + j] * std::cos((j + 1) * w) +
					 b[i * k + j] * std::sin((j + 1) * w);
		}
		return u;
	}
};

namespace {

// Residual parameters, in the order the residual_* numerics appear in task.xml.
enum ParamIndex {
	kForwardX = 0,
	kForwardY,
	kForwardZ,
	kLeadPath,
	kLeadAttitude,
	kControlSmoothingTau,
	kStrokeFreq,
	kPhaseFront,
	kPhaseCentre,
	kStrokeAmp,
	kNumParam,
};

// The plate's free joint takes the first 7 qpos and 6 qvel entries; the 18 leg
// joints follow in actuator order (R1 R2 R3 L1 L2 L3, proximal/mid/distal).
constexpr int kJointQpos = 7;
constexpr int kJointQvel = 6;

// Sign mapping a right-side joint onto its left-side mirror, i.e. a target
// phase offset of 0 (+1) or pi (-1). It comes from the model's joint axes: the
// mid and front/centre distal axes are already mirror images of each other,
// while the proximal and rear-paddle axes are not.
double MirrorSign(int row, int joint) {
	if (joint == 1) return 1.0;
	if (joint == 2 && row != 2) return 1.0;
	return -1.0;
}

using Gait = PaddleCrabGait;

// Parses numeric "gait_<name>"; null if it is missing or malformed.
std::shared_ptr<const Gait> LoadGait(const mjModel* model,
																		 const std::string& name) {
	const int id = mj_name2id(model, mjOBJ_NUMERIC,
														absl::StrCat("gait_", name).c_str());
	if (id < 0) return nullptr;
	const double* data = model->numeric_data + model->numeric_adr[id];
	const int size = model->numeric_size[id];
	if (size < 3) return nullptr;
	auto gait = std::make_shared<Gait>();
	gait->name = name;
	gait->period = data[0];
	gait->k = static_cast<int>(data[1]);
	gait->speed = data[2];
	gait->nu = model->nu;
	const int nu = gait->nu, k = gait->k;
	if (gait->period <= 0 || k < 0 || size != 3 + nu * (1 + 2 * k)) {
		mju_warning("PaddleCrabSwimming: malformed gait_%s", name.c_str());
		return nullptr;
	}
	const double* coef = data + 3;
	gait->c.assign(coef, coef + nu);
	gait->a.assign(coef + nu, coef + nu + nu * k);
	gait->b.assign(coef + nu + nu * k, coef + nu + 2 * nu * k);
	return gait;
}

// The selected gait, the simulation time its clock started at, and a phase
// offset in cycles; published for the actuator dynamics callback below. A
// published gait is never freed (gait_keep_alive), so a rollout thread that
// loaded the pointer just before a switch can keep using it.
std::atomic<const Gait*> active_gait{nullptr};
std::atomic<double> gait_start_time{0.0};
std::atomic<double> gait_phase{0.0};
std::mutex gait_keep_alive_mutex;
std::vector<std::shared_ptr<const Gait>> gait_keep_alive;

// Smoothing time constant, published for the actuator dynamics callback below.
// The callback is global and runs on every rollout thread, so it cannot reach
// into the task.
std::atomic<double> smoothing_tau{0.08};

// Actuator dynamics for the model's dyntype="user" servos: a first-order lag on
// the setpoint, so the joint is driven by a filtered command rather than the
// planner's raw one. MuJoCo integrates this inside every mj_step, which keeps
// the filter identical in the simulation and in each planner rollout, and keeps
// its state in mjData where each rollout thread has its own copy.
mjtNum ControlSmoothingDynamics(const mjModel* model, const mjData* data,
																		int id) {
	// MuJoCo integrates act explicitly, so a tau below the timestep would ring.
	const double tau = mju_max(smoothing_tau.load(std::memory_order_relaxed),
													 model->opt.timestep);
	double setpoint = data->ctrl[id];
	const Gait* gait = active_gait.load(std::memory_order_acquire);
	if (gait && id < gait->nu) {
		const double phase =
				(data->time - gait_start_time.load(std::memory_order_relaxed)) /
						gait->period +
				gait_phase.load(std::memory_order_relaxed);
		setpoint += gait->Command(id, phase);
		if (model->actuator_ctrllimited[id]) {
			setpoint = mju_clip(setpoint, model->actuator_ctrlrange[2 * id],
													model->actuator_ctrlrange[2 * id + 1]);
		}
	}
	return (setpoint - data->act[id]) / tau;
}

}  // namespace

std::string PaddleCrabSwimming::XmlPath() const {
	return GetModelPath("paddle_crab_swimming/task.xml");
}

std::string PaddleCrabSwimming::Name() const { return "PaddleCrabSwimming"; }

PaddleCrabSwimming::PaddleCrabSwimming() : residual_(this) {
	cost_log_.open("paddle_crab_swimming_cost_log.csv", std::ios::trunc);
	cost_log_ << "time,cost\n";
	pos_log_.open("paddle_crab_swimming_pos_log.csv", std::ios::trunc);
	pos_log_ << "time,x,y,z\n";

	// Only actuators declared dyntype="user" reach this callback, so installing
	// it globally is harmless for the other tasks in the binary.
	mjcb_act_dyn = &ControlSmoothingDynamics;
}

// Publishes the smoothing slider to the actuator dynamics callback.
void PaddleCrabSwimming::PublishSmoothingTau() {
	if (kControlSmoothingTau < static_cast<int>(parameters.size())) {
		smoothing_tau.store(parameters[kControlSmoothingTau],
											std::memory_order_relaxed);
	}
}

// Publishes the gait selected in residual_select_Gait, its clock starting at
// `time`, to the actuator dynamics callback.
void PaddleCrabSwimming::PublishGait(double time) {
	active_gait_ = gait_param_ >= 0 &&
										 gait_param_ < static_cast<int>(parameters.size())
										 ? ReinterpretAsInt(parameters[gait_param_])
										 : 0;
	const Gait* gait = nullptr;
	if (active_gait_ > 0 &&
			active_gait_ < static_cast<int>(gait_library_.size())) {
		gait = gait_library_[active_gait_].get();
	}
	gait_start_time.store(time, std::memory_order_relaxed);
	if (gait_phase_param_ >= 0) {
		gait_phase.store(parameters[gait_phase_param_], std::memory_order_relaxed);
	}
	active_gait.store(gait, std::memory_order_release);
}

void PaddleCrabSwimming::ResetLocked(const mjModel* model) {
	PublishSmoothingTau();

	// Gait library, indexed like the residual_list_Gait drop-down ("Off" first).
	gait_param_ = ParameterIndex(model, "select_Gait");
	gait_phase_param_ = ParameterIndex(model, "GaitPhase");
	gait_library_.assign(1, nullptr);
	const auto lists = ResidualSelectionLists(model);
	if (auto it = lists.find("Gait"); it != lists.end()) {
		for (size_t i = 1; i < it->second.size(); ++i) {
			auto gait = LoadGait(model, it->second[i]);
			if (!gait) {
				mju_warning("PaddleCrabSwimming: no gait_%s in gait_reference.xml",
										it->second[i].c_str());
			}
			gait_library_.push_back(gait);
		}
	}
	{
		std::lock_guard<std::mutex> lock(gait_keep_alive_mutex);
		for (const auto& gait : gait_library_) {
			if (gait) gait_keep_alive.push_back(gait);
		}
	}
	PublishGait(0.0);
}

void PaddleCrabSwimming::ResidualFn::Residual(const mjModel* model,
																							const mjData* data,
																							double* residual) const {
	int counter = 0;

	// Forward axis, and the horizontal axis to its left.
	double forward[3] = {parameters_[kForwardX], parameters_[kForwardY],
											 parameters_[kForwardZ]};
	mju_normalize3(forward);
	const double up[3] = {0.0, 0.0, 1.0};
	double lateral[3];
	mju_cross(lateral, up, forward);

	double* pos = SensorByName(model, data, "base_pos");
	double* linvel = SensorByName(model, data, "base_linvel");
	double* angvel = SensorByName(model, data, "base_angvel");
	double* xaxis = SensorByName(model, data, "base_xaxis");
	double* zaxis = SensorByName(model, data, "base_zaxis");

	// Steering terms are error + lead * d(error)/dt. The lead makes a slow
	// correction pay off inside the horizon, but it also prices in the body
	// rocking that swimming unavoidably causes, so it stays small.
	const double lead_path = parameters_[kLeadPath];
	const double lead_attitude = parameters_[kLeadAttitude];

	// ----- Forward ----- //
	// Null norm, so this is a linear reward on speed rather than a target to
	// settle on: the gradient never vanishes, which is what lets the planner
	// find a stroke instead of stalling once it is "fast enough".
	residual[counter++] = -mju_dot3(linvel, forward);

	// ----- Lateral ----- //
	residual[counter++] =
			mju_dot3(lateral, pos) + lead_path * mju_dot3(lateral, linvel);

	// ----- Heading ----- //
	// The body spawns aligned with the world, so any y-component of its x-axis
	// is a yaw error.
	residual[counter++] = xaxis[1] + lead_path * angvel[2];

	// ----- Level ----- //
	residual[counter++] = zaxis[0] + lead_attitude * angvel[1];
	residual[counter++] = zaxis[1] - lead_attitude * angvel[0];

	// ----- Depth ----- //
	residual[counter++] = pos[2] - model->qpos0[2] + lead_attitude * linvel[2];

	// ----- Effort ----- //
	for (int i = 0; i < model->nu; ++i) {
		residual[counter++] = data->ctrl[i];
	}

	// A joint oscillating at omega traces a circle in (qdot/omega, q), so that
	// pair is a phasor and phase relationships become vector comparisons. That
	// avoids an angle difference, which would be discontinuous at the wrap and
	// undefined when a joint is at rest.
	const double omega = 2.0 * mjPI * mju_max(parameters_[kStrokeFreq], 0.01);

	// A gait primitive already carries the rhythm (see the header); the emergent
	// rhythm terms would only pull against it.
	const bool gait = active_gait.load(std::memory_order_relaxed) != nullptr;
	const int rhythm_start = counter;

	// ----- Symmetry ----- //
	// Left/right phase lock. The target offset is 0 or pi, so the phasor rotation
	// collapses to MirrorSign. Comparing whole phasors locks amplitude and phase
	// together, which comparing joint angles alone would not.
	//
	// Deliberately not normalised by amplitude. Dividing through makes a small
	// exploratory wiggle score a large *relative* asymmetry, which penalises
	// exactly the tentative motion the planner needs to get started; measured, it
	// froze at rest for every weight tried. Growing as amplitude squared instead
	// leaves early exploration almost free.
	for (int row = 0; row < 3; ++row) {
		for (int joint = 0; joint < 3; ++joint) {
			const int right = 3 * row + joint;
			const int left = 9 + 3 * row + joint;
			const double sign = MirrorSign(row, joint);
			residual[counter++] = data->qpos[kJointQpos + left] -
														 sign * data->qpos[kJointQpos + right];
			residual[counter++] = (data->qvel[kJointQvel + left] -
													 sign * data->qvel[kJointQvel + right]) /
													omega;
		}
	}

	// ----- RowPhase ----- //
	// Metachronal timing: hold each front and centre mid joint at a commanded
	// phase offset from the rear mid joint on the same side.
	auto phase_lock = [&](int lead_index, int ref_index, double delta) {
		const double lead[2] = {data->qvel[kJointQvel + lead_index] / omega,
													data->qpos[kJointQpos + lead_index]};
		const double ref[2] = {data->qvel[kJointQvel + ref_index] / omega,
											 data->qpos[kJointQpos + ref_index]};
		const double c = mju_cos(delta), s = mju_sin(delta);
		residual[counter++] = lead[0] - (c * ref[0] - s * ref[1]);
		residual[counter++] = lead[1] - (s * ref[0] + c * ref[1]);
	};
	phase_lock(1, 7, parameters_[kPhaseFront]);
	phase_lock(4, 7, parameters_[kPhaseCentre]);
	phase_lock(10, 16, parameters_[kPhaseFront]);
	phase_lock(13, 16, parameters_[kPhaseCentre]);

	// ----- Periodicity ----- //
	// Hold each mid joint's phasor on a circle of radius StrokeAmp, i.e. keep it
	// oscillating steadily rather than settling anywhere.
	//
	// A q + qddot/omega^2 form is the textbook pointwise test for harmonic motion
	// but is useless here: joint acceleration is dominated by servo stiffness, not
	// the gait envelope, so it acts as an acceleration penalty and froze the
	// planner at weights as low as 0.001. This form needs no acceleration, and
	// being large at rest it pushes the planner out of the do-nothing minimum
	// instead of into it.
	const double stroke_amp = parameters_[kStrokeAmp];
	for (int leg = 0; leg < 6; ++leg) {
		const int index = 3 * leg + 1;  // mid joint carries the up/down stroke
		const double q = data->qpos[kJointQpos + index];
		const double v = data->qvel[kJointQvel + index] / omega;
		residual[counter++] = mju_sqrt(q * q + v * v) - stroke_amp;
	}
	if (gait) {
		mju_zero(residual + rhythm_start, counter - rhythm_start);
	}

	CheckSensorDim(model, counter);
}

void PaddleCrabSwimming::TransitionLocked(mjModel* model, mjData* data) {
	PublishSmoothingTau();

	// Restart the gait clock whenever the selection changes; follow the phase
	// slider continuously.
	const int selected = gait_param_ >= 0
													 ? ReinterpretAsInt(parameters[gait_param_])
													 : 0;
	if (selected != active_gait_) {
		PublishGait(data->time);
	} else if (gait_phase_param_ >= 0) {
		gait_phase.store(parameters[gait_phase_param_], std::memory_order_relaxed);
	}

	double* base_pos = SensorByName(model, data, "base_pos");
	if (pos_log_.is_open() && base_pos) {
		pos_log_ << data->time << "," << base_pos[0] << "," << base_pos[1] << ","
						 << base_pos[2] << "\n";
		pos_log_.flush();
	}

	if (cost_log_.is_open()) {
		std::vector<double> residual(num_residual, 0.0);
		residual_.Residual(model, data, residual.data());
		cost_log_ << data->time << "," << residual_.CostValue(residual.data())
							<< "\n";
		cost_log_.flush();
	}
}

}  // namespace mjpc
