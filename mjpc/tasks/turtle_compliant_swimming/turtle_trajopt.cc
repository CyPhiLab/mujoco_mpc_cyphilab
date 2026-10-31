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

// Headless trajectory optimization for TurtleCompliantSwimming.
//
// Runs the MJPC agent in closed loop without the GUI, planning synchronously
// (so results do not depend on machine speed), and writes a CSV log plus a
// stroke-averaged summary. --mode=reference plays the reference gait open loop
// on the same model for comparison.
//
//   turtle_trajopt --mode=reference --duration=20 --out=ref.csv
//   turtle_trajopt --mode=mpc --duration=20 --weight=Gait=0.3 --out=mpc.csv
//   turtle_trajopt --numeric=agent_planner=2 ...   (iLQG instead of sampling)

#include <absl/flags/flag.h>
#include <absl/flags/parse.h>
#include <absl/strings/numbers.h>
#include <absl/strings/str_split.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>

#include "mjpc/agent.h"
#include "mjpc/planners/include.h"
#include "mjpc/planners/sampling/planner.h"
#include "mjpc/spline/spline.h"
#include "mjpc/task.h"
#include "mjpc/tasks/tasks.h"
#include "mjpc/tasks/turtle_compliant_swimming/turtle_compliant_swimming.h"
#include "mjpc/threadpool.h"
#include "mjpc/utilities.h"

ABSL_FLAG(std::string, mode, "mpc",
          "mpc: closed-loop MPC. reference: play the reference gait open loop.");
ABSL_FLAG(double, duration, 20.0, "simulated time [s]");
ABSL_FLAG(int, threads, mjpc::NumAvailableHardwareThreads(),
          "planner threads");
ABSL_FLAG(double, replan, 0.02, "simulated time between planning steps [s]");
ABSL_FLAG(int, iterations, 1, "planner iterations per planning step");
ABSL_FLAG(int, warmup, 20, "planner iterations at t = 0 before simulating");
ABSL_FLAG(bool, seed_reference, true,
          "sampling planner: extend the nominal plan with the reference gait "
          "instead of holding its last value");
ABSL_FLAG(std::vector<std::string>, weight, {},
          "cost weight overrides, Name=value (e.g. Gait=0.3,Power=0.05)");
ABSL_FLAG(std::vector<std::string>, param, {},
          "residual parameter overrides, Name=value (e.g. Speed=0.2)");
ABSL_FLAG(std::vector<std::string>, numeric, {},
          "task.xml custom numeric overrides applied before the agent starts, "
          "name=value (e.g. agent_planner=2,agent_horizon=1.5)");
ABSL_FLAG(std::string, gait_csv, "",
          "replace the reference gait with this table: one row per sample "
          "(uniform in time over one stroke), one column per actuator, "
          "header row with actuator names; same sample count as "
          "reference_gait.xml (e.g. a gait extracted by analyze_runs.py)");
ABSL_FLAG(std::string, out, "", "CSV log path (empty: no log)");
ABSL_FLAG(double, log_dt, 0.01, "CSV log interval [s]");

namespace {

using mjpc::spline::TimeSpline;

constexpr char kTaskName[] = "TurtleCompliantSwimming";
constexpr char kRootBody[] = "fr13_s105k";

std::vector<std::pair<std::string, double>> ParseAssignments(
    const std::vector<std::string>& items) {
  std::vector<std::pair<std::string, double>> out;
  for (const auto& item : items) {
    std::vector<std::string> kv = absl::StrSplit(item, '=');
    double value;
    if (kv.size() != 2 || !absl::SimpleAtod(kv[1], &value)) {
      mju_error("bad assignment '%s', expected name=value", item.c_str());
    }
    out.emplace_back(kv[0], value);
  }
  return out;
}

// Sampling planner: before each iteration, extend the winning candidate's plan
// to the end of the next horizon with reference-gait nodes. The planner
// resamples that candidate onto its new spline points, so the nominal plan
// follows the gait instead of holding its last node.
void SeedSamplingPlan(mjpc::SamplingPlanner* planner,
                      const mjpc::TurtleCompliantSwimming* task,
                      const mjModel* model, double time, double horizon) {
  TimeSpline& plan = planner->candidate_policy[planner->winner].plan;
  if (plan.Size() == 0) return;
  int n = std::max(planner->policy.num_spline_points, 2);
  double dt = horizon / (n - 1);
  double t = (plan.end() - 1)->time();
  std::vector<double> ref(model->nu);
  while (t < time + horizon + dt) {
    t += dt;
    task->ReferenceCtrl(model, t, ref.data());
    TimeSpline::Node node = plan.AddNode(t);
    std::copy(ref.begin(), ref.end(), node.values().begin());
  }
}

// Overwrite the turtle_gait_<actuator> numerics with a CSV table.
void LoadGaitCsv(const std::string& path, mjModel* model) {
  std::ifstream in(path);
  if (!in) mju_error("cannot open %s", path.c_str());
  std::string line;
  std::getline(in, line);
  std::vector<std::string> names = absl::StrSplit(line, ',');
  std::vector<std::vector<double>> columns(names.size());
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    std::vector<std::string> cells = absl::StrSplit(line, ',');
    if (cells.size() != names.size()) mju_error("ragged row in %s", path.c_str());
    for (size_t j = 0; j < cells.size(); j++) {
      double v;
      if (!absl::SimpleAtod(cells[j], &v)) mju_error("bad number in %s", path.c_str());
      columns[j].push_back(v);
    }
  }
  for (size_t j = 0; j < names.size(); j++) {
    std::string numeric = "turtle_gait_" + names[j];
    int id = mj_name2id(model, mjOBJ_NUMERIC, numeric.c_str());
    if (id < 0) mju_error("no numeric '%s'", numeric.c_str());
    if (model->numeric_size[id] != static_cast<int>(columns[j].size())) {
      mju_error("%s has %d samples, reference_gait.xml has %d", path.c_str(),
                static_cast<int>(columns[j].size()), model->numeric_size[id]);
    }
    mju_copy(model->numeric_data + model->numeric_adr[id], columns[j].data(),
             columns[j].size());
  }
}

// MJPC evaluates the cost through the sensor callback, as the GUI app does:
// the planning model uses the planner's frozen residual, the physics model the
// task's live one.
mjpc::Agent* g_agent = nullptr;
void SensorCallback(const mjModel* model, mjData* data, int stage) {
  if (stage != mjSTAGE_ACC || !g_agent || g_agent->allocate_enabled) return;
  if (g_agent->IsPlanningModel(model)) {
    g_agent->PlanningResidual()->Residual(model, data, data->sensordata);
  } else {
    g_agent->ActiveTask()->Residual(model, data, data->sensordata);
  }
}

struct Sample {
  double time;
  double com[3];
  double com_vel[3];
  double forward[3];
  double up_z;
  double power;
  double cost;
};

}  // namespace

int main(int argc, char** argv) {
  absl::ParseCommandLine(argc, argv);
  const std::string mode = absl::GetFlag(FLAGS_mode);
  if (mode != "mpc" && mode != "reference") {
    std::cerr << "--mode must be mpc or reference\n";
    return 1;
  }

  mjpc::Agent agent;
  agent.SetTaskList(mjpc::GetTasks());
  agent.gui_task_id = agent.GetTaskIdByName(kTaskName);
  mjpc::Agent::LoadModelResult load = agent.LoadModel();
  mjModel* model = load.model.get();
  if (!model) {
    std::cerr << load.error << "\n";
    return 1;
  }

  // numeric overrides go into the model before the agent reads them
  for (const auto& [name, value] : ParseAssignments(absl::GetFlag(FLAGS_numeric))) {
    int id = mj_name2id(model, mjOBJ_NUMERIC, name.c_str());
    if (id < 0) mju_error("no numeric '%s' in task.xml", name.c_str());
    model->numeric_data[model->numeric_adr[id]] = value;
  }

  const std::string gait_csv = absl::GetFlag(FLAGS_gait_csv);
  if (!gait_csv.empty()) LoadGaitCsv(gait_csv, model);

  mjData* data = mj_makeData(model);
  int home = mj_name2id(model, mjOBJ_KEY, "home");
  if (home >= 0) mj_resetDataKeyframe(model, data, home);
  mj_forward(model, data);

  agent.estimator_enabled = false;
  agent.Initialize(model);
  agent.Allocate();
  agent.Reset(data->ctrl);
  agent.plan_enabled = true;
  g_agent = &agent;
  mjcb_sensor = SensorCallback;

  auto* task = dynamic_cast<mjpc::TurtleCompliantSwimming*>(agent.ActiveTask());
  for (const auto& [name, value] : ParseAssignments(absl::GetFlag(FLAGS_weight))) {
    if (agent.SetWeightByName(name, value) < 0) {
      mju_error("no cost term '%s'", name.c_str());
    }
  }
  for (const auto& [name, value] : ParseAssignments(absl::GetFlag(FLAGS_param))) {
    if (agent.SetParamByName(name, value) < 0) {
      mju_error("no residual parameter '%s'", name.c_str());
    }
  }
  task->UpdateResidual();

  auto* sampling = dynamic_cast<mjpc::SamplingPlanner*>(&agent.ActivePlanner());
  bool seed = mode == "mpc" && sampling && absl::GetFlag(FLAGS_seed_reference);

  std::cout << "task " << kTaskName << ", mode " << mode << ", MuJoCo "
            << mj_versionString() << "\n";
  if (mode == "mpc") {
    std::cout << "planner " << mjpc::GetNumberOrDefault(0, model, "agent_planner")
              << " (0 sampling, 2 iLQG, 6 sample-gradient), horizon " << agent.Horizon() << " s, threads "
              << absl::GetFlag(FLAGS_threads)
              << (seed ? ", nominal seeded from reference gait" : "") << "\n";
  }
  std::cout << "weights:";
  for (int i = 0; i < task->num_term; i++) {
    std::cout << " " << task->weight_names[i] << "=" << task->weight[i];
  }
  std::cout << "\nparams:";
  const char* kParamNames[] = {"Speed", "Heading", "Cadence", "Phase"};
  for (int i = 0; i < 4 && i < static_cast<int>(task->parameters.size()); i++) {
    std::cout << " " << kParamNames[i] << "=" << task->parameters[i];
  }
  std::cout << "\n";

  int root = mj_name2id(model, mjOBJ_BODY, kRootBody);
  double total_mass = mj_getTotalmass(model);
  double period = model->numeric_data[model->numeric_adr[mj_name2id(
                      model, mjOBJ_NUMERIC, "turtle_gait_period")]] /
                  task->parameters[2];  // Cadence
  double heading = task->parameters[1];
  double dir[3] = {std::sin(heading), -std::cos(heading), 0.0};

  mjpc::ThreadPool pool(absl::GetFlag(FLAGS_threads));
  const double duration = absl::GetFlag(FLAGS_duration);
  const double replan = absl::GetFlag(FLAGS_replan);
  const int iterations = absl::GetFlag(FLAGS_iterations);
  const double horizon = agent.Horizon();

  auto plan = [&](int n) {
    for (int i = 0; i < n; i++) {
      if (seed) {
        SeedSamplingPlan(sampling, task, model, data->time, horizon);
      }
      agent.PlanIteration(&pool);
    }
  };

  // log file
  FILE* csv = nullptr;
  const std::string out = absl::GetFlag(FLAGS_out);
  if (!out.empty()) {
    csv = std::fopen(out.c_str(), "w");
    if (!csv) mju_error("cannot open %s", out.c_str());
    std::fprintf(csv, "time,com_x,com_y,com_z,vel_x,vel_y,vel_z,"
                      "fwd_x,fwd_y,fwd_z,up_z,power,cost");
    for (int i = 0; i < model->nu; i++) {
      std::fprintf(csv, ",ctrl_%s", mj_id2name(model, mjOBJ_ACTUATOR, i));
    }
    for (int i = 0; i < model->nu; i++) {
      std::fprintf(csv, ",ref_%s", mj_id2name(model, mjOBJ_ACTUATOR, i));
    }
    for (int i = 0; i < model->nq; i++) std::fprintf(csv, ",qpos_%d", i);
    std::fprintf(csv, "\n");
  }

  auto start = std::chrono::steady_clock::now();
  if (mode == "mpc") {
    agent.state.Set(model, data);
    plan(absl::GetFlag(FLAGS_warmup));
  }

  std::vector<double> ref(model->nu);
  std::vector<double> residual(task->num_residual);
  std::vector<Sample> samples;
  double next_plan = 0.0, next_log = 0.0;
  int steps = static_cast<int>(std::ceil(duration / model->opt.timestep));
  for (int s = 0; s < steps; s++) {
    task->Transition(model, data);
    agent.state.Set(model, data);
    if (mode == "mpc") {
      if (data->time >= next_plan - 1e-9) {
        plan(iterations);
        next_plan += replan;
      }
      agent.ActivePlanner().ActionFromPolicy(
          data->ctrl, agent.state.state().data(), agent.state.time(),
          /*use_previous=*/false);
    } else {
      task->ReferenceCtrl(model, data->time, data->ctrl);
    }
    // record the state, control and power this step integrates over
    mj_forward(model, data);
    task->Residual(model, data, residual.data());
    Sample x;
    x.time = data->time;
    mju_copy3(x.com, data->subtree_com + 3 * root);
    mju_copy3(x.com_vel, mjpc::SensorByName(model, data, "com_linvel"));
    const double* xmat = data->xmat + 9 * root;
    x.forward[0] = -xmat[1];
    x.forward[1] = -xmat[4];
    x.forward[2] = -xmat[7];
    x.up_z = xmat[8];
    x.power = 0.0;
    for (int i = 0; i < model->nu; i++) {
      x.power += mju_max(data->actuator_force[i] * data->actuator_velocity[i], 0.0);
    }
    x.cost = task->CostValue(residual.data());
    samples.push_back(x);

    if (csv && data->time >= next_log - 1e-9) {
      next_log += absl::GetFlag(FLAGS_log_dt);
      task->ReferenceCtrl(model, data->time, ref.data());
      std::fprintf(csv, "%.4f", x.time);
      for (double v : x.com) std::fprintf(csv, ",%.6f", v);
      for (double v : x.com_vel) std::fprintf(csv, ",%.6f", v);
      for (double v : x.forward) std::fprintf(csv, ",%.6f", v);
      std::fprintf(csv, ",%.6f,%.6f,%.6f", x.up_z, x.power, x.cost);
      for (int i = 0; i < model->nu; i++) std::fprintf(csv, ",%.6f", data->ctrl[i]);
      for (int i = 0; i < model->nu; i++) std::fprintf(csv, ",%.6f", ref[i]);
      for (int i = 0; i < model->nq; i++) std::fprintf(csv, ",%.6f", data->qpos[i]);
      std::fprintf(csv, "\n");
    }

    mj_step(model, data);
    if (!std::isfinite(data->qpos[0])) {
      std::cerr << "simulation diverged at t = " << data->time << "\n";
      return 1;
    }
  }
  double wall = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - start).count();
  if (csv) std::fclose(csv);

  // ----- summary over whole strokes, skipping the first two -----
  double t0 = 2 * period;
  int strokes = static_cast<int>(std::floor((duration - t0) / period + 1e-9));
  if (strokes < 1) {
    std::cout << "run too short for a stroke-averaged summary\n";
    return 0;
  }
  double t1 = t0 + strokes * period;
  auto at = [&](double t) {
    auto it = std::lower_bound(samples.begin(), samples.end(), t,
        [](const Sample& a, double b) { return a.time < b; });
    return static_cast<int>(std::min<long>(it - samples.begin(), samples.size() - 1));
  };
  int i0 = at(t0), i1 = at(t1);
  double dt = samples[i1].time - samples[i0].time;
  double disp[3];
  mju_sub3(disp, samples[i1].com, samples[i0].com);
  double along = mju_dot3(disp, dir);
  double dist = mju_norm3(disp);
  double work = 0.0, cost = 0.0, max_tilt = 0.0, max_yaw = 0.0;
  for (int i = i0 + 1; i <= i1; i++) {
    double h = samples[i].time - samples[i - 1].time;
    work += 0.5 * (samples[i].power + samples[i - 1].power) * h;
    cost += samples[i].cost * h;
    max_tilt = std::max(max_tilt, std::acos(mju_clip(samples[i].up_z, -1, 1)));
    double f[3];
    mju_copy3(f, samples[i].forward);
    double yaw_err = std::atan2(f[0] * dir[1] - f[1] * dir[0],
                                f[0] * dir[0] + f[1] * dir[1]);
    max_yaw = std::max(max_yaw, std::fabs(yaw_err));
  }
  const double* f_end = samples[i1].forward;
  double pitch_end = std::asin(mju_clip(f_end[2], -1, 1));
  const double g = 9.81;

  std::printf("\n== %s, %d strokes, t = %.2f..%.2f s (stroke %.3f s), "
              "wall %.1f s ==\n", mode.c_str(), strokes, t0, t1, period, wall);
  std::printf("COM displacement      [%+.3f %+.3f %+.3f] m\n",
              disp[0], disp[1], disp[2]);
  std::printf("speed along heading   %.4f m/s\n", along / dt);
  std::printf("net speed |disp|/t    %.4f m/s\n", dist / dt);
  std::printf("off-heading drift     %.4f m/s (sideways + vertical)\n",
              std::sqrt(std::max(dist * dist - along * along, 0.0)) / dt);
  std::printf("max tilt from upright %.1f deg, max yaw error %.1f deg, "
              "final pitch %+.1f deg\n",
              max_tilt * 180 / M_PI, max_yaw * 180 / M_PI, pitch_end * 180 / M_PI);
  std::printf("positive mech. work   %.2f J, mean power %.2f W\n", work,
              work / dt);
  std::printf("energy per metre      %.1f J/m along heading (CoT %.3f)\n",
              along > 0 ? work / along : NAN,
              along > 0 ? work / (total_mass * g * along) : NAN);
  std::printf("mean task cost        %.4f\n", cost / dt);
  return 0;
}
