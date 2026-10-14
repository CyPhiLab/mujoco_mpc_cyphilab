// Standalone headless tool: runs PaddleCrabSwimming closed loop (planner +
// 1 ms physics, like the GUI) for a list of settings and reports how each one
// swims. Each setting is a comma-free list of NAME=VALUE assignments separated
// by '+', e.g. "Gait=v0.08+Effort=0.1"; NAME is a residual parameter, a
// residual_select_ drop-down (Gait), or a cost term weight.
//
// Usage:
//   cost_sweep [--out cost_sweep.csv] [--seconds 20] [--replan 0.2]
//              [--plan_iters 1] [--agent_dt 0.02] [--threads 4] [--open_loop]
//              SETTING [SETTING ...]
//
//   --replan      seconds of simulation between planner iterations; the GUI
//                 manages about 5 replans/s with physics in real time
//   --open_loop   no planner: ctrl stays 0, so with a gait selected the robot
//                 runs the gait primitive alone (a check against the offline
//                 optimizer, analysis/gait_chain.py)
//
// Writes the per-step cost as "step,cost_<setting>,..." (read by
// plot_cost.py) and OUT_summary.csv with horizontal speed, displacement and
// heading per setting.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <absl/strings/str_split.h>
#include <mujoco/mujoco.h>
#include "mjpc/agent.h"
#include "mjpc/task.h"
#include "mjpc/tasks/paddle_crab_swimming/paddle_crab_swimming.h"
#include "mjpc/threadpool.h"
#include "mjpc/utilities.h"

namespace {
mjpc::Task* g_task = nullptr;

// Fills in the task's `user` sensor residuals during mj_forward/mj_step, as
// done by mjpc's grpc agent service (see grpc_agent_util.cc). Every model here
// is a copy of the task's, so the callback applies to all of them.
void ResidualSensorCallback(const mjModel* m, mjData* d, int stage) {
  if (g_task && stage == mjSTAGE_ACC) {
    g_task->Residual(m, d, d->sensordata);
  }
}

struct Result {
  std::string label;
  std::vector<double> cost;
  // speed is horizontal distance over time; dx, dy, dz the displacement.
  double speed = 0, dx = 0, dy = 0, dz = 0, yaw = 0, seconds = 0;
};

// Applies one "Name=Value" assignment; false if nothing matched.
bool Apply(mjpc::Agent& agent, const mjModel* model, const std::string& item) {
  std::vector<std::string> kv = absl::StrSplit(item, '=');
  if (kv.size() != 2) return false;
  if (mj_name2id(model, mjOBJ_TEXT, ("residual_list_" + kv[0]).c_str()) >= 0) {
    // A drop-down; ResidualParameterFromSelection gives 0 for unknown values.
    agent.SetSelectionParamByName(kv[0], kv[1]);
    return mjpc::ResidualSelection(
               model, kv[0],
               mjpc::ResidualParameterFromSelection(model, kv[0], kv[1])) ==
           kv[1];
  }
  char* end = nullptr;
  const double value = std::strtod(kv[1].c_str(), &end);
  if (end == kv[1].c_str()) return false;
  if (agent.SetParamByName(kv[0], value) >= 0) return true;
  const int term = mjpc::CostTermByName(model, kv[0]);
  if (term >= 0) {
    // Cost terms are the user sensors, in order.
    int index = 0;
    for (int i = 0; i < term; ++i) {
      if (model->sensor_type[i] == mjSENS_USER) ++index;
    }
    agent.ActiveTask()->weight[index] = value;
    return true;
  }
  return false;
}
}  // namespace

int main(int argc, char** argv) {
  std::string out_path = "cost_sweep.csv";
  double seconds = 20, replan = 0.2, agent_dt = -1;
  int plan_iters = 1, threads = 4;
  bool open_loop = false;
  std::vector<std::string> settings;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    auto next = [&]() { return std::string(i + 1 < argc ? argv[++i] : ""); };
    if (arg == "--out") out_path = next();
    else if (arg == "--seconds") seconds = std::atof(next().c_str());
    else if (arg == "--replan") replan = std::atof(next().c_str());
    else if (arg == "--plan_iters") plan_iters = std::atoi(next().c_str());
    else if (arg == "--agent_dt") agent_dt = std::atof(next().c_str());
    else if (arg == "--threads") threads = std::atoi(next().c_str());
    else if (arg == "--open_loop") open_loop = true;
    else settings.push_back(arg);
  }
  if (settings.empty()) settings.push_back("Gait=Off");

  auto task = std::make_shared<mjpc::PaddleCrabSwimming>();
  char error[1024] = "";
  mjModel* sim_model =
      mj_loadXML(task->XmlPath().c_str(), nullptr, error, sizeof(error));
  if (!sim_model) {
    std::fprintf(stderr, "Failed to load model: %s\n", error);
    return 1;
  }
  if (agent_dt > 0) {
    const int id = mj_name2id(sim_model, mjOBJ_NUMERIC, "agent_timestep");
    if (id >= 0) sim_model->numeric_data[sim_model->numeric_adr[id]] = agent_dt;
  }
  mjcb_sensor = ResidualSensorCallback;
  mjpc::ThreadPool pool(threads);
  const int plate = mj_name2id(sim_model, mjOBJ_BODY, "plate");

  std::vector<Result> results;
  for (const std::string& setting : settings) {
    // A fresh agent per setting, so nothing carries over between runs.
    mjpc::Agent agent(sim_model, task);
    g_task = agent.ActiveTask();
    const std::vector<std::string> items = absl::StrSplit(setting, '+');
    for (const std::string& item : items) {
      if (!Apply(agent, agent.GetModel(), item)) {
        std::fprintf(stderr, "unknown setting '%s'\n", item.c_str());
        return 1;
      }
    }
    g_task->UpdateResidual();

    mjData* data = mj_makeData(sim_model);
    mj_forward(sim_model, data);
    agent.Reset();
    agent.SetState(data);
    agent.plan_enabled = !open_loop;

    Result result;
    result.label = setting;
    const double y0 = data->xpos[3 * plate + 1];
    const double x0 = data->xpos[3 * plate];
    const double z0 = data->xpos[3 * plate + 2];
    std::vector<double> residual(g_task->num_residual, 0.0);
    double next_plan = 0;
    const int steps = static_cast<int>(std::ceil(seconds / sim_model->opt.timestep));
    for (int t = 0; t < steps; ++t) {
      g_task->Transition(sim_model, data);
      if (open_loop) {
        mju_zero(data->ctrl, sim_model->nu);
      } else {
        if (data->time >= next_plan - 1e-9) {
          agent.SetState(data);
          for (int i = 0; i < plan_iters; ++i) agent.PlanIteration(&pool);
          next_plan += replan;
        }
        agent.ActivePlanner().ActionFromPolicy(
            data->ctrl, agent.state.state().data(), data->time);
      }
      mj_step(sim_model, data);
      if (t % 10 == 0) {
        g_task->Residual(sim_model, data, residual.data());
        result.cost.push_back(g_task->CostValue(residual.data()));
      }
    }
    const double* xmat = data->xmat + 9 * plate;
    result.seconds = data->time;
    result.dx = data->xpos[3 * plate] - x0;
    result.dy = data->xpos[3 * plate + 1] - y0;
    result.dz = data->xpos[3 * plate + 2] - z0;
    result.speed = std::hypot(result.dx, result.dy) / data->time;
    result.yaw = std::atan2(xmat[3], xmat[0]);
    std::fprintf(stderr,
                 "%-28s speed %.4f m/s  dx %+.3f m  dy %+.3f m  dz %+.3f m  "
                 "yaw %+.3f rad\n",
                 setting.c_str(), result.speed, result.dx, result.dy,
                 result.dz, result.yaw);
    results.push_back(result);
    mj_deleteData(data);
    g_task = nullptr;
  }

  FILE* out = std::fopen(out_path.c_str(), "w");
  if (!out) {
    std::fprintf(stderr, "Failed to open output file: %s\n", out_path.c_str());
    return 1;
  }
  std::fprintf(out, "step");
  for (const Result& r : results) std::fprintf(out, ",cost_%s", r.label.c_str());
  std::fprintf(out, "\n");
  for (size_t t = 0; t < results[0].cost.size(); ++t) {
    std::fprintf(out, "%zu", t);
    for (const Result& r : results) std::fprintf(out, ",%.6f", r.cost[t]);
    std::fprintf(out, "\n");
  }
  std::fclose(out);

  std::string summary_path = out_path;
  const size_t dot = summary_path.rfind(".csv");
  summary_path = (dot == std::string::npos ? summary_path
                                           : summary_path.substr(0, dot)) +
                 "_summary.csv";
  FILE* summary = std::fopen(summary_path.c_str(), "w");
  if (summary) {
    std::fprintf(summary, "setting,seconds,speed,dx,dy,dz,yaw,open_loop\n");
    for (const Result& r : results) {
      std::fprintf(summary, "%s,%.3f,%.5f,%.5f,%.5f,%.5f,%.5f,%d\n",
                   r.label.c_str(), r.seconds, r.speed, r.dx, r.dy, r.dz,
                   r.yaw, open_loop);
    }
    std::fclose(summary);
  }
  std::fprintf(stderr, "Wrote %s and %s\n", out_path.c_str(),
               summary_path.c_str());
  mj_deleteModel(sim_model);
  return 0;
}
