// Standalone headless tool: runs the PaddleCrabSwimming planner for several
// DesiredSpeed values and writes per-step cost to a CSV file (one column per
// desired speed), so the cost-vs-time curves can be plotted and compared.
//
// Usage: cost_sweep [output_csv] [comma_separated_speeds] [control_steps] [plan_iters_per_step]
// Defaults: cost_sweep.csv, "0.1,0.2,...,1.0", 40, 3

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>
#include "mjpc/agent.h"
#include "mjpc/task.h"
#include "mjpc/tasks/paddle_crab_swimming/paddle_crab_swimming.h"
#include "mjpc/threadpool.h"

namespace {
mjpc::Task* g_task = nullptr;
mjModel* g_model = nullptr;

// Fills in the task's `user` sensor residuals during mj_forward/mj_step, as
// done by mjpc's grpc agent service (see grpc_agent_util.cc).
void ResidualSensorCallback(const mjModel* m, mjData* d, int stage) {
  if (m == g_model && stage == mjSTAGE_ACC) {
    g_task->Residual(m, d, d->sensordata);
  }
}

std::vector<double> ParseSpeeds(const std::string& csv) {
  std::vector<double> out;
  std::stringstream ss(csv);
  std::string item;
  while (std::getline(ss, item, ',')) out.push_back(std::atof(item.c_str()));
  return out;
}
}  // namespace

int main(int argc, char** argv) {
  std::string out_path = argc > 1 ? argv[1] : "cost_sweep.csv";
  std::vector<double> desired_speeds =
      argc > 2 ? ParseSpeeds(argv[2])
               : std::vector<double>{0.1, 0.2, 0.3, 0.4, 0.5,
                                      0.6, 0.7, 0.8, 0.9, 1.0};
  const int control_steps = argc > 3 ? std::atoi(argv[3]) : 40;
  const int plan_iters_per_step = argc > 4 ? std::atoi(argv[4]) : 3;

  auto task = std::make_shared<mjpc::PaddleCrabSwimming>();

  char load_error[1024] = "";
  mjModel* initial_model =
      mj_loadXML(task->XmlPath().c_str(), nullptr, load_error, sizeof(load_error));
  if (!initial_model) {
    std::fprintf(stderr, "Failed to load model: %s\n", load_error);
    return 1;
  }

  mjpc::Agent agent(initial_model, task);
  mj_deleteModel(initial_model);  // Agent keeps its own internal copy.

  g_task = agent.ActiveTask();
  g_model = agent.GetModel();
  mjcb_sensor = ResidualSensorCallback;

  mjData* data = mj_makeData(g_model);
  mjpc::ThreadPool pool(8);

  std::vector<std::vector<double>> all_costs(desired_speeds.size());

  for (size_t vi = 0; vi < desired_speeds.size(); ++vi) {
    double speed = desired_speeds[vi];
    agent.SetParamByName("DesiredSpeed", speed);
    g_task->UpdateResidual();

    mj_resetData(g_model, data);
    mj_forward(g_model, data);
    agent.Reset();
    agent.SetState(data);
    agent.plan_enabled = true;

    for (int t = 0; t < control_steps; ++t) {
      for (int i = 0; i < plan_iters_per_step; ++i) {
        agent.PlanIteration(&pool);
      }

      agent.state.CopyTo(g_model, data);
      mj_forward(g_model, data);
      g_task->Transition(g_model, data);
      agent.ActivePlanner().ActionFromPolicy(
          data->ctrl, agent.state.state().data(), agent.state.time());
      mj_step(g_model, data);
      agent.state.Set(g_model, data);

      std::vector<double> residual(g_task->num_residual, 0.0);
      g_task->Residual(g_model, data, residual.data());
      all_costs[vi].push_back(g_task->CostValue(residual.data()));
    }
    std::fprintf(stderr, "desired_speed=%.1f  final cost=%.4f\n", speed,
                 all_costs[vi].back());
  }

  FILE* out = std::fopen(out_path.c_str(), "w");
  if (!out) {
    std::fprintf(stderr, "Failed to open output file: %s\n", out_path.c_str());
    return 1;
  }
  std::fprintf(out, "step");
  for (double v : desired_speeds) std::fprintf(out, ",cost_v%.1f", v);
  std::fprintf(out, "\n");
  for (int t = 0; t < control_steps; ++t) {
    std::fprintf(out, "%d", t);
    for (size_t vi = 0; vi < desired_speeds.size(); ++vi) {
      std::fprintf(out, ",%.6f", all_costs[vi][t]);
    }
    std::fprintf(out, "\n");
  }
  std::fclose(out);
  std::fprintf(stderr, "Wrote %s\n", out_path.c_str());

  mj_deleteData(data);
  return 0;
}
