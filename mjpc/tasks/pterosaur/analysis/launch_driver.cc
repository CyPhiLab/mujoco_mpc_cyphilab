// Headless MJPC launch driver for the Pterosaur "Launch Track" mode.
//
// Runs synchronous MPC (planning between physics steps) on a design variant
// of the pterosaur model and writes the trajectory to CSV for analysis and
// rendering (see run_mpc_launch.py).
//
// Usage:
//   launch_driver --out traj.csv [--gain_scale 1] [--stiffness k0,..,k11]
//     [--springref q0,..,q11] [--limits lo0,hi0,..,lo11,hi11]
//     [--speed 5] [--angle 30] [--ref_start 1.02] [--duration 2.5]
//     [--sim_dt 0.002] [--iters_per_step 1] [--threads 4]
//     [--horizon 0.35] [--plan_dt 0.01]
//     [--weights RefTakeoff=10,RefJointPos=0.5]

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>
#include "mjpc/agent.h"
#include "mjpc/task.h"
#include "mjpc/tasks/tasks.h"
#include "mjpc/threadpool.h"
#include "mjpc/utilities.h"

namespace {

mjpc::Task* task = nullptr;

void ResidualCallback(const mjModel* model, mjData* data, int stage) {
  if (stage == mjSTAGE_ACC) task->Residual(model, data, data->sensordata);
}

std::vector<double> ParseList(const std::string& s) {
  std::vector<double> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ',')) out.push_back(std::stod(item));
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  std::string out = "traj.csv";
  double gain_scale = 1, speed = 5, angle = 30, ref_start = 1.02;
  double duration = 2.5, sim_dt = 0.002;
  int iters_per_step = 1, threads = 4;
  double horizon = -1, plan_dt = -1;
  std::string weights;
  std::vector<double> stiffness, springref, limits;
  for (int i = 1; i + 1 < argc; i += 2) {
    std::string key = argv[i], val = argv[i + 1];
    if (key == "--out") out = val;
    else if (key == "--gain_scale") gain_scale = std::stod(val);
    else if (key == "--speed") speed = std::stod(val);
    else if (key == "--angle") angle = std::stod(val);
    else if (key == "--ref_start") ref_start = std::stod(val);
    else if (key == "--duration") duration = std::stod(val);
    else if (key == "--sim_dt") sim_dt = std::stod(val);
    else if (key == "--iters_per_step") iters_per_step = std::stoi(val);
    else if (key == "--threads") threads = std::stoi(val);
    else if (key == "--stiffness") stiffness = ParseList(val);
    else if (key == "--springref") springref = ParseList(val);
    else if (key == "--limits") limits = ParseList(val);
    else if (key == "--horizon") horizon = std::stod(val);
    else if (key == "--plan_dt") plan_dt = std::stod(val);
    else if (key == "--weights") weights = val;
    else {
      std::cerr << "unknown flag " << key << "\n";
      return 1;
    }
  }

  mjpc::Agent agent;
  agent.SetTaskList(mjpc::GetTasks());
  agent.gui_task_id = agent.GetTaskIdByName("Pterosaur");
  mjpc::Agent::LoadModelResult load = agent.LoadModel();
  mjModel* model = load.model.get();
  if (!model) {
    std::cerr << load.error << "\n";
    return 1;
  }

  // ----- design variant, applied before the agent copies the model ----- //
  for (int i = 0; i < model->nu; i++) model->actuator_gainprm[i*mjNGAIN] *= gain_scale;
  int nhinge = model->nu;  // one actuated hinge per actuator, after freejoint
  for (int j = 0; j < nhinge; j++) {
    int jnt = j + 1;
    if (!stiffness.empty()) model->jnt_stiffness[jnt] = stiffness[j];
    if (!springref.empty()) model->qpos_spring[model->jnt_qposadr[jnt]] = springref[j];
    if (!limits.empty()) {
      model->jnt_limited[jnt] = 1;
      model->jnt_range[2*jnt] = limits[2*j];
      model->jnt_range[2*jnt + 1] = limits[2*j + 1];
    }
  }

  if (horizon > 0) {
    double* h = mjpc::GetCustomNumericData(model, "agent_horizon");
    if (h) h[0] = horizon;
  }
  if (plan_dt > 0) {
    double* dt = mjpc::GetCustomNumericData(model, "agent_timestep");
    if (dt) dt[0] = plan_dt;
  }

  mjData* data = mj_makeData(model);
  int home = mj_name2id(model, mjOBJ_KEY, "home");
  if (home >= 0) mj_resetDataKeyframe(model, data, home);
  mj_forward(model, data);

  agent.estimator_enabled = false;
  agent.Initialize(model);  // copies model, planning timestep from the task
  agent.Allocate();
  agent.Reset(data->ctrl);
  agent.plan_enabled = true;
  task = agent.ActiveTask();
  mjcb_sensor = &ResidualCallback;

  // physics timestep may be finer than the planning timestep
  model->opt.timestep = sim_dt;

  // task parameters and mode
  task->parameters[mjpc::ParameterIndex(model, "Launch speed")] = speed;
  task->parameters[mjpc::ParameterIndex(model, "Launch angle")] = angle;
  task->parameters[mjpc::ParameterIndex(model, "Ref start")] = ref_start;

  // the first transition initializes the task in Quadruped; switch after it
  task->Transition(model, data);
  task->mode = 6;  // Launch Track (task_transition order in task.xml)

  mjpc::ThreadPool pool(threads);
  int floor_id = mj_name2id(model, mjOBJ_GEOM, "floor");
  int torso = mj_name2id(model, mjOBJ_BODY, "body");

  FILE* f = std::fopen(out.c_str(), "w");
  std::fprintf(f, "time,mode,floor_contact,com_x,com_z,comvel_x,comvel_y,comvel_z");
  for (int i = 0; i < model->nq; i++) std::fprintf(f, ",qpos%d", i);
  for (int i = 0; i < model->nu; i++) std::fprintf(f, ",ctrl%d", i);
  std::fprintf(f, "\n");

  // first transition snaps the state into the reference, then warm up the
  // planner at the start state before time advances
  task->Transition(model, data);
  mj_forward(model, data);

  // cost weight overrides, after entering the mode (which sets its weights)
  std::stringstream ws(weights);
  std::string item;
  while (std::getline(ws, item, ',')) {
    size_t eq = item.find('=');
    int id = mjpc::CostTermByName(model, item.substr(0, eq));
    if (id < 0) {
      std::cerr << "unknown cost term " << item << "\n";
      return 1;
    }
    task->weight[id] = std::stod(item.substr(eq + 1));
  }

  agent.state.Set(model, data);
  for (int i = 0; i < 20; i++) agent.PlanIteration(&pool);

  int steps = static_cast<int>(duration / sim_dt);
  // plan every 10 ms of simulated time
  int plan_every = std::max(1, static_cast<int>(0.01 / sim_dt + 0.5));
  for (int k = 0; k < steps; k++) {
    task->Transition(model, data);
    agent.state.Set(model, data);
    if (k % plan_every == 0) {
      for (int i = 0; i < iters_per_step; i++) agent.PlanIteration(&pool);
    }
    agent.ActivePlanner().ActionFromPolicy(data->ctrl, agent.state.state().data(),
                                           agent.state.time(), false);
    mj_step(model, data);

    if (data->warning[mjWARN_BADQACC].number > 0) {
      std::cerr << "simulation unstable at t=" << data->time << "\n";
      break;
    }
    int contact = 0;
    for (int c = 0; c < data->ncon; c++) {
      if (data->contact[c].geom1 == floor_id || data->contact[c].geom2 == floor_id) {
        contact = 1;
        break;
      }
    }
    const double* com = data->subtree_com + 3*torso;
    const double* comvel = data->subtree_linvel + 3*torso;
    std::fprintf(f, "%.5f,%d,%d,%.5f,%.5f,%.5f,%.5f,%.5f", data->time, task->mode,
                 contact, com[0], com[2], comvel[0], comvel[1], comvel[2]);
    for (int i = 0; i < model->nq; i++) std::fprintf(f, ",%.6f", data->qpos[i]);
    for (int i = 0; i < model->nu; i++) std::fprintf(f, ",%.4f", data->ctrl[i]);
    std::fprintf(f, "\n");
  }
  std::fclose(f);
  mjcb_sensor = nullptr;
  mj_deleteData(data);
  std::cout << "wrote " << out << "\n";
  return 0;
}
