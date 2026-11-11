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

// Evaluates the Pterosaur Launch-mode residual and cost weights at states read
// from stdin, to validate the MJWarp port in
// python/mujoco_mpc/mjwarp/tasks/pterosaur.py.
//
// usage: pterosaur_residual_dump path/to/task.xml < states.txt
// stdin:  nrow, then per row: time qpos[nq] qvel[nv] ctrl[nu]
//         (rows in increasing time; the first row is the launch start)
// stdout: per row: weight[num_term] residual[num_residual]

#include <cstdio>
#include <iostream>
#include <vector>

#include <mujoco/mujoco.h>
#include "mjpc/tasks/pterosaur/pterosaur.h"

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: " << argv[0] << " task.xml < states.txt\n";
    return 1;
  }
  char error[1024];
  mjModel* model = mj_loadXML(argv[1], nullptr, error, sizeof(error));
  if (!model) {
    std::cerr << error << "\n";
    return 1;
  }
  mjData* data = mj_makeData(model);
  mjpc::Pterosaur task;
  task.Reset(model);
  std::vector<double> residual(task.num_residual);

  int nrow = 0;
  std::cin >> nrow;
  for (int row = 0; row < nrow; row++) {
    std::cin >> data->time;
    for (int i = 0; i < model->nq; i++) std::cin >> data->qpos[i];
    for (int i = 0; i < model->nv; i++) std::cin >> data->qvel[i];
    for (int i = 0; i < model->nu; i++) std::cin >> data->ctrl[i];
    mj_forward(model, data);
    if (row == 0) {
      // the first transition initializes the task in Quadruped mode
      task.Transition(model, data);
      task.mode = 5;  // Launch
    }
    task.Transition(model, data);
    task.Residual(model, data, residual.data());
    for (int k = 0; k < task.num_term; k++) printf("%.9g ", task.weight[k]);
    for (int i = 0; i < task.num_residual; i++) printf("%.9g ", residual[i]);
    printf("\n");
  }
  mj_deleteData(data);
  mj_deleteModel(model);
  return 0;
}
