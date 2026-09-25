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

#include "mjpc/tasks/pterosaur/pterosaur.h"

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>
#include <cmath>

#include <mujoco/mujoco.h>
#include "mjpc/task.h"
#include "mjpc/utilities.h"

namespace mjpc {
std::string Pterosaur::XmlPath() const {
  return GetModelPath("pterosaur/task.xml");
}
std::string Pterosaur::Name() const { return "Pterosaur"; }

void Pterosaur::ResidualFn::Residual(const mjModel* model,
                                         const mjData* data,
                                         double* residual) const {
  // start counter
  int counter = 0;

  // get foot positions
  double* foot_pos[kNumFoot];
  for (A1Foot foot : kFootAll)
    foot_pos[foot] = data->geom_xpos + 3 * foot_geom_id_[foot];

  // average foot position
  double avg_foot_pos[3];
  AverageFootPos(avg_foot_pos, foot_pos);

  double* torso_xmat = data->xmat + 9*torso_body_id_;
  double* goal_pos = data->mocap_pos + 3*goal_mocap_id_;
  double* compos = SensorByName(model, data, "torso_subtreecom");


  // ---------- Upright ----------
if (current_mode_ != kModeFlip && current_mode_ != kModeLaunch) {
    if (current_mode_ == kModeBiped) {
      double biped_type = parameters_[biped_type_param_id_];
      int handstand = ReinterpretAsInt(biped_type) ? -1 : 1;
      residual[counter++] = torso_xmat[6] - handstand;
    } else {
      residual[counter++] = torso_xmat[8] - 1;
    }
    residual[counter++] = 0;
    residual[counter++] = 0;
  } else if (current_mode_ == kModeFlip) {
    // special handling of flip orientation
    double flip_time = data->time - mode_start_time_;
    double quat[4];
    FlipQuat(quat, flip_time);
    double* torso_xquat = data->xquat + 4*torso_body_id_;
    mju_subQuat(residual + counter, torso_xquat, quat);
    counter += 3;
  } else {
    // special handling of launch orientation
    double launch_time = data->time - mode_start_time_;
    double rise_start_upright = preload_time_;
    double rise_end_upright = preload_time_ + rise_time_;
    double pivot_end_upright = rise_end_upright + pivot_time_;
    if (launch_time >= rise_end_upright && launch_time < pivot_end_upright) {
      // pivot: disable upright cost
      mju_zero(residual + counter, 3);
    } else if (launch_time >= rise_start_upright &&
               launch_time < rise_end_upright) {
      // rise: interpolate body orientation crouch -> pivot
      double alpha = (launch_time - rise_start_upright) / rise_time_;
      alpha = mju_min(1.0, mju_max(0.0, alpha));
      double* crouch = KeyQPosByName(model, data, "crouch");
      double* pivot = KeyQPosByName(model, data, "pivot");
      double quat[4];
      for (int i = 0; i < 4; ++i) {
        quat[i] = (1.0 - alpha) * crouch[3 + i] + alpha * pivot[3 + i];
      }
      mju_normalize4(quat);
      double* torso_xquat = data->xquat + 4*torso_body_id_;
      mju_subQuat(residual + counter, torso_xquat, quat);
    } else {
      double quat[4];
      LaunchQuat(quat, launch_time);
      double* torso_xquat = data->xquat + 4*torso_body_id_;
      mju_subQuat(residual + counter, torso_xquat, quat);
    }
    counter += 3;
  }


  // ---------- Height ----------
  // quadrupedal or bipedal height of torso over feet
  double* torso_pos = data->xipos + 3*torso_body_id_;
  bool is_biped = current_mode_ == kModeBiped;
  double height_goal = is_biped ? kHeightBiped : kHeightQuadruped;
  if (current_mode_ == kModeScramble) {
    // disable height term in Scramble
    residual[counter++] = 0;
  } else if (current_mode_ == kModeFlip) {
    // height target for Backflip
    double flip_time = data->time - mode_start_time_;
    residual[counter++] = torso_pos[2] - FlipHeight(flip_time);
  } else if (current_mode_ == kModeLaunch) {
    double launch_time = data->time - mode_start_time_;
    residual[counter++] = torso_pos[2] - LaunchHeight(launch_time);
  } else {
    residual[counter++] = (torso_pos[2] - avg_foot_pos[2]) - height_goal;
  }


  // ---------- Position ----------
  double* head = data->site_xpos + 3*head_site_id_;
  double target[3];
  if (current_mode_ == kModeWalk) {
    // follow prescribed Walk trajectory
    double mode_time = data->time - mode_start_time_;
    Walk(target, mode_time);
  } else if (current_mode_ == kModeLaunch) {
    double launch_time = data->time - mode_start_time_;
    double forward[3] = {torso_xmat[0], torso_xmat[3], torso_xmat[6]};
    mju_normalize(forward, 3);
    double horizontal_speed = ResidualFn::kLaunchSpeed *
        mju_cos(ResidualFn::kLaunchAngle);
    double rise_end = preload_time_ + rise_time_;
    double pivot_end = rise_end + pivot_time_;
    if (launch_time < preload_time_) {
      double alpha = launch_time / preload_time_;
      double alpha_smooth = alpha * alpha * (3.0 - 2.0 * alpha);
      double preload_shift = -0.05 + 0.10 * alpha_smooth;
      target[0] = position_[0] + forward[0] * preload_shift;
      target[1] = position_[1] + forward[1] * preload_shift;
    } else if (launch_time < rise_end) {
      // rise: maintain near-crouch horizontal position while extending upward
      target[0] = position_[0] + forward[0] * 0.05;
      target[1] = position_[1] + forward[1] * 0.05;
    } else if (launch_time < pivot_end) {
      double time_in_pivot = launch_time - rise_end;
      double alpha = time_in_pivot / pivot_time_;
      // Interpolate distance from 0 to 0.5 m over the pivot phase
      double pivot_distance = 0.5 * alpha;
      target[0] = position_[0] + forward[0] * pivot_distance;
      target[1] = position_[1] + forward[1] * pivot_distance;
    } else {
      double elapsed = launch_time - pivot_end;
      double pivot_distance = 0.05 + 1.5 * horizontal_speed * pivot_time_;
      target[0] = position_[0] + forward[0] *
          (pivot_distance + horizontal_speed * elapsed);
      target[1] = position_[1] + forward[1] *
          (pivot_distance + horizontal_speed * elapsed);
    }
    target[2] = head[2];
  } else {
    // go to the goal mocap body
    target[0] = goal_pos[0];
    target[1] = goal_pos[1];
    target[2] = goal_pos[2];
  }
  residual[counter++] = head[0] - target[0];
  residual[counter++] = head[1] - target[1];
  residual[counter++] =
      current_mode_ == kModeScramble ? 2 * (head[2] - target[2]) : 0;

  double* comvel = SensorByName(model, data, "torso_subtreelinvel");

  // ---------- Launch velocity ----------
  if (current_mode_ == kModeLaunch) {
    double launch_time = data->time - mode_start_time_;
    double forward[3] = {torso_xmat[0], torso_xmat[3], torso_xmat[6]};
    double up[3] = {torso_xmat[2], torso_xmat[5], torso_xmat[8]};
    mju_normalize(forward, 3);
    mju_normalize(up, 3);
    double desired_dir[3];
    double desired_speed = 0;
    double rise_end = preload_time_ + rise_time_;
    double pivot_end = rise_end + pivot_time_;
    if (launch_time < pivot_end) {
      mju_add3(desired_dir, forward, up);
      desired_speed = ResidualFn::kLaunchPivotSpeed;
    } else if (launch_time < pivot_end + jump_time_) {
      mju_add3(desired_dir, forward, up);
      desired_speed = ResidualFn::kLaunchSpeed;
    } else if (launch_time < pivot_end + jump_time_ +
               flight_time_) {
      mju_add3(desired_dir, forward, up);
      desired_speed = ResidualFn::kLaunchSpeed;
    } else {
      mju_zero(desired_dir, 3);
    }
    if (desired_speed > 0) {
      mju_normalize(desired_dir, 3);
    }
    residual[counter++] = mju_dot(comvel, desired_dir, 3) - desired_speed;
  } else {
    residual[counter++] = 0;
  }

  // ---------- Gait ----------
  A1Gait gait = GetGait();
  double step[kNumFoot];
  FootStep(step, GetPhase(data->time), gait);
  for (A1Foot foot : kFootAll) {
    if (is_biped) {
      // ignore "hands" in biped mode
      bool handstand = ReinterpretAsInt(parameters_[biped_type_param_id_]);
      bool front_hand = !handstand && (foot == kFootFL || foot == kFootFR);
      bool back_hand = handstand && (foot == kFootHL || foot == kFootHR);
      if (front_hand || back_hand) {
        residual[counter++] = 0;
        continue;
      }
    }
    double query[3] = {foot_pos[foot][0], foot_pos[foot][1], foot_pos[foot][2]};

    if (current_mode_ == kModeScramble) {
      double torso_to_goal[3];
      double* goal = data->mocap_pos + 3*goal_mocap_id_;
      mju_sub3(torso_to_goal, goal, torso_pos);
      mju_normalize3(torso_to_goal);
      mju_sub3(torso_to_goal, goal, foot_pos[foot]);
      torso_to_goal[2] = 0;
      mju_normalize3(torso_to_goal);
      mju_addToScl3(query, torso_to_goal, 0.15);
    }

    double ground_height = Ground(model, data, query);
    double height_target = ground_height + kFootRadius + step[foot];
    double height_difference = foot_pos[foot][2] - height_target;
    if (current_mode_ == kModeScramble) {
      // in Scramble, foot higher than target is not penalized
      height_difference = mju_min(0, height_difference);
    }
    residual[counter++] = step[foot] ? height_difference : 0;
  }


  // ---------- Balance ----------
  double capture_point[3];
  double fall_time = mju_sqrt(2*height_goal / 9.81);
  mju_addScl3(capture_point, compos, comvel, fall_time);
  if (current_mode_ == kModeLaunch) {
    double launch_time = data->time - mode_start_time_;
    double rise_end = preload_time_ + rise_time_;
    double pivot_end = rise_end + pivot_time_;
    if (launch_time < preload_time_) {
      residual[counter++] = capture_point[0] - avg_foot_pos[0];
      residual[counter++] = capture_point[1] - avg_foot_pos[1];
    } else if (launch_time < rise_end) {
      residual[counter++] = capture_point[0] - avg_foot_pos[0];
      residual[counter++] = capture_point[1] - avg_foot_pos[1];
    } else if (launch_time < pivot_end) {
      double front_support[3] = {0};
      front_support[0] = 0.5 * (foot_pos[kFootFL][0] + foot_pos[kFootFR][0]);
      front_support[1] = 0.5 * (foot_pos[kFootFL][1] + foot_pos[kFootFR][1]);
      front_support[2] = 0.5 * (foot_pos[kFootFL][2] + foot_pos[kFootFR][2]);
      residual[counter++] = capture_point[0] - front_support[0];
      residual[counter++] = capture_point[1] - front_support[1];
    } else {
      residual[counter++] = capture_point[0] - avg_foot_pos[0];
      residual[counter++] = capture_point[1] - avg_foot_pos[1];
    }
  } else {
    residual[counter++] = capture_point[0] - avg_foot_pos[0];
    residual[counter++] = capture_point[1] - avg_foot_pos[1];
  }


  // ---------- Effort ----------
  mju_scl(residual + counter, data->actuator_force, 2e-2, model->nu);
  if (current_mode_ == kModeLaunch) {
    double launch_time = data->time - mode_start_time_;
    double rise_end = preload_time_ + rise_time_;
    double pivot_end = rise_end + pivot_time_;
    if (launch_time < pivot_end) {
      const double crouch_effort_scale = 3.0;
      const int abduction_dofs[4] = {0, 3, 6, 9};
      for (int i = 0; i < 4; ++i) {
        residual[counter + abduction_dofs[i]] *= crouch_effort_scale;
      }

      if (launch_time >= preload_time_ && launch_time < rise_end) {
        // rise: encourage leg hip2 more strongly while keeping leg hip1 costly.
        const double leg_abduction_extra_scale = 1.5;
        const int leg_abduction_dofs[2] = {6, 9};
        for (int i = 0; i < 2; ++i) {
          residual[counter + leg_abduction_dofs[i]] *=
              leg_abduction_extra_scale;
        }

        const double arm_and_knee_effort_scale = 0.5;
        const int arm_and_knee_dofs[6] = {1, 2, 4, 5, 8, 11};
        for (int i = 0; i < 6; ++i) {
          residual[counter + arm_and_knee_dofs[i]] *=
              arm_and_knee_effort_scale;
        }

        const double leg_hip2_effort_scale = 0.35;
        const int leg_hip2_dofs[2] = {7, 10};
        for (int i = 0; i < 2; ++i) {
          residual[counter + leg_hip2_dofs[i]] *= leg_hip2_effort_scale;
        }
      } else if (launch_time >= rise_end && launch_time < pivot_end) {
        // pivot: use same effort scaling as rise
        const double leg_abduction_extra_scale = 1.5;
        const int leg_abduction_dofs[2] = {6, 9};
        for (int i = 0; i < 2; ++i) {
          residual[counter + leg_abduction_dofs[i]] *=
              leg_abduction_extra_scale;
        }

        const double arm_and_knee_effort_scale = 0.5;
        const int arm_and_knee_dofs[6] = {1, 2, 4, 5, 8, 11};
        for (int i = 0; i < 6; ++i) {
          residual[counter + arm_and_knee_dofs[i]] *=
              arm_and_knee_effort_scale;
        }

        const double leg_hip2_effort_scale = 0.35;
        const int leg_hip2_dofs[2] = {7, 10};
        for (int i = 0; i < 2; ++i) {
          residual[counter + leg_hip2_dofs[i]] *= leg_hip2_effort_scale;
        }
      }
    }
  }
  counter += model->nu;


  // ---------- Posture ----------
  constexpr int kPostureOrientDim = 3;
  double* home = KeyQPosByName(model, data, "home");
  double* posture_residual = residual + counter;
  mju_zero(posture_residual, kPostureOrientDim + model->nu);
  mju_sub(posture_residual + kPostureOrientDim, data->qpos + 7, home + 7,
          model->nu);
  if (current_mode_ == kModeFlip) {
    double flip_time = data->time - mode_start_time_;
    if (flip_time < crouch_time_) {
      double* crouch = KeyQPosByName(model, data, "crouch");
      mju_sub(posture_residual + kPostureOrientDim, data->qpos + 7,
              crouch + 7, model->nu);
    } else if (flip_time >= crouch_time_ &&
               flip_time < jump_time_ + flight_time_) {
      // free legs during flight phase
      mju_zero(posture_residual + kPostureOrientDim, model->nu);
    }
  } else if (current_mode_ == kModeLaunch) {
    double launch_time = data->time - mode_start_time_;
    double* crouch = KeyQPosByName(model, data, "crouch");
    double* pivot = KeyQPosByName(model, data, "pivot");
    double rise_end = preload_time_ + rise_time_;
    double pivot_end = rise_end + pivot_time_;
    if (launch_time < rise_end) {
      std::vector<double> target(model->nu);
      if (launch_time < preload_time_) {
        // preload: interpolate home -> crouch
        double alpha = launch_time / preload_time_;
        alpha = mju_min(1.0, mju_max(0.0, alpha));
        for (int i = 0; i < model->nu; ++i) {
          double home_val = home[7 + i];
          double crouch_val = crouch[7 + i];
          target[i] = (1.0 - alpha) * home_val + alpha * crouch_val;
        }
      } else {
        // rise: interpolate crouch -> home
        double alpha = (launch_time - preload_time_) / rise_time_;
        alpha = mju_min(1.0, mju_max(0.0, alpha));
        for (int i = 0; i < model->nu; ++i) {
          double home_val = home[7 + i];
          double crouch_val = crouch[7 + i];
          target[i] = (1.0 - alpha) * crouch_val + alpha * home_val;
        }
      }
      mju_sub(posture_residual + kPostureOrientDim, data->qpos + 7,
              target.data(), model->nu);

      if (launch_time < preload_time_) {
        // preload remains leg-only.
        for (int i = 0; i < 6; ++i) {
          posture_residual[kPostureOrientDim + i] = 0.0;
        }

        // Normalize by active leg dimensions only in preload.
        double norm_scale = 1.0 / std::sqrt(6.0);
        for (int i = 6; i < model->nu; ++i) {
          posture_residual[kPostureOrientDim + i] *= norm_scale;
        }
      }
    } else if (launch_time < pivot_end) {
      // pivot: interpolate home -> pivot posture target.
      double time_in_pivot = launch_time - rise_end;
      double alpha = time_in_pivot / pivot_time_;
      alpha = mju_min(1.0, mju_max(0.0, alpha));
      std::vector<double> target(model->nu);
      for (int i = 0; i < model->nu; ++i) {
        double home_val = home[7 + i];
        double pivot_val = pivot[7 + i];
        target[i] = (1.0 - alpha) * home_val + alpha * pivot_val;
      }
      mju_sub(posture_residual + kPostureOrientDim, data->qpos + 7,
              target.data(), model->nu);

      // Do not penalize shoulder2 joint positions in pivot.
      posture_residual[kPostureOrientDim + 1] = 0.0;
      posture_residual[kPostureOrientDim + 4] = 0.0;

      // Add pivot shoulder2 velocity targets from pivot keyframe qvel.
      int pivot_key_id = mj_name2id(model, mjOBJ_KEY, "pivot");
      if (pivot_key_id >= 0 && model->key_qvel) {
        const double* pivot_qvel = model->key_qvel + pivot_key_id * model->nv;
        const double qvel_gain = 0.25;
        const int shoulder2_dofs[2] = {1, 4};
        for (int i = 0; i < 2; ++i) {
          int dof = shoulder2_dofs[i];
          posture_residual[kPostureOrientDim + dof] += qvel_gain *
              (data->qvel[6 + dof] - pivot_qvel[6 + dof]);
        }
      }

      // Add torso orientation tracking only during pivot.
      double target_quat[4] = {pivot[3], pivot[4], pivot[5], pivot[6]};
      mju_normalize4(target_quat);
      double* torso_xquat = data->xquat + 4 * torso_body_id_;
      mju_subQuat(posture_residual, torso_xquat, target_quat);

      // Reduce posture contribution during pivot.
      const double kPivotPostureScale = 0.5;
      for (int i = 0; i < kPostureOrientDim + model->nu; ++i) {
        posture_residual[i] *= kPivotPostureScale;
      }
    } else {
      mju_zero(posture_residual + kPostureOrientDim, model->nu);
    }
  }
  for (A1Foot foot : kFootAll) {
    for (int joint = 0; joint < 3; joint++) {
      posture_residual[kPostureOrientDim + 3 * foot + joint] *=
          kJointPostureGain[joint];
    }
  }
  if (current_mode_ == kModeBiped) {
    // loosen the "hands" in Biped mode
    bool handstand = ReinterpretAsInt(parameters_[biped_type_param_id_]);
    double arm_posture = parameters_[arm_posture_param_id_];
    if (handstand) {
      posture_residual[kPostureOrientDim + 6] *= arm_posture;
      posture_residual[kPostureOrientDim + 7] *= arm_posture;
      posture_residual[kPostureOrientDim + 8] *= arm_posture;
      posture_residual[kPostureOrientDim + 9] *= arm_posture;
      posture_residual[kPostureOrientDim + 10] *= arm_posture;
      posture_residual[kPostureOrientDim + 11] *= arm_posture;
    } else {
      posture_residual[kPostureOrientDim + 0] *= arm_posture;
      posture_residual[kPostureOrientDim + 1] *= arm_posture;
      posture_residual[kPostureOrientDim + 2] *= arm_posture;
      posture_residual[kPostureOrientDim + 3] *= arm_posture;
      posture_residual[kPostureOrientDim + 4] *= arm_posture;
      posture_residual[kPostureOrientDim + 5] *= arm_posture;
    }
  }
  counter += kPostureOrientDim + model->nu;


  // ---------- Yaw ----------
  double torso_heading[3] = {torso_xmat[0], torso_xmat[3], torso_xmat[6]};
  if (current_mode_ == kModeBiped) {
    int handstand =
        ReinterpretAsInt(parameters_[biped_type_param_id_]) ? 1 : -1;
    torso_heading[0] = handstand * torso_xmat[2];
    torso_heading[1] = handstand * torso_xmat[5];
    torso_heading[2] = handstand * torso_xmat[8];
  }
  mju_normalize(torso_heading, 3);
  double heading_goal = parameters_[ParameterIndex(model, "Heading")];
  residual[counter++] = torso_heading[0] - mju_cos(heading_goal);
  residual[counter++] = torso_heading[1] - mju_sin(heading_goal);
  residual[counter++] = torso_heading[2];


  // ---------- Angular momentum ----------
  mju_copy3(residual + counter, SensorByName(model, data, "torso_angmom"));
  counter +=3;


  // ---------- Ground Contact ----------
  if (current_mode_ == kModeLaunch) {
    double launch_time = data->time - mode_start_time_;
    double* fr_touch = SensorByName(model, data, "FR_touch");
    double* fl_touch = SensorByName(model, data, "FL_touch");
    double* rr_touch = SensorByName(model, data, "RR_touch");
    double* rl_touch = SensorByName(model, data, "RL_touch");
    double rise_end = preload_time_ + rise_time_;
    double pivot_end = rise_end + pivot_time_;
    
    // penalize lack of contact: cost = max(0, threshold - touch_force)
    double contact_threshold = 0.05;
    auto touch_penalty = [contact_threshold](double* touch) {
      return mju_max(0.0, contact_threshold - touch[0]);
    };
    
    if (launch_time < preload_time_) {
      // preload: only front two feet must touch during crouch
      residual[counter++] = touch_penalty(fr_touch);
      residual[counter++] = touch_penalty(fl_touch);
      residual[counter++] = 0.0;
      residual[counter++] = 0.0;
    } else if (launch_time < rise_end) {
      // rise: keep all four feet loaded while extending upward
      // front two: penalize lack of contact (minimum threshold)
      // rear two: same penalty style as front feet
      residual[counter++] = touch_penalty(fr_touch);
      residual[counter++] = touch_penalty(fl_touch);
      residual[counter++] = touch_penalty(rr_touch);
      residual[counter++] = touch_penalty(rl_touch);
    } else if (launch_time < pivot_end) {
      // pivot: front feet maintain contact, rear feet unload
      residual[counter++] = touch_penalty(fr_touch);
      residual[counter++] = touch_penalty(fl_touch);
      residual[counter++] = 0.0;
      residual[counter++] = 0.0;
    } else if (launch_time < pivot_end + jump_time_) {
      // jump: front two early, then disable
      double jump_elapsed = launch_time - pivot_end;
      bool jump_second_half = jump_elapsed > 0.5 * jump_time_;
      if (jump_second_half) {
        residual[counter++] = 0.0;
        residual[counter++] = 0.0;
        residual[counter++] = 0.0;
        residual[counter++] = 0.0;
      } else {
        residual[counter++] = touch_penalty(fr_touch);
        residual[counter++] = touch_penalty(fl_touch);
        residual[counter++] = 0.0;
        residual[counter++] = 0.0;
      }
    } else if (launch_time < pivot_end + jump_time_ + flight_time_) {
      // flight: no ground contact requirement
      residual[counter++] = 0.0;
      residual[counter++] = 0.0;
      residual[counter++] = 0.0;
      residual[counter++] = 0.0;
    } else {
      // land: front two early, then all four
      double land_elapsed = launch_time - pivot_end - jump_time_ - flight_time_;
      bool land_second_half = land_elapsed > 0.5 * land_time_;
      residual[counter++] = touch_penalty(fr_touch);
      residual[counter++] = touch_penalty(fl_touch);
      residual[counter++] = land_second_half ? touch_penalty(rr_touch) : 0.0;
      residual[counter++] = land_second_half ? touch_penalty(rl_touch) : 0.0;
    }
  } else if (current_mode_ == kModeLaunchTrack && ref_key_start_ >= 0) {
    // require contact wherever the reference has it
    double ref_time = RefTime(data->time - mode_start_time_);
    bool hands_contact, feet_contact;
    LaunchReference(model, ref_time, nullptr, nullptr, &hands_contact,
                    &feet_contact);
    double contact_threshold = 0.05;
    auto touch_penalty = [&](const char* name, bool active) {
      double* touch = SensorByName(model, data, name);
      return active ? mju_max(0.0, contact_threshold - touch[0]) : 0.0;
    };
    residual[counter++] = touch_penalty("FR_touch", hands_contact);
    residual[counter++] = touch_penalty("FL_touch", hands_contact);
    residual[counter++] = touch_penalty("RR_touch", feet_contact);
    residual[counter++] = touch_penalty("RL_touch", feet_contact);
  } else {
    residual[counter++] = 0.0;
    residual[counter++] = 0.0;
    residual[counter++] = 0.0;
    residual[counter++] = 0.0;
  }


  // ---------- Launch Track reference ----------
  // base pos (3), base orientation (3), joint pos (nu), base vel (6),
  // joint vel (nu), takeoff CoM velocity and angular momentum (6),
  // symmetry (12), clearance (6)
  int ref_dim = 3 + 3 + model->nu + 6 + model->nu + 6 + 12 + 6;
  if (current_mode_ == kModeLaunchTrack && ref_key_start_ >= 0) {
    double launch_time = data->time - mode_start_time_;
    double ref_time = RefTime(launch_time);
    std::vector<double> ref_qpos(model->nq), ref_qvel(model->nv);
    LaunchReference(model, ref_time, ref_qpos.data(), ref_qvel.data(),
                    nullptr, nullptr);
    // reference velocities scale with the time compression
    mju_scl(ref_qvel.data(), ref_qvel.data(), RefRate(launch_time), model->nv);

    // push: from the deepest crouch to takeoff the reference base motion is
    // replaced by a takeoff velocity target, so the launch speed comes from
    // the push rather than the reference. after takeoff the reference base
    // motion is not ballistic (it assumes aerodynamic forces): track only
    // posture and trunk orientation.
    bool airborne = ref_time >= ref_takeoff_time_;
    bool push = !airborne && ref_time >= ref_push_start_;

    // base position
    if (airborne) {
      mju_zero(residual + counter, 3);
    } else {
      mju_sub3(residual + counter, data->qpos, ref_qpos.data());
      if (push) {
        mju_scl3(residual + counter, residual + counter, kRefPushBaseScale);
      }
    }
    counter += 3;

    // base orientation
    mju_subQuat(residual + counter, data->qpos + 3, ref_qpos.data() + 3);
    counter += 3;

    // joint position; hold the legs firmly once the feet have lifted off
    // so they do not flail while the arms vault
    mju_sub(residual + counter, data->qpos + 7, ref_qpos.data() + 7,
            model->nu);
    bool hands_contact, feet_contact;
    LaunchReference(model, ref_time, nullptr, nullptr, &hands_contact,
                    &feet_contact);
    if (!feet_contact) {
      mju_scl(residual + counter + 6, residual + counter + 6,
              kRefLegLiftoffScale, 6);
    }
    if (airborne) {
      mju_scl(residual + counter, residual + counter, kRefFlightJointScale,
              model->nu);
    }
    counter += model->nu;

    // base linear and angular velocity
    mju_sub(residual + counter, data->qvel, ref_qvel.data(), 6);
    if (airborne || push) {
      mju_zero(residual + counter, 3);
    }
    counter += 6;

    // joint velocity
    mju_sub(residual + counter, data->qvel + 6, ref_qvel.data() + 6,
            model->nu);
    counter += model->nu;

    // takeoff: CoM velocity ramps to the launch velocity at takeoff time,
    // along the reference heading, with no whole-body spin
    if (push) {
      double alpha = (ref_time - ref_push_start_) /
                     (ref_takeoff_time_ - ref_push_start_);
      double speed = alpha * parameters_[launch_speed_param_id_];
      double angle = parameters_[launch_angle_param_id_] * mjPI / 180;
      double direction[3] = {-mju_cos(angle), 0, mju_sin(angle)};
      double target[3];
      mju_rotVecQuat(target, direction, ref_yaw_quat_);
      mju_scl3(target, target, speed);
      mju_sub3(residual + counter, comvel, target);
      // angular momentum about the CoM, normalized by mass
      double* angmom = SensorByName(model, data, "torso_subtreeangmom");
      double mass = model->body_subtreemass[torso_body_id_];
      mju_scl3(residual + counter + 3, angmom, kRefSpinScale / mass);
    } else {
      mju_zero(residual + counter, 6);
    }
    counter += 6;

    // symmetry: left limbs mirror right limbs (joint position, velocity),
    // so both arms and both legs push together
    for (int limb = 0; limb < 2; limb++) {
      const double* sign = limb == 0 ? kMirrorSign : kMirrorSignLeg;
      int left = 6 * limb, right = 6 * limb + 3;
      for (int j = 0; j < 3; j++) {
        residual[counter + 3*limb + j] = data->qpos[7 + left + j] -
                                         sign[j] * data->qpos[7 + right + j];
        residual[counter + 6 + 3*limb + j] =
            0.1 * (data->qvel[6 + left + j] - sign[j] * data->qvel[6 + right + j]);
      }
    }
    counter += 12;

    // clearance: knees, shins and forearms stay off the ground, the robot
    // pushes on its feet
    struct {
      int body;
      double point[3];
      double clearance;
    } points[6] = {
        {tibia_body_id_[0], {0, 0, 0}, kClearKnee},
        {tibia_body_id_[1], {0, 0, 0}, kClearKnee},
        {tibia_body_id_[0], {0.184, 0, 0}, kClearShin},
        {tibia_body_id_[1], {0.184, 0, 0}, kClearShin},
        {forearm_body_id_[0], {-0.34, 0, -0.01}, kClearForearm},
        {forearm_body_id_[1], {-0.34, 0, -0.01}, kClearForearm},
    };
    for (const auto& p : points) {
      double pos[3];
      mju_mulMatVec3(pos, data->xmat + 9*p.body, p.point);
      mju_addTo3(pos, data->xpos + 3*p.body);
      double height = pos[2] - Ground(model, data, pos);
      residual[counter++] = mju_max(0, p.clearance - height);
    }
  } else {
    mju_zero(residual + counter, ref_dim);
    counter += ref_dim;
  }


  // sensor dim sanity check
  CheckSensorDim(model, counter);
}

//  ============  transition  ============
void Pterosaur::TransitionLocked(mjModel* model, mjData* data) {
  // ---------- handle mjData reset ----------
  if (data->time < residual_.last_transition_time_ ||
      residual_.last_transition_time_ == -1) {
    if (mode != ResidualFn::kModeQuadruped && mode != ResidualFn::kModeBiped) {
      mode = ResidualFn::kModeQuadruped;  // mode stateful, switch to Quadruped
    }
    residual_.last_transition_time_ = residual_.phase_start_time_ =
        residual_.phase_start_ = data->time;
  }

  // ---------- prevent forbidden mode transitions ----------
  // switching mode, not from quadruped
  if (mode != residual_.current_mode_ &&
      residual_.current_mode_ != ResidualFn::kModeQuadruped) {
    // switch into stateful mode only allowed from Quadruped
    if (mode == ResidualFn::kModeWalk || mode == ResidualFn::kModeFlip ||
        mode == ResidualFn::kModeLaunch ||
        mode == ResidualFn::kModeLaunchTrack) {
      mode = ResidualFn::kModeQuadruped;
    }
  }

  // ---------- handle phase velocity change ----------
  double phase_velocity = 2 * mjPI * parameters[residual_.cadence_param_id_];
  if (phase_velocity != residual_.phase_velocity_) {
    residual_.phase_start_ = residual_.GetPhase(data->time);
    residual_.phase_start_time_ = data->time;
    residual_.phase_velocity_ = phase_velocity;
  }


  // ---------- automatic gait switching ----------
  double* comvel = SensorByName(model, data, "torso_subtreelinvel");
  double beta = mju_exp(-(data->time - residual_.last_transition_time_) /
                        ResidualFn::kAutoGaitFilter);
  residual_.com_vel_[0] = beta * residual_.com_vel_[0] + (1 - beta) * comvel[0];
  residual_.com_vel_[1] = beta * residual_.com_vel_[1] + (1 - beta) * comvel[1];
  // TODO(b/268398978): remove reinterpret, int64_t business
  int auto_switch =
      ReinterpretAsInt(parameters[residual_.gait_switch_param_id_]);
  if (mode == ResidualFn::kModeBiped) {
    // biped always trots
    parameters[residual_.gait_param_id_] =
        ReinterpretAsDouble(ResidualFn::kGaitTrot);
  } else if (auto_switch) {
    double com_speed = mju_norm(residual_.com_vel_, 2);
    for (int64_t gait : ResidualFn::kGaitAll) {
      // scramble requires a non-static gait
      if (mode == ResidualFn::kModeScramble && gait == ResidualFn::kGaitStand)
        continue;
      bool lower = com_speed > ResidualFn::kGaitAuto[gait];
      bool upper = gait == ResidualFn::kGaitGallop ||
                   com_speed <= ResidualFn::kGaitAuto[gait + 1];
      bool wait = mju_abs(residual_.gait_switch_time_ - data->time) >
                  ResidualFn::kAutoGaitMinTime;
      if (lower && upper && wait) {
        parameters[residual_.gait_param_id_] = ReinterpretAsDouble(gait);
        residual_.gait_switch_time_ = data->time;
      }
    }
  }


  // ---------- handle gait switch, manual or auto ----------
  double gait_selection = parameters[residual_.gait_param_id_];
  if (gait_selection != residual_.current_gait_) {
    residual_.current_gait_ = gait_selection;
    ResidualFn::A1Gait gait = residual_.GetGait();
    parameters[residual_.duty_param_id_] = ResidualFn::kGaitParam[gait][0];
    parameters[residual_.cadence_param_id_] = ResidualFn::kGaitParam[gait][1];
    parameters[residual_.amplitude_param_id_] = ResidualFn::kGaitParam[gait][2];
    weight[residual_.balance_cost_id_] = ResidualFn::kGaitParam[gait][3];
    weight[residual_.upright_cost_id_] = ResidualFn::kGaitParam[gait][4];
    weight[residual_.height_cost_id_] = ResidualFn::kGaitParam[gait][5];
  }


  // ---------- Walk ----------
  double* goal_pos = data->mocap_pos + 3*residual_.goal_mocap_id_;
  if (mode == ResidualFn::kModeWalk) {
    double angvel = parameters[ParameterIndex(model, "Walk turn")];
    double speed = parameters[ParameterIndex(model, "Walk speed")];

    // current torso direction
    double* torso_xmat = data->xmat + 9*residual_.torso_body_id_;
    double forward[2] = {torso_xmat[0], torso_xmat[3]};
    mju_normalize(forward, 2);
    double leftward[2] = {-forward[1], forward[0]};

    // switching into Walk or parameters changed, reset task state
    if (mode != residual_.current_mode_ || residual_.angvel_ != angvel ||
        residual_.speed_ != speed) {
      // save time
      residual_.mode_start_time_ = data->time;

      // save current speed and angvel
      residual_.speed_ = speed;
      residual_.angvel_ = angvel;

      // compute and save rotation axis / walk origin
      double axis[2] = {data->xpos[3*residual_.torso_body_id_],
                        data->xpos[3*residual_.torso_body_id_+1]};
      if (mju_abs(angvel) > ResidualFn::kMinAngvel) {
        // don't allow turning with very small angvel
        double d = speed / angvel;
        axis[0] += d * leftward[0];
        axis[1] += d * leftward[1];
      }
      residual_.position_[0] = axis[0];
      residual_.position_[1] = axis[1];

      // save vector from axis to initial goal position
      residual_.heading_[0] = goal_pos[0] - axis[0];
      residual_.heading_[1] = goal_pos[1] - axis[1];
    }

    // move goal
    double time = data->time - residual_.mode_start_time_;
    residual_.Walk(goal_pos, time);
  }


  // ---------- Flip ----------
  double* compos = SensorByName(model, data, "torso_subtreecom");

  if (mode == ResidualFn::kModeFlip) {
    // switching into Flip, reset task state
    if (mode != residual_.current_mode_) {
      // save time
      residual_.mode_start_time_ = data->time;

      // save body orientation, ground height
      mju_copy4(residual_.orientation_,
                data->xquat + 4 * residual_.torso_body_id_);
      residual_.ground_ = Ground(model, data, compos);

      // save parameters
      residual_.save_weight_ = weight;
      residual_.save_gait_switch_ = parameters[residual_.gait_switch_param_id_];

      // set parameters
      weight[CostTermByName(model, "Upright")] = 0.2;
      weight[CostTermByName(model, "Height")] = 5;
      weight[CostTermByName(model, "Position")] = 0;
      weight[CostTermByName(model, "Gait")] = 0;
      weight[CostTermByName(model, "Balance")] = 0;
      weight[CostTermByName(model, "Effort")] = 0.005;
      weight[CostTermByName(model, "Posture")] = 0.1;
      parameters[residual_.gait_switch_param_id_] = ReinterpretAsDouble(0);
    }

    // time from start of Flip
    double flip_time = data->time - residual_.mode_start_time_;

    if (flip_time >=
        residual_.jump_time_ + residual_.flight_time_ + residual_.land_time_) {
      // Flip ended, back to Quadruped, restore values
      mode = ResidualFn::kModeQuadruped;
      weight = residual_.save_weight_;
      parameters[residual_.gait_switch_param_id_] = residual_.save_gait_switch_;
      goal_pos[0] = data->site_xpos[3*residual_.head_site_id_ + 0];
      goal_pos[1] = data->site_xpos[3*residual_.head_site_id_ + 1];
    }
  }
    
  
  // ---------- Launch ----------
  if (mode == ResidualFn::kModeLaunch) {
    // switching into Launch, reset task state
    if (mode != residual_.current_mode_) {
      // save time
      residual_.mode_start_time_ = data->time;

      // save body orientation, ground height
      mju_copy4(residual_.orientation_,
                data->xquat + 4 * residual_.torso_body_id_);
      residual_.ground_ = Ground(model, data, compos);

      // save launch origin for horizontal target
      double* head = data->site_xpos + 3*residual_.head_site_id_;
      residual_.position_[0] = head[0];
      residual_.position_[1] = head[1];
      residual_.position_[2] = head[2];

      // save parameters
      residual_.save_weight_ = weight;
      residual_.save_gait_switch_ = parameters[residual_.gait_switch_param_id_];

      // set launch phase timings
      residual_.preload_time_ = ResidualFn::kLaunchPreloadTime;
        residual_.rise_time_ = ResidualFn::kLaunchRiseTime;
      residual_.pivot_time_ = ResidualFn::kLaunchPivotTime;
      residual_.jump_time_ = ResidualFn::kLaunchPushTime;
      residual_.flight_time_ = 2 * ResidualFn::kLaunchSpeed *
          mju_sin(ResidualFn::kLaunchAngle) / residual_.gravity_;
      residual_.land_time_ = ResidualFn::kLaunchLandTime;

      // set velocities used in cost targets
      residual_.jump_vel_ = ResidualFn::kLaunchSpeed *
          mju_sin(ResidualFn::kLaunchAngle);

      // set weights for launch
      weight[CostTermByName(model, "Upright")] = 0.2;
      weight[CostTermByName(model, "Height")] = 1.0;
      weight[CostTermByName(model, "Position")] = 0.4;
      weight[CostTermByName(model, "LaunchVelocity")] = 1.0;
      weight[CostTermByName(model, "Gait")] = 0;
      weight[CostTermByName(model, "Balance")] = 0.6;
      weight[CostTermByName(model, "Effort")] = 0.005;
      weight[CostTermByName(model, "Posture")] = 0.1;
      parameters[residual_.gait_switch_param_id_] = ReinterpretAsDouble(0);
    }

    // time from start of Launch
    double launch_time = data->time - residual_.mode_start_time_;
    double rise_end = residual_.preload_time_ + residual_.rise_time_;
    double pivot_end = rise_end + residual_.pivot_time_;
    double launch_duration = pivot_end + residual_.jump_time_ +
      residual_.flight_time_ + residual_.land_time_;

    // adjust weights per phase
    if (launch_time < residual_.preload_time_) {
      // preload: disable launch velocity and posture, increase balance and ground contact
      weight[CostTermByName(model, "Balance")] = 0.0;
      weight[CostTermByName(model, "GroundContact")] = 3.0;
      weight[CostTermByName(model, "LaunchVelocity")] = 0.0;
      weight[CostTermByName(model, "Posture")] = 0.055;
    } else if (launch_time < rise_end) {
      // rise: keep strong balance/contact while still suppressing launch velocity
      weight[CostTermByName(model, "Balance")] = 1.0;
      weight[CostTermByName(model, "GroundContact")] = 3.6;
      weight[CostTermByName(model, "LaunchVelocity")] = 0.0;
      weight[CostTermByName(model, "Posture")] = 0.05;
    } else if (launch_time < pivot_end) {
      // pivot: shift toward launch behavior, balance and launch velocity off
      weight[CostTermByName(model, "Balance")] = 0;
      weight[CostTermByName(model, "GroundContact")] = 0.8;
      weight[CostTermByName(model, "LaunchVelocity")] = 0;
      weight[CostTermByName(model, "Posture")] = 3.0;
    } else if (launch_time < pivot_end + residual_.jump_time_) {
      // jump/push: effort only, everything else off
      weight[CostTermByName(model, "Upright")] = 0;
      weight[CostTermByName(model, "Height")] = 0;
      weight[CostTermByName(model, "Position")] = 0;
      weight[CostTermByName(model, "LaunchVelocity")] = 0;
      weight[CostTermByName(model, "Balance")] = 0;
      weight[CostTermByName(model, "GroundContact")] = 0;
      weight[CostTermByName(model, "Posture")] = 0;
    } else if (launch_time < pivot_end + residual_.jump_time_ +
                              residual_.flight_time_) {
      // flight: effort only, everything else off
      weight[CostTermByName(model, "Upright")] = 0;
      weight[CostTermByName(model, "Height")] = 0;
      weight[CostTermByName(model, "Position")] = 0;
      weight[CostTermByName(model, "LaunchVelocity")] = 0;
      weight[CostTermByName(model, "Balance")] = 0;
      weight[CostTermByName(model, "GroundContact")] = 0;
      weight[CostTermByName(model, "Posture")] = 0;
    } else {
      // land: effort only, everything else off
      weight[CostTermByName(model, "Upright")] = 0;
      weight[CostTermByName(model, "Height")] = 0;
      weight[CostTermByName(model, "Position")] = 0;
      weight[CostTermByName(model, "LaunchVelocity")] = 0;
      weight[CostTermByName(model, "Balance")] = 0;
      weight[CostTermByName(model, "GroundContact")] = 0;
      weight[CostTermByName(model, "Posture")] = 0;
    }

    if (launch_time >= launch_duration) {
      // Launch ended, back to Quadruped, restore values
      mode = ResidualFn::kModeQuadruped;
      weight = residual_.save_weight_;
      parameters[residual_.gait_switch_param_id_] = residual_.save_gait_switch_;
      goal_pos[0] = data->site_xpos[3*residual_.head_site_id_ + 0];
      goal_pos[1] = data->site_xpos[3*residual_.head_site_id_ + 1];
    }
  }

  // ---------- Launch Track ----------
  if (mode == ResidualFn::kModeLaunchTrack && residual_.ref_key_start_ < 0) {
    // no reference loaded
    mode = ResidualFn::kModeQuadruped;
  }
  if (mode == ResidualFn::kModeLaunchTrack) {
    // switching into Launch Track, reset task state
    if (mode != residual_.current_mode_) {
      // start the reference at "Ref start", e.g. the deepest crouch
      double ref_start = mju_clip(
          parameters[residual_.ref_start_param_id_], 0,
          (residual_.ref_num_frames_ - 1) * residual_.ref_dt_);
      residual_.mode_start_time_ = data->time - ref_start;

      // align reference frame 0 with the robot: keep its horizontal position
      // and heading, shift height by the ground offset
      const double* ref_qpos0 =
          model->key_qpos + model->nq * residual_.ref_key_start_;
      mju_copy3(residual_.ref_origin_, ref_qpos0);
      double* torso_xmat = data->xmat + 9*residual_.torso_body_id_;
      double yaw = mju_atan2(torso_xmat[3], torso_xmat[0]);
      double z_axis[3] = {0, 0, 1};
      mju_axisAngle2Quat(residual_.ref_yaw_quat_, z_axis, yaw);
      residual_.ground_ = Ground(model, data, compos);
      residual_.ref_offset_[0] = data->qpos[0];
      residual_.ref_offset_[1] = data->qpos[1];
      residual_.ref_offset_[2] = ref_qpos0[2] + residual_.ground_ -
                                 ResidualFn::kRefGroundHeight;

      // snap state to the reference at ref_start
      // (no mj_forward here: the sensor callback would re-lock the task)
      residual_.LaunchReference(model, ref_start, data->qpos, data->qvel,
                                nullptr, nullptr);

      // save parameters
      residual_.save_weight_ = weight;
      residual_.save_gait_switch_ = parameters[residual_.gait_switch_param_id_];

      // set weights for reference tracking
      weight[CostTermByName(model, "Upright")] = 0;
      weight[CostTermByName(model, "Height")] = 0;
      weight[CostTermByName(model, "Position")] = 0;
      weight[CostTermByName(model, "LaunchVelocity")] = 0;
      weight[CostTermByName(model, "Gait")] = 0;
      weight[CostTermByName(model, "Balance")] = 0;
      weight[CostTermByName(model, "Effort")] = 0.005;
      weight[CostTermByName(model, "Posture")] = 0;
      weight[CostTermByName(model, "Orientation")] = 0;
      weight[CostTermByName(model, "Angmom")] = 0;
      weight[CostTermByName(model, "GroundContact")] = 1;
      weight[residual_.ref_base_pos_cost_id_] = 5;
      weight[residual_.ref_base_ori_cost_id_] = 3;
      weight[residual_.ref_joint_pos_cost_id_] = 2;
      weight[residual_.ref_base_vel_cost_id_] = 0.5;
      weight[residual_.ref_joint_vel_cost_id_] = 0.05;
      weight[residual_.ref_takeoff_cost_id_] = 2;
      weight[residual_.symmetry_cost_id_] = 5;
      weight[residual_.clearance_cost_id_] = 20;
      parameters[residual_.gait_switch_param_id_] = ReinterpretAsDouble(0);
    }

    // time from start of Launch Track
    double ref_time =
        residual_.RefTime(data->time - residual_.mode_start_time_);
    double duration =
        (residual_.ref_num_frames_ - 1) * residual_.ref_dt_;

    if (ref_time >= duration) {
      // reference ended, back to Quadruped, restore values
      mode = ResidualFn::kModeQuadruped;
      weight = residual_.save_weight_;
      parameters[residual_.gait_switch_param_id_] = residual_.save_gait_switch_;
      goal_pos[0] = data->site_xpos[3*residual_.head_site_id_ + 0];
      goal_pos[1] = data->site_xpos[3*residual_.head_site_id_ + 1];
    }
  }

  // save mode
  residual_.current_mode_ = static_cast<ResidualFn::A1Mode>(mode);
  residual_.last_transition_time_ = data->time;
}

// colors of visualisation elements drawn in ModifyScene()
constexpr float kStepRgba[4] = {0.6, 0.8, 0.2, 1};  // step-height cylinders
constexpr float kHullRgba[4] = {0.4, 0.2, 0.8, 1};  // convex hull
constexpr float kAvgRgba[4] = {0.4, 0.2, 0.8, 1};   // average foot position
constexpr float kCapRgba[4] = {0.3, 0.3, 0.8, 1};   // capture point
constexpr float kPcpRgba[4] = {0.5, 0.5, 0.2, 1};   // projected capture point

// draw task-related geometry in the scene
void Pterosaur::ModifyScene(const mjModel* model, const mjData* data,
                           mjvScene* scene) const {
  // flip target pose
  if (residual_.current_mode_ == ResidualFn::kModeFlip) {
    double flip_time = data->time - residual_.mode_start_time_;
    double* torso_pos = data->xpos + 3*residual_.torso_body_id_;
    double pos[3] = {torso_pos[0], torso_pos[1],
                     residual_.FlipHeight(flip_time)};
    double quat[4];
    residual_.FlipQuat(quat, flip_time);
    double mat[9];
    mju_quat2Mat(mat, quat);
    double size[3] = {0.25, 0.15, 0.05};
    float rgba[4] = {0, 1, 0, 0.5};
    AddGeom(scene, mjGEOM_BOX, size, pos, mat, rgba);

    // don't draw anything else during flip
    return;
  }

  // launch track reference base pose
  if (residual_.current_mode_ == ResidualFn::kModeLaunchTrack &&
      residual_.ref_key_start_ >= 0) {
    double ref_time =
        residual_.RefTime(data->time - residual_.mode_start_time_);
    std::vector<double> ref_qpos(model->nq);
    residual_.LaunchReference(model, ref_time, ref_qpos.data(), nullptr,
                              nullptr, nullptr);
    double mat[9];
    mju_quat2Mat(mat, ref_qpos.data() + 3);
    double size[3] = {0.25, 0.15, 0.05};
    float rgba[4] = {0, 1, 0, 0.5};
    AddGeom(scene, mjGEOM_BOX, size, ref_qpos.data(), mat, rgba);

    // don't draw gait visuals during launch track
    return;
  }

  // current foot positions
  double* foot_pos[ResidualFn::kNumFoot];
  for (ResidualFn::A1Foot foot : ResidualFn::kFootAll)
    foot_pos[foot] = data->geom_xpos + 3 * residual_.foot_geom_id_[foot];

  // stance and flight positions
  double flight_pos[ResidualFn::kNumFoot][3];
  double stance_pos[ResidualFn::kNumFoot][3];
  // set to foot horizontal position:
  for (ResidualFn::A1Foot foot : ResidualFn::kFootAll) {
    flight_pos[foot][0] = stance_pos[foot][0] = foot_pos[foot][0];
    flight_pos[foot][1] = stance_pos[foot][1] = foot_pos[foot][1];
  }

  // ground height below feet
  double ground[ResidualFn::kNumFoot];
  for (ResidualFn::A1Foot foot : ResidualFn::kFootAll) {
    ground[foot] = Ground(model, data, foot_pos[foot]);
  }

  // step heights
  ResidualFn::A1Gait gait = residual_.GetGait();
  double step[ResidualFn::kNumFoot];
  residual_.FootStep(step, residual_.GetPhase(data->time), gait);

  // draw step height
  for (ResidualFn::A1Foot foot : ResidualFn::kFootAll) {
    stance_pos[foot][2] = ResidualFn::kFootRadius + ground[foot];
    if (residual_.current_mode_ == ResidualFn::kModeBiped) {
      // skip "hands" in biped mode
      bool handstand =
          ReinterpretAsInt(parameters[residual_.biped_type_param_id_]);
      bool front_hand = !handstand && (foot == ResidualFn::kFootFL ||
                                       foot == ResidualFn::kFootFR);
      bool back_hand = handstand && (foot == ResidualFn::kFootHL ||
                                     foot == ResidualFn::kFootHR);
      if (front_hand || back_hand) continue;
    }
    if (step[foot]) {
      flight_pos[foot][2] = ResidualFn::kFootRadius + step[foot] + ground[foot];
      AddConnector(scene, mjGEOM_CYLINDER, ResidualFn::kFootRadius,
                   stance_pos[foot], flight_pos[foot], kStepRgba);
    }
  }

  // support polygon (currently unused for cost)
  double polygon[2*ResidualFn::kNumFoot];
  for (ResidualFn::A1Foot foot : ResidualFn::kFootAll) {
    polygon[2*foot] = foot_pos[foot][0];
    polygon[2*foot + 1] = foot_pos[foot][1];
  }
  int hull[ResidualFn::kNumFoot];
  int num_hull = Hull2D(hull, ResidualFn::kNumFoot, polygon);
  for (int i=0; i < num_hull; i++) {
    int j = (i + 1) % num_hull;
    AddConnector(scene, mjGEOM_CAPSULE, ResidualFn::kFootRadius/2,
                 stance_pos[hull[i]], stance_pos[hull[j]], kHullRgba);
  }

  // capture point
  bool is_biped = residual_.current_mode_ == ResidualFn::kModeBiped;
  double height_goal =
      is_biped ? ResidualFn::kHeightBiped : ResidualFn::kHeightQuadruped;
  double fall_time = mju_sqrt(2*height_goal / residual_.gravity_);
  double capture[3];
  double* compos = SensorByName(model, data, "torso_subtreecom");
  double* comvel = SensorByName(model, data, "torso_subtreelinvel");
  mju_addScl3(capture, compos, comvel, fall_time);

  // ground under CoM
  double com_ground = Ground(model, data, compos);

  // average foot position
  double feet_pos[3];
  residual_.AverageFootPos(feet_pos, foot_pos);
  feet_pos[2] = com_ground;

  double foot_size[3] = {ResidualFn::kFootRadius, 0, 0};

  // average foot position
  AddGeom(scene, mjGEOM_SPHERE, foot_size, feet_pos, /*mat=*/nullptr, kAvgRgba);

  // capture point
  capture[2] = com_ground;
  AddGeom(scene, mjGEOM_SPHERE, foot_size, capture, /*mat=*/nullptr, kCapRgba);

  // capture point, projected onto hull
  double pcp2[2];
  NearestInHull(pcp2, capture, polygon, hull, num_hull);
  double pcp[3] = {pcp2[0], pcp2[1], com_ground};
  AddGeom(scene, mjGEOM_SPHERE, foot_size, pcp, /*mat=*/nullptr, kPcpRgba);
}

//  ============  task-state utilities  ============
// save task-related ids
void Pterosaur::ResetLocked(const mjModel* model) {
  // ----------  task identifiers  ----------
  residual_.gait_param_id_ = ParameterIndex(model, "select_Gait");
  residual_.gait_switch_param_id_ = ParameterIndex(model, "select_Gait switch");
  residual_.flip_dir_param_id_ = ParameterIndex(model, "select_Flip dir");
  residual_.biped_type_param_id_ = ParameterIndex(model, "select_Biped type");
  residual_.cadence_param_id_ = ParameterIndex(model, "Cadence");
  residual_.amplitude_param_id_ = ParameterIndex(model, "Amplitude");
  residual_.duty_param_id_ = ParameterIndex(model, "Duty ratio");
  residual_.arm_posture_param_id_ = ParameterIndex(model, "Arm posture");
  residual_.balance_cost_id_ = CostTermByName(model, "Balance");
  residual_.upright_cost_id_ = CostTermByName(model, "Upright");
  residual_.height_cost_id_ = CostTermByName(model, "Height");
  residual_.launch_velocity_cost_id_ = CostTermByName(model, "LaunchVelocity");
  residual_.ref_base_pos_cost_id_ = CostTermByName(model, "RefBasePos");
  residual_.ref_base_ori_cost_id_ = CostTermByName(model, "RefBaseOri");
  residual_.ref_joint_pos_cost_id_ = CostTermByName(model, "RefJointPos");
  residual_.ref_base_vel_cost_id_ = CostTermByName(model, "RefBaseVel");
  residual_.ref_joint_vel_cost_id_ = CostTermByName(model, "RefJointVel");
  residual_.ref_takeoff_cost_id_ = CostTermByName(model, "RefTakeoff");
  residual_.symmetry_cost_id_ = CostTermByName(model, "Symmetry");
  residual_.clearance_cost_id_ = CostTermByName(model, "Clearance");
  const char* tibias[2] = {"tibia", "tibia_2"};
  const char* forearms[2] = {"radius_and_ulna", "radius_and_ulna_2"};
  for (int i = 0; i < 2; i++) {
    residual_.tibia_body_id_[i] = mj_name2id(model, mjOBJ_BODY, tibias[i]);
    residual_.forearm_body_id_[i] = mj_name2id(model, mjOBJ_BODY, forearms[i]);
    if (residual_.tibia_body_id_[i] < 0 || residual_.forearm_body_id_[i] < 0) {
      mju_error("pterosaur: tibia/forearm body not found");
    }
  }
  residual_.launch_speed_param_id_ = ParameterIndex(model, "Launch speed");
  residual_.launch_angle_param_id_ = ParameterIndex(model, "Launch angle");
  residual_.ref_start_param_id_ = ParameterIndex(model, "Ref start");
  residual_.push_time_param_id_ = ParameterIndex(model, "Push time");

  // ----------  launch track reference  ----------
  // keyframes launch_ref_000, launch_ref_001, ... from launch_reference.xml
  residual_.ref_key_start_ = mj_name2id(model, mjOBJ_KEY, "launch_ref_000");
  residual_.ref_num_frames_ = 0;
  if (residual_.ref_key_start_ >= 0) {
    char name[32];
    while (true) {
      std::snprintf(name, sizeof(name), "launch_ref_%03d",
                    residual_.ref_num_frames_);
      if (mj_name2id(model, mjOBJ_KEY, name) !=
          residual_.ref_key_start_ + residual_.ref_num_frames_) {
        break;
      }
      residual_.ref_num_frames_++;
    }
  }
  int hands_id = mj_name2id(model, mjOBJ_NUMERIC, "launch_ref_hands_contact");
  int feet_id = mj_name2id(model, mjOBJ_NUMERIC, "launch_ref_feet_contact");
  double* takeoff = GetCustomNumericData(model, "launch_ref_takeoff");
  if (residual_.ref_num_frames_ < 2 || hands_id < 0 || feet_id < 0 ||
      !takeoff ||
      model->numeric_size[hands_id] != residual_.ref_num_frames_ ||
      model->numeric_size[feet_id] != residual_.ref_num_frames_) {
    // reference missing or malformed: Launch Track disabled
    residual_.ref_key_start_ = -1;
  } else {
    residual_.ref_dt_ =
        model->key_time[residual_.ref_key_start_ + 1] -
        model->key_time[residual_.ref_key_start_];
    residual_.ref_takeoff_time_ = takeoff[0];

    // push starts at the deepest crouch (lowest base) before takeoff
    residual_.ref_push_start_ = 0;
    double lowest = mjMAXVAL;
    for (int i = 0; i < residual_.ref_num_frames_; i++) {
      double t = i * residual_.ref_dt_;
      double z = model->key_qpos[model->nq*(residual_.ref_key_start_ + i) + 2];
      if (t < residual_.ref_takeoff_time_ && z < lowest) {
        lowest = z;
        residual_.ref_push_start_ = t;
      }
    }
    residual_.ref_hands_contact_adr_ = model->numeric_adr[hands_id];
    residual_.ref_feet_contact_adr_ = model->numeric_adr[feet_id];
  }

  // ----------  model identifiers  ----------
  residual_.torso_body_id_ = mj_name2id(model, mjOBJ_XBODY, "body");
  if (residual_.torso_body_id_ < 0) mju_error("body 'trunk' not found");

  residual_.head_site_id_ = mj_name2id(model, mjOBJ_SITE, "head");
  if (residual_.head_site_id_ < 0) mju_error("site 'head' not found");

  int goal_id = mj_name2id(model, mjOBJ_XBODY, "goal");
  if (goal_id < 0) mju_error("body 'goal' not found");

  residual_.goal_mocap_id_ = model->body_mocapid[goal_id];
  if (residual_.goal_mocap_id_ < 0) mju_error("body 'goal' is not mocap");

  // foot geom ids
  int foot_index = 0;
  for (const char* footname : {"FL", "HL", "FR", "HR"}) {
    int foot_id = mj_name2id(model, mjOBJ_GEOM, footname);
    if (foot_id < 0) mju_error_s("geom '%s' not found", footname);
    residual_.foot_geom_id_[foot_index] = foot_id;
    foot_index++;
  }

  // shoulder body ids
  int shoulder_index = 0;
  for (const char* shouldername : {"smaller_housing", "leg_motor_1", "smaller_housing_2", "hip"}) {
    int foot_id = mj_name2id(model, mjOBJ_BODY, shouldername);
    if (foot_id < 0) mju_error_s("body '%s' not found", shouldername);
    residual_.shoulder_body_id_[shoulder_index] = foot_id;
    shoulder_index++;
  }

  // ----------  derived kinematic quantities for Flip  ----------
  residual_.gravity_ = mju_norm3(model->opt.gravity);
  // velocity at takeoff
  residual_.jump_vel_ =
      mju_sqrt(2 * residual_.gravity_ *
               (ResidualFn::kMaxHeight - ResidualFn::kLeapHeight)); 
  // time in flight phase
  residual_.flight_time_ = 2 * residual_.jump_vel_ / residual_.gravity_;
  // acceleration during jump phase
  residual_.jump_acc_ =
      residual_.jump_vel_ * residual_.jump_vel_ /
      (2 * (ResidualFn::kLeapHeight - ResidualFn::kCrouchHeight));
  // time in crouch sub-phase of jump
  residual_.crouch_time_ =
      mju_sqrt(2 * (ResidualFn::kHeightQuadruped - ResidualFn::kCrouchHeight) /
               residual_.jump_acc_);
  // time in leap sub-phase of jump
  residual_.leap_time_ = residual_.jump_vel_ / residual_.jump_acc_;
  // jump total time  
  residual_.jump_time_ = residual_.crouch_time_ + residual_.leap_time_;
  // velocity at beginning of crouch
  residual_.crouch_vel_ = -residual_.jump_acc_ * residual_.crouch_time_;
  // time of landing phase
  residual_.land_time_ =
      2 * (ResidualFn::kLeapHeight - ResidualFn::kHeightQuadruped) /
      residual_.jump_vel_;
  // acceleration during landing
  residual_.land_acc_ = residual_.jump_vel_ / residual_.land_time_;
  // rotational velocity during flight phase (rotates 1.25 pi)
  residual_.flight_rot_vel_ = 1.25 * mjPI / residual_.flight_time_;
  // rotational velocity at start of leap (rotates 0.5 pi)
  residual_.jump_rot_vel_ =
      mjPI / residual_.leap_time_ - residual_.flight_rot_vel_;
  // rotational acceleration during leap (rotates 0.5 pi)
  residual_.jump_rot_acc_ =
      (residual_.flight_rot_vel_ - residual_.jump_rot_vel_) /
      residual_.leap_time_;
  // rotational deceleration during land (rotates 0.25 pi)
  residual_.land_rot_acc_ =
      2 * (residual_.flight_rot_vel_ * residual_.land_time_ - mjPI / 4) /
      (residual_.land_time_ * residual_.land_time_);
}

// compute average foot position, depending on mode
void Pterosaur::ResidualFn::AverageFootPos(
    double avg_foot_pos[3], double* foot_pos[kNumFoot]) const {
  if (current_mode_ == kModeBiped) {
    int handstand = ReinterpretAsInt(parameters_[biped_type_param_id_]);
    if (handstand) {
      mju_add3(avg_foot_pos, foot_pos[kFootFL], foot_pos[kFootFR]);
    } else {
      mju_add3(avg_foot_pos, foot_pos[kFootHL], foot_pos[kFootHR]);
    }
    mju_scl3(avg_foot_pos, avg_foot_pos, 0.5);
  } else {
    mju_add3(avg_foot_pos, foot_pos[kFootHL], foot_pos[kFootHR]);
    mju_addTo3(avg_foot_pos, foot_pos[kFootFL]);
    mju_addTo3(avg_foot_pos, foot_pos[kFootFR]);
    mju_scl3(avg_foot_pos, avg_foot_pos, 0.25);
  }
}

// return phase as a function of time
double Pterosaur::ResidualFn::GetPhase(double time) const {
  return phase_start_ + (time - phase_start_time_) * phase_velocity_;
}

// horizontal Walk trajectory
void Pterosaur::ResidualFn::Walk(double pos[2], double time) const {
  if (mju_abs(angvel_) < kMinAngvel) {
    // no rotation, go in straight line
    double forward[2] = {heading_[0], heading_[1]};
    mju_normalize(forward, 2);
    pos[0] = position_[0] + heading_[0] + time*speed_*forward[0];
    pos[1] = position_[1] + heading_[1] + time*speed_*forward[1];
  } else {
    // walk on a circle
    double angle = time * angvel_;
    double mat[4] = {mju_cos(angle), -mju_sin(angle),
                     mju_sin(angle),  mju_cos(angle)};
    mju_mulMatVec(pos, mat, heading_, 2, 2);
    pos[0] += position_[0];
    pos[1] += position_[1];
  }
}

// get gait
Pterosaur::ResidualFn::A1Gait Pterosaur::ResidualFn::GetGait() const {
  if (current_mode_ == kModeBiped)
    return kGaitTrot;
  return static_cast<A1Gait>(ReinterpretAsInt(current_gait_));
}

// return normalized target step height
double Pterosaur::ResidualFn::StepHeight(double time, double footphase,
                                             double duty_ratio) const {
  double angle = fmod(time + mjPI - footphase, 2*mjPI) - mjPI;
  double value = 0;
  if (duty_ratio < 1) {
    angle *= 0.5 / (1 - duty_ratio);
    value = mju_cos(mju_clip(angle, -mjPI/2, mjPI/2));
  }
  return mju_abs(value) < 1e-6 ? 0.0 : value;
}

// compute target step height for all feet
void Pterosaur::ResidualFn::FootStep(double step[kNumFoot], double time,
                                         A1Gait gait) const {
  double amplitude = parameters_[amplitude_param_id_];
  double duty_ratio = parameters_[duty_param_id_];
  for (A1Foot foot : kFootAll) {
    double footphase = 2*mjPI*kGaitPhase[gait][foot];
    step[foot] = amplitude * StepHeight(time, footphase, duty_ratio);
  }
}

// height during flip
double Pterosaur::ResidualFn::FlipHeight(double time) const {
  if (time >= jump_time_ + flight_time_ + land_time_) {
    return kHeightQuadruped + ground_;
  }
  double h = 0;
  if (time < jump_time_) {
    h = kHeightQuadruped + time * crouch_vel_ + 0.5 * time * time * jump_acc_;
  } else if (time >= jump_time_ && time < jump_time_ + flight_time_) {
    time -= jump_time_;
    h = kLeapHeight + jump_vel_*time - 0.5*9.81*time*time;
  } else if (time >= jump_time_ + flight_time_) {
    time -= jump_time_ + flight_time_;
    h = kLeapHeight - jump_vel_*time + 0.5*land_acc_*time*time;
  }
  return h + ground_;
}
// height during launch
double Pterosaur::ResidualFn::LaunchHeight(double time) const {
  double start_h = position_[2];
  double crouch_h = ground_ + 0.5;
  double rise_h = ground_ + kHeightQuadruped;
  double pivot_h = ground_ + 1.1;
  double jump_h = ground_ + 1.25;
  double landing_h = ground_ + 0.85;
  if (time < 0) {
    return start_h;
  }
  if (time < preload_time_) {
    double alpha = time / preload_time_;
    // Smoothstep gives zero velocity at start/end of preload.
    double alpha_smooth = alpha * alpha * (3.0 - 2.0 * alpha);
    return start_h + (crouch_h - start_h) * alpha_smooth;
  }
  time -= preload_time_;
  if (time < rise_time_) {
    double alpha = time / rise_time_;
    // Quadratic rise interpolation.
    double alpha_smooth = alpha * alpha;
    return crouch_h + (rise_h - crouch_h) * alpha_smooth;
  }
  time -= rise_time_;
  if (time < pivot_time_) {
    double alpha = time / pivot_time_;
    return rise_h + (pivot_h - rise_h) * alpha * alpha;
  }
  time -= pivot_time_;
  if (time < jump_time_) {
    double alpha = time / jump_time_;
    return pivot_h + (jump_h - pivot_h) * alpha;
  }
  time -= jump_time_;
  if (time < flight_time_) {
    double mid = flight_time_ * 0.5;
    if (time <= mid) {
      return jump_h + (2.8 - jump_h) * (time / mid);
    }
    return 2.8 + (landing_h - 2.8) * ((time - mid) / mid);
  }
  time -= flight_time_;
  if (time < land_time_) {
    double alpha = time / land_time_;
    return landing_h + (ground_ + kHeightQuadruped - landing_h) * alpha;
  }
  return ground_ + kHeightQuadruped;
}

// orientation during flip
//  total rotation = leap + flight + land
//            2*pi = pi/2 + 5*pi/4 + pi/4
void Pterosaur::ResidualFn::FlipQuat(double quat[4], double time) const {
  double angle = 0;
  if (time >= jump_time_ + flight_time_ + land_time_) {
    angle = 2*mjPI;
  } else if (time >= crouch_time_ && time < jump_time_) {
    time -= crouch_time_;
    angle = 0.5 * jump_rot_acc_ * time * time + jump_rot_vel_ * time;
  } else if (time >= jump_time_ && time < jump_time_ + flight_time_) {
    time -= jump_time_;
    angle = mjPI/2 + flight_rot_vel_ * time;
  } else if (time >= jump_time_ + flight_time_) {
    time -= jump_time_ + flight_time_;
    angle = 1.75*mjPI + flight_rot_vel_*time - 0.5*land_rot_acc_ * time * time;
  }
  int flip_dir = ReinterpretAsInt(parameters_[flip_dir_param_id_]);
  double axis[3] = {0, flip_dir ? 1.0 : -1.0, 0};
  mju_axisAngle2Quat(quat, axis, angle);
  mju_mulQuat(quat, orientation_, quat);
}

void Pterosaur::ResidualFn::LaunchQuat(double quat[4], double time) const {
  mju_copy4(quat, orientation_);
}

// launch track: the push (deepest crouch to takeoff) plays back in "Push
// time" seconds; 0 keeps the reference timing
double Pterosaur::ResidualFn::RefRate(double launch_time) const {
  double push_time = parameters_[push_time_param_id_];
  double ref_push = ref_takeoff_time_ - ref_push_start_;
  if (push_time <= 0 || launch_time < ref_push_start_ ||
      launch_time >= ref_push_start_ + push_time) {
    return 1;
  }
  return ref_push / push_time;
}

double Pterosaur::ResidualFn::RefTime(double launch_time) const {
  double push_time = parameters_[push_time_param_id_];
  if (push_time <= 0 || launch_time < ref_push_start_) return launch_time;
  double ref_push = ref_takeoff_time_ - ref_push_start_;
  if (launch_time < ref_push_start_ + push_time) {
    return ref_push_start_ + (launch_time - ref_push_start_) * ref_push / push_time;
  }
  return ref_takeoff_time_ + (launch_time - ref_push_start_ - push_time);
}

// launch track: reference state at time, interpolated between keyframes and
// aligned to the robot pose at mode entry. any output may be nullptr.
void Pterosaur::ResidualFn::LaunchReference(const mjModel* model, double time,
                                            double* qpos, double* qvel,
                                            bool* hands_contact,
                                            bool* feet_contact) const {
  int last = ref_num_frames_ - 1;
  double index = mju_clip(time / ref_dt_, 0, last);
  int index_0 = std::min(static_cast<int>(std::floor(index)), last);
  int index_1 = std::min(index_0 + 1, last);
  double weight_1 = index - index_0;
  double weight_0 = 1 - weight_1;

  if (qpos) {
    const double* qpos_0 = model->key_qpos + model->nq*(ref_key_start_ + index_0);
    const double* qpos_1 = model->key_qpos + model->nq*(ref_key_start_ + index_1);
    mju_scl(qpos, qpos_0, weight_0, model->nq);
    mju_addToScl(qpos, qpos_1, weight_1, model->nq);
    mju_normalize4(qpos + 3);

    // rotate about reference origin by heading, then translate
    double pos[3];
    mju_sub3(pos, qpos, ref_origin_);
    mju_rotVecQuat(qpos, pos, ref_yaw_quat_);
    mju_addTo3(qpos, ref_offset_);
    double quat[4];
    mju_mulQuat(quat, ref_yaw_quat_, qpos + 3);
    mju_copy4(qpos + 3, quat);
  }

  if (qvel) {
    const double* qvel_0 = model->key_qvel + model->nv*(ref_key_start_ + index_0);
    const double* qvel_1 = model->key_qvel + model->nv*(ref_key_start_ + index_1);
    mju_scl(qvel, qvel_0, weight_0, model->nv);
    mju_addToScl(qvel, qvel_1, weight_1, model->nv);

    // linear velocity is in world frame, angular velocity in body frame
    double vel[3];
    mju_rotVecQuat(vel, qvel, ref_yaw_quat_);
    mju_copy3(qvel, vel);
  }

  int nearest = weight_1 < 0.5 ? index_0 : index_1;
  if (hands_contact) {
    *hands_contact = model->numeric_data[ref_hands_contact_adr_ + nearest];
  }
  if (feet_contact) {
    *feet_contact = model->numeric_data[ref_feet_contact_adr_ + nearest];
  }
}
}  // namespace mjpc
