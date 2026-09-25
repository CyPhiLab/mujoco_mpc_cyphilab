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

#ifndef MJPC_TASKS_PTEROSAUR_PTEROSAUR_H_
#define MJPC_TASKS_PTEROSAUR_PTEROSAUR_H_

#include <string>
#include <mujoco/mujoco.h>
#include "mjpc/task.h"

namespace mjpc {

class Pterosaur : public Task {
 public:
  std::string Name() const override;
  std::string XmlPath() const override;
  class ResidualFn : public mjpc::BaseResidualFn {
   public:
    
    explicit ResidualFn(const Pterosaur* task)
        : mjpc::BaseResidualFn(task) {}
    ResidualFn(const ResidualFn&) = default;
    void Residual(const mjModel* model, const mjData* data,
                  double* residual) const override;

   private:
    friend class Pterosaur;
    //  ============  enums  ============
    // modes
    enum A1Mode {
      kModeQuadruped = 0,
      kModeBiped,
      kModeWalk,
      kModeScramble,
      kModeFlip,
      kModeLaunch,
      kModeLaunchTrack,
      kNumMode
    };

    // feet
    enum A1Foot {
      kFootFL  = 0,
      kFootHL,
      kFootFR,
      kFootHR,
      kNumFoot
    };

    // gaits
    enum A1Gait {
      kGaitStand = 0,
      kGaitWalk,
      kGaitTrot,
      kGaitCanter,
      kGaitGallop,
      kNumGait
    };

    //  ============  constants  ============
    constexpr static A1Foot kFootAll[kNumFoot] = {kFootFL, kFootHL,
                                                  kFootFR, kFootHR};
    constexpr static A1Foot kFootHind[2] = {kFootHL, kFootHR};
    constexpr static A1Gait kGaitAll[kNumGait] = {kGaitStand, kGaitWalk,
                                                  kGaitTrot, kGaitCanter,
                                                  kGaitGallop};

    // gait phase signature (normalized)
    constexpr static double kGaitPhase[kNumGait][kNumFoot] =
    {
    // FL     HL     FR     HR
      {0,     0,     0,     0   },   // stand
      {0,     0.75,  0.5,   0.25},   // walk
      {0,     0.5,   0.5,   0   },   // trot
      {0,     0.33,  0.33,  0.66},   // canter
      {0,     0.4,   0.05,  0.35}    // gallop
    };

    // gait parameters, set when switching into gait
    constexpr static double kGaitParam[kNumGait][6] =
    {
    // duty ratio  cadence  amplitude  balance   upright   height
    // unitless    Hz       meter      unitless  unitless  unitless
      {1,          1,       0,         0,        1,        1},      // stand
      {0.75,       1,       0.06,      0,        1,        1},      // walk
      {0.45,       2,       0.06,      0.2,      1,        1},      // trot
      {0.4,        4,       0.10,      0.03,     0.5,      0.2},    // canter
      {0.3,        3.5,     0.20,      0.03,     0.2,      0.1}     // gallop
    };

    // velocity ranges for automatic gait switching, meter/second
    constexpr static double kGaitAuto[kNumGait] =
    {
      0,     // stand
      0.02,  // walk
      0.02,  // trot
      0.6,   // canter
      2,     // gallop
    };
    // notes:
    // - walk is never triggered by auto-gait
    // - canter actually has a wider range than gallop

    // automatic gait switching: time constant for com speed filter
    constexpr static double kAutoGaitFilter = 0.2;    // second

    // automatic gait switching: minimum time between switches
    constexpr static double kAutoGaitMinTime = 1;     // second

    // target torso height over feet when quadrupedal
    constexpr static double kHeightQuadruped = 0.8;  // meter

    // target torso height over feet when bipedal
    constexpr static double kHeightBiped = 1.8;       // meter
    // radius of foot geoms
    constexpr static double kFootRadius = 0.02;       // meter

    // below this target yaw velocity, walk straight
    constexpr static double kMinAngvel = 0.01;        // radian/second

    // posture gain factors for abduction, hip, knee
    constexpr static double kJointPostureGain[3] = {2, 1, 1};  // unitless

    // flip: crouching height, from which leap is initiated
    constexpr static double kCrouchHeight = 0.3;     // meter

    // flip: leap height, beginning of flight phase
    constexpr static double kLeapHeight = 1.3;        // meter

    // flip: maximum height of flight phase
    constexpr static double kMaxHeight = 2.8;         // meter

    // launch: preload, rise, pivot, push, land durations
    constexpr static double kLaunchPreloadTime = 0.5;  // second
    constexpr static double kLaunchRiseTime = 0.25;    // second
    constexpr static double kLaunchPivotTime = 0.5;     // second
    constexpr static double kLaunchPushTime = 0.18;     // second
    constexpr static double kLaunchLandTime = 0.25;     // second

    // launch: target speeds
    constexpr static double kLaunchPivotSpeed = 7.0;    // m/s
    constexpr static double kLaunchSpeed = 12.96;       // m/s
    constexpr static double kLaunchAngle = 35 * mjPI / 180.0;

    // launch track: floor height the reference was recorded on
    constexpr static double kRefGroundHeight = -1.0;  // meter

    // launch track: base position tracking scale during the push
    constexpr static double kRefPushBaseScale = 0.3;   // unitless

    // launch track: weight of takeoff angular momentum (per unit mass)
    // relative to takeoff velocity
    constexpr static double kRefSpinScale = 5.0;       // 1/meter

    // launch track: leg joint tracking scale once the feet have lifted off
    constexpr static double kRefLegLiftoffScale = 3.0; // unitless

    // launch track: joint tracking scale in flight, holds the limbs still
    constexpr static double kRefFlightJointScale = 4.0; // unitless

    // launch track: left/right mirror sign per limb joint
    // (shoulder1 axes are mirrored, so the arm abduction sign flips)
    constexpr static double kMirrorSign[3] = {-1, 1, 1};  // arm
    constexpr static double kMirrorSignLeg[3] = {1, 1, 1};

    // launch track: minimum height above ground of knee, mid-shin and
    // mid-forearm points (reference minima are 18, 10 and 21 cm)
    constexpr static double kClearKnee = 0.12;         // meter
    constexpr static double kClearShin = 0.06;         // meter
    constexpr static double kClearForearm = 0.15;      // meter

    //  ============  methods  ============
    // return internal phase clock
    double GetPhase(double time) const;

    // return current gait
    A1Gait GetGait() const;

    // compute average foot position, depending on mode
    void AverageFootPos(double avg_foot_pos[3],
                        double* foot_pos[kNumFoot]) const;

    // return normalized target step height
    double StepHeight(double time, double footphase, double duty_ratio) const;

    // compute target step height for all feet
    void FootStep(double step[kNumFoot], double time, A1Gait gait) const;

    // walk horizontal position given time
    void Walk(double pos[2], double time) const;

    // height during flip
    double FlipHeight(double time) const;

    // height during launch
    double LaunchHeight(double time) const;

    // orientation during flip
    void FlipQuat(double quat[4], double time) const;

    // orientation during launch
    void LaunchQuat(double quat[4], double time) const;

    // launch track: reference time for time in mode; the push (deepest
    // crouch to takeoff) is compressed to the "Push time" parameter
    double RefTime(double launch_time) const;

    // launch track: rate of reference time per time in mode
    double RefRate(double launch_time) const;

    // launch track: aligned reference state and contact flags at time
    void LaunchReference(const mjModel* model, double time, double* qpos,
                         double* qvel, bool* hands_contact,
                         bool* feet_contact) const;

    //  ============  task state variables, managed by Transition  ============
    A1Mode current_mode_       = kModeQuadruped;
    double last_transition_time_ = -1;

    // common mode states
    double mode_start_time_  = 0;
    double position_[3]       = {0};

    // walk states
    double heading_[2]        = {0};
    double speed_             = 0;
    double angvel_            = 0;

    // backflip states
    double ground_            = 0;
    double orientation_[4]    = {0};
    double save_gait_switch_  = 0;
    std::vector<double> save_weight_;

    // gait-related states
    double current_gait_      = kGaitStand;
    double phase_start_       = 0;
    double phase_start_time_  = 0;
    double phase_velocity_    = 0;
    double com_vel_[2]        = {0};
    double gait_switch_time_  = 0;

    //  ============  constants, computed in Reset()  ============
    int torso_body_id_        = -1;
    int head_site_id_         = -1;
    int goal_mocap_id_        = -1;
    int gait_param_id_        = -1;
    int gait_switch_param_id_ = -1;
    int flip_dir_param_id_    = -1;
    int biped_type_param_id_  = -1;
    int cadence_param_id_     = -1;
    int amplitude_param_id_   = -1;
    int duty_param_id_        = -1;
    int arm_posture_param_id_ = -1;
    int upright_cost_id_      = -1;
    int balance_cost_id_      = -1;
    int height_cost_id_       = -1;
    int launch_velocity_cost_id_ = -1;
    int ref_base_pos_cost_id_ = -1;
    int ref_base_ori_cost_id_ = -1;
    int ref_joint_pos_cost_id_ = -1;
    int ref_base_vel_cost_id_ = -1;
    int ref_joint_vel_cost_id_ = -1;
    int ref_takeoff_cost_id_ = -1;
    int symmetry_cost_id_ = -1;
    int clearance_cost_id_ = -1;
    int tibia_body_id_[2] = {-1, -1};
    int forearm_body_id_[2] = {-1, -1};
    int launch_speed_param_id_ = -1;
    int launch_angle_param_id_ = -1;
    int ref_start_param_id_ = -1;
    int push_time_param_id_ = -1;
    int foot_geom_id_[kNumFoot];
    int shoulder_body_id_[kNumFoot];

    // derived kinematic quantities describing flip trajectory
    double gravity_           = 0;
    double jump_vel_          = 0;
    double flight_time_       = 0;
    double jump_acc_          = 0;
    double crouch_time_       = 0;
    double preload_time_      = 0;
    double rise_time_         = 0;
    double pivot_time_        = 0;
    double leap_time_         = 0;
    double jump_time_         = 0;
    double crouch_vel_        = 0;
    double land_time_         = 0;
    double land_acc_          = 0;
    double flight_rot_vel_    = 0;
    double jump_rot_vel_      = 0;
    double jump_rot_acc_      = 0;
    double land_rot_acc_      = 0;

    // launch track reference, from launch_reference.xml
    int ref_key_start_        = -1;    // key id of launch_ref_000
    int ref_num_frames_       = 0;
    double ref_dt_            = 0;
    double ref_takeoff_time_  = 0;
    double ref_push_start_    = 0;     // deepest crouch before takeoff
    int ref_hands_contact_adr_ = -1;   // numeric_data address
    int ref_feet_contact_adr_  = -1;   // numeric_data address

    // launch track alignment of reference to robot, set on mode entry
    double ref_origin_[3]     = {0};   // reference base position, frame 0
    double ref_offset_[3]     = {0};   // translation after yaw rotation
    double ref_yaw_quat_[4]   = {1, 0, 0, 0};
  };

  Pterosaur() : residual_(this) {}
  void TransitionLocked(mjModel* model, mjData* data) override;

  // call base-class Reset, save task-related ids
  void ResetLocked(const mjModel* model) override;

  // draw task-related geometry in the scene
  void ModifyScene(const mjModel* model, const mjData* data,
                   mjvScene* scene) const override;

 protected:
  std::unique_ptr<mjpc::ResidualFn> ResidualLocked() const override {
    return std::make_unique<ResidualFn>(residual_);
  }
  ResidualFn* InternalResidual() override { return &residual_; }

 private:
  friend class ResidualFn;
  ResidualFn residual_;
};

}  // namespace mjpc

#endif  // MJPC_TASKS_PTEROSAUR_PTEROSAUR_H_
