#!/usr/bin/env python3
"""Export a launch_track.py result as a self-contained package: the
model as standalone MJCF (with its meshes), the start state and
controls, the recorded trajectory, and a replay script that needs only
mujoco and numpy.

load_model() edits the compiled model after compiling (motor gains and
torque-speed lines, joint limits, contact parameters, time step); those
edits are copied back into the spec before writing the XML, and the export
is checked by replaying the controls on the exported model.

Usage:
  export_model.py RUN_DIR STEP_NAME OUT_DIR
"""
import json
import os
import shutil
import sys

import mujoco
import numpy as np

import launch_track as LT
import launch_trajopt as T

REPLAY = '''#!/usr/bin/env python3
"""Replay the exported launch: python replay.py [--video out.mp4]"""
import os, sys
import mujoco
import numpy as np

here = os.path.dirname(os.path.abspath(__file__))
m = mujoco.MjModel.from_xml_path(os.path.join(here, 'model.xml'))
z = np.load(os.path.join(here, 'trajectory.npz'))
d = mujoco.MjData(m)
d.qpos[:] = z['start_qpos']                 # starts at rest
mujoco.mj_forward(m, d)
err = 0.0
for t in range(len(z['ctrl'])):
  d.ctrl[:] = z['ctrl'][t]
  mujoco.mj_step(m, d)
  err = max(err, np.abs(d.qpos - z['qpos'][t]).max())   # qpos[t]: after step t
k = int(z['takeoff_step'])
v = z['comvel'][k]
print(f'max |qpos - recorded| {err:.2e}; takeoff t={z["time"][k]:.3f} s, '
      f'{np.linalg.norm(v):.2f} m/s at {np.degrees(np.arctan2(v[2], -v[0])):.1f} deg')
'''


README = '''# Exported launch: {run}/{step}

Self-contained: needs only `mujoco` (exported with {mujoco}) and `numpy`.

- `model.xml` + `assets/`: the robot and floor as standalone MJCF (motor
  gains, torque-speed lines, shoulder differential tendons, joint limits and
  contact parameters baked in). Time step {dt} s, mass {mass:.1f} kg.
  Keyframe `launch_start` = the start pose (start at rest).
- `trajectory.npz`: `start_qpos`; `ctrl` ({n} x nu, actuator commands in
  [-1, 1], one row per step); recorded `qpos`, `qvel`, `comvel` (CoM
  velocity), `time` and contact flags, each row the state *after* that
  step; `takeoff_step`; `controls_symmetric` (the optimizer's 6 left/right
  symmetric channels, mirrored to the 12 actuators as in `ctrl`).
- `replay.py`: replays `ctrl` from `start_qpos` and checks against the
  recording (export check: max |qpos error| {err:.1e}).
- `info.json`: actuator and joint names, stall torques, the optimizer
  settings that produced the launch.

Takeoff at {t_off:.3f} s: {speed:.2f} m/s at {angle:.1f} deg.

Actuator model: force = gain * ctrl - (gain / 20 rad/s) * actuator speed,
clipped to +-gain (DC motor torque-speed line). The two shoulder actuators
per side drive fixed tendons (swing + abduction, swing - abduction).
'''


def sync_spec(spec, m):
  """Copy the post-compile edits of load_model from the model into the spec."""
  spec.option.timestep = m.opt.timestep
  for i, a in enumerate(spec.actuators):
    j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a.name)
    a.gaintype = mujoco.mjtGain(m.actuator_gaintype[j])
    a.gainprm[:] = m.actuator_gainprm[j]
    a.biastype = mujoco.mjtBias(m.actuator_biastype[j])
    a.biasprm[:] = m.actuator_biasprm[j]
    a.forcelimited = mujoco.mjtLimited.mjLIMITED_TRUE if m.actuator_forcelimited[j] else mujoco.mjtLimited.mjLIMITED_FALSE
    a.forcerange = m.actuator_forcerange[j]
    a.ctrllimited = mujoco.mjtLimited.mjLIMITED_TRUE if m.actuator_ctrllimited[j] else mujoco.mjtLimited.mjLIMITED_FALSE
    a.ctrlrange = m.actuator_ctrlrange[j]
  for jt in spec.joints:
    j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jt.name)
    if j < 0 or m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
      continue
    jt.limited = mujoco.mjtLimited.mjLIMITED_TRUE if m.jnt_limited[j] else mujoco.mjtLimited.mjLIMITED_FALSE
    jt.range = m.jnt_range[j]
    k = np.array(jt.stiffness, dtype=float)
    k[0] = m.jnt_stiffness[j]
    jt.stiffness = k
    jt.springref = m.qpos_spring[m.jnt_qposadr[j]]
  for g in spec.geoms:
    if not g.name:
      continue
    j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, g.name)
    g.solimp = m.geom_solimp[j]
    g.solref = m.geom_solref[j]
    g.margin = m.geom_margin[j]


def main():
  run, step, out = sys.argv[1:4]
  row = next(r for r in json.load(open(os.path.join(run, 'summary.json')))
             if r['name'] == step)
  solver, args = LT.solver_for_step(run, row)
  m, spec = solver.m, T.LAST_SPEC
  z = dict(np.load(os.path.join(run, step + '.npz')))
  os.makedirs(os.path.join(out, 'assets'), exist_ok=True)

  sync_spec(spec, m)
  for key in list(spec.keys):            # reference keyframes: not needed
    spec.delete(key)
  spec.add_key(name='launch_start', qpos=z['start_qpos'], time=0.0)
  spec.meshdir = 'assets'
  for mesh in spec.meshes:
    src = os.path.join(T.TASK_DIR, 'assets', os.path.basename(mesh.file))
    shutil.copy(src, os.path.join(out, 'assets', os.path.basename(mesh.file)))
    mesh.file = os.path.basename(mesh.file)
  xml = spec.to_xml()
  open(os.path.join(out, 'model.xml'), 'w').write(xml)

  # check: the exported XML reproduces the trajectory
  me = mujoco.MjModel.from_xml_path(os.path.join(out, 'model.xml'))
  d = mujoco.MjData(me)
  d.qpos[:] = z['start_qpos']
  mujoco.mj_forward(me, d)
  err = 0.0
  for t in range(len(z['ctrl'])):
    d.ctrl[:] = z['ctrl'][t]
    mujoco.mj_step(me, d)
    err = max(err, np.abs(d.qpos - z['qpos'][t]).max())
  print(f'exported model replay: max |qpos - recorded| = {err:.2e}')

  takeoff = int(round(row['takeoff_time'] / m.opt.timestep)) - 1
  np.savez(os.path.join(out, 'trajectory.npz'),
           controls_symmetric=np.load(os.path.join(run, step + '_controls.npy')),
           takeoff_step=takeoff, **z)
  open(os.path.join(out, 'replay.py'), 'w').write(REPLAY)
  open(os.path.join(out, 'README.md'), 'w').write(README.format(
      run=run, step=step, mujoco=mujoco.__version__, dt=m.opt.timestep,
      mass=m.body_subtreemass[1], n=len(z['ctrl']), t_off=row['takeoff_time'],
      speed=row['speed'], angle=row['angle_deg'], err=err))
  info = {'source_run': run, 'source_step': step, 'mujoco_version': mujoco.__version__,
          'timestep_s': m.opt.timestep, 'mass_kg': float(m.body_subtreemass[1]),
          'actuators': [m.actuator(i).name for i in range(m.nu)],
          'actuator_stall_torque_Nm': m.actuator_gainprm[:m.nu, 0].round(3).tolist(),
          'no_load_speed_rad_s': 20.0,
          'joints': [m.joint(j).name for j in range(m.njnt)],
          'takeoff': {k: row[k] for k in ('takeoff_time', 'speed', 'angle_deg')},
          'optimizer_args': args.__dict__}
  json.dump(info, open(os.path.join(out, 'info.json'), 'w'), indent=1)
  print('wrote', out)


if __name__ == '__main__':
  main()
