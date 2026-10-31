#!/usr/bin/env python3
"""Render a turtle_trajopt CSV log (qpos columns) to an MP4, camera following the shell.

  MUJOCO_GL=egl python render_run.py mpc.csv mpc.mp4 [--label "MPC, Gait=1"]

Needs `pip install mujoco imageio imageio-ffmpeg`. Use MUJOCO_GL=osmesa without a GPU.
"""
import argparse
import os
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("csv")
  ap.add_argument("out")
  ap.add_argument("--fps", type=int, default=30)
  ap.add_argument("--width", type=int, default=960)
  ap.add_argument("--height", type=int, default=540)
  ap.add_argument("--speedup", type=float, default=1.0, help="playback speed multiplier")
  args = ap.parse_args()

  import imageio
  import mujoco

  # task.xml includes ../common.xml, so load it from its own directory
  model = mujoco.MjModel.from_xml_path(str(HERE / "task.xml"))
  data = mujoco.MjData(model)
  log = np.genfromtxt(args.csv, delimiter=",", names=True)
  qcols = [n for n in log.dtype.names if n.startswith("qpos_")]
  if len(qcols) != model.nq:
    raise SystemExit(f"{args.csv} has {len(qcols)} qpos columns, model has nq={model.nq}")
  qpos = np.stack([log[c] for c in qcols], 1)
  t = log["time"]

  renderer = mujoco.Renderer(model, height=args.height, width=args.width)
  cam = mujoco.MjvCamera()
  cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
  cam.trackbodyid = model.body("fr13_s105k").id
  cam.distance = 1.6
  cam.azimuth = 200
  cam.elevation = -15
  opt = mujoco.MjvOption()

  frames = []
  for ft in np.arange(t[0], t[-1], args.speedup / args.fps):
    i = min(np.searchsorted(t, ft), len(t) - 1)
    data.qpos[:] = qpos[i]
    mujoco.mj_forward(model, data)
    renderer.update_scene(data, cam, opt)
    frames.append(renderer.render())
  imageio.mimsave(args.out, frames, fps=args.fps)
  print(f"wrote {args.out}: {len(frames)} frames")


if __name__ == "__main__":
  main()
