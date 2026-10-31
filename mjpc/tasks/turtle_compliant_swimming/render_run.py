#!/usr/bin/env python3
"""Render turtle_trajopt CSV logs (qpos columns) to an MP4, runs tiled side by side.

  MUJOCO_GL=egl python render_run.py out.mp4 "reference=reference.csv" "MPC, Gait=1=mpc.csv"

Each tile follows its run's shell with the same camera; the seabed grid and the
"forward" readout (COM distance along world -y) show how far each run has swum. The label is the
text before the last '='. Needs `pip install mujoco imageio imageio-ffmpeg pillow`; without
a GPU, MUJOCO_GL=egl or osmesa renders in software (about 0.5 s per 640x360 frame).
"""
import argparse
import os
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("out")
  ap.add_argument("runs", nargs="+", help="label=path.csv")
  ap.add_argument("--fps", type=int, default=15)
  ap.add_argument("--width", type=int, default=640, help="per tile")
  ap.add_argument("--height", type=int, default=360)
  ap.add_argument("--speedup", type=float, default=1.0, help="playback speed multiplier")
  ap.add_argument("--start", type=float, default=0.0, help="first time to render [s]")
  ap.add_argument("--shadows", action="store_true", help="slower")
  ap.add_argument("--azimuth", type=float, default=210)
  ap.add_argument("--elevation", type=float, default=-12)
  ap.add_argument("--distance", type=float, default=1.5)
  args = ap.parse_args()

  import imageio
  import mujoco
  from PIL import Image, ImageDraw, ImageFont

  # task.xml includes ../common.xml, so load it from the task directory
  model = mujoco.MjModel.from_xml_path(str(HERE / "task.xml"))
  data = mujoco.MjData(model)
  root = model.body("fr13_s105k").id

  runs = []
  for item in args.runs:
    label, path = item.rsplit("=", 1)
    log = np.genfromtxt(path, delimiter=",", names=True)
    qcols = [n for n in log.dtype.names if n.startswith("qpos_")]
    if len(qcols) != model.nq:
      raise SystemExit(f"{path} has {len(qcols)} qpos columns, model has nq={model.nq}")
    runs.append((label, log["time"], np.stack([log[c] for c in qcols], 1)))
  t_end = min(r[1][-1] for r in runs)

  renderer = mujoco.Renderer(model, height=args.height, width=args.width)
  cam = mujoco.MjvCamera()
  cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
  cam.trackbodyid = root
  cam.distance, cam.azimuth, cam.elevation = args.distance, args.azimuth, args.elevation
  try:
    font = ImageFont.truetype("DejaVuSans.ttf", 18)
  except OSError:
    font = ImageFont.load_default()

  frames = []
  for ft in np.arange(args.start, t_end, args.speedup / args.fps):
    tiles = []
    for label, t, qpos in runs:
      i = min(np.searchsorted(t, ft), len(t) - 1)
      data.qpos[:] = qpos[i]
      mujoco.mj_forward(model, data)
      renderer.update_scene(data, cam)
      renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = args.shadows
      renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = False
      img = Image.fromarray(renderer.render())
      draw = ImageDraw.Draw(img)
      fwd = -data.subtree_com[root][1]
      draw.text((10, 8), label, fill=(255, 255, 255), font=font, stroke_width=2, stroke_fill=(0, 0, 0))
      draw.text((10, args.height - 28), f"t = {ft:5.1f} s   forward {fwd:+.2f} m",
                fill=(255, 255, 255), font=font, stroke_width=2, stroke_fill=(0, 0, 0))
      tiles.append(np.asarray(img))
    frames.append(np.concatenate(tiles, axis=1))
  imageio.mimsave(args.out, frames, fps=args.fps, quality=8, macro_block_size=8)
  print(f"wrote {args.out}: {len(frames)} frames, {len(runs)} tile(s)")
  os._exit(0)  # skip GL context teardown, which errors on some headless EGL setups


if __name__ == "__main__":
  main()
