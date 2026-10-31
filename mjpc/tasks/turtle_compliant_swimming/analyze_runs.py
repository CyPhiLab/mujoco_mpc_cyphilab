#!/usr/bin/env python3
"""Compare turtle_trajopt runs and extract an optimized gait.

  python analyze_runs.py ref=reference.csv mpc=mpc.csv --out-dir figs \
      [--extract mpc --extract-strokes 4]

Writes to --out-dir:
  summary.md        stroke-averaged metrics per run (skips the first two strokes)
  paths.png         COM path (side and top view), speed and tilt over time
  gait_<run>.png    phase-averaged actuator targets of each run vs the reference
  gait_<run>.csv    with --extract: the run's phase-averaged actuator targets over
                    its last strokes, in the format turtle_trajopt --gait_csv reads

The extracted gait is a Fourier fit (--harmonics) of the commanded targets, so it is
periodic and smooth. It is open loop: the MPC run it comes from also corrected each
stroke with feedback, so play it back with --mode=reference to see what survives.
"""
import argparse
import re
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
# Categorical slots 1-6 of the dataviz reference palette (validated, light mode).
COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]
G = 9.81


def load(path):
  data = np.genfromtxt(path, delimiter=",", names=True)
  return data


def gait_period():
  xml = (HERE / "reference_gait.xml").read_text()
  return float(re.search(r'name="turtle_gait_period" data="([^"]+)"', xml).group(1))


def actuators(d):
  return [n[len("ctrl_"):] for n in d.dtype.names if n.startswith("ctrl_")]


def window(d, period, skip=2, last=None):
  """Indices of whole strokes after `skip` strokes (or the `last` strokes)."""
  t = d["time"]
  n = int(np.floor((t[-1] - skip * period) / period + 1e-9))
  if last is not None:
    n = min(n, last)
  t1 = skip * period + int(np.floor((t[-1] - skip * period) / period + 1e-9)) * period
  t0 = t1 - n * period
  i0, i1 = np.searchsorted(t, t0 - 1e-9), np.searchsorted(t, t1 - 1e-9)
  return i0, min(i1, len(t) - 1), n


def metrics(d, period, mass, heading=0.0):
  direction = np.array([np.sin(heading), -np.cos(heading), 0.0])
  i0, i1, n = window(d, period)
  t = d["time"]
  com = np.stack([d["com_x"], d["com_y"], d["com_z"]], 1)
  disp = com[i1] - com[i0]
  dt = t[i1] - t[i0]
  along = disp @ direction
  work = np.trapezoid(d["power"][i0:i1 + 1], t[i0:i1 + 1])
  tilt = np.degrees(np.arccos(np.clip(d["up_z"][i0:i1 + 1], -1, 1)))
  return dict(strokes=n, speed=along / dt, net=np.linalg.norm(disp) / dt,
              drift=np.sqrt(max(disp @ disp - along**2, 0)) / dt,
              climb=disp[2] / dt, tilt=tilt.max(), power=work / dt,
              jpm=work / along if along > 0 else np.nan,
              cot=work / (mass * G * along) if along > 0 else np.nan)


def phase_fit(t, y, period, harmonics, samples):
  """Least-squares Fourier series of y(t) with the given period, on a phase grid."""
  w = 2 * np.pi * t / period
  cols = [np.ones_like(w)]
  for k in range(1, harmonics + 1):
    cols += [np.cos(k * w), np.sin(k * w)]
  coef, *_ = np.linalg.lstsq(np.stack(cols, 1), y, rcond=None)
  wg = 2 * np.pi * np.arange(samples) / samples
  cols = [np.ones_like(wg)]
  for k in range(1, harmonics + 1):
    cols += [np.cos(k * wg), np.sin(k * wg)]
  return np.stack(cols, 1) @ coef


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("runs", nargs="+", help="label=path.csv, in legend order")
  ap.add_argument("--out-dir", type=Path, default=Path("."))
  ap.add_argument("--extract", action="append", default=[], help="label of a run to extract a gait from")
  ap.add_argument("--extract-strokes", type=int, default=4)
  ap.add_argument("--harmonics", type=int, default=12)
  ap.add_argument("--samples", type=int, default=128)
  ap.add_argument("--mass", type=float, default=8.994, help="robot mass [kg] for CoT")
  args = ap.parse_args()

  import matplotlib
  matplotlib.use("Agg")
  import matplotlib.pyplot as plt
  plt.rcParams.update({"axes.spines.top": False, "axes.spines.right": False,
                       "axes.grid": True, "grid.color": "#e6e5e0", "grid.linewidth": 0.6,
                       "axes.edgecolor": "#8a897f", "axes.labelcolor": "#3d3c36",
                       "xtick.color": "#5c5b53", "ytick.color": "#5c5b53",
                       "font.size": 9, "legend.frameon": False, "lines.linewidth": 2})

  args.out_dir.mkdir(parents=True, exist_ok=True)
  period = gait_period()
  runs = []
  for item in args.runs:
    label, path = item.split("=", 1)
    runs.append((label, load(path)))
  if len(runs) > len(COLORS):
    raise SystemExit(f"at most {len(COLORS)} runs per figure")
  color = {label: COLORS[i] for i, (label, _) in enumerate(runs)}

  # ---- summary table ----
  rows = ["| run | strokes | speed along heading [m/s] | net speed [m/s] | off-heading drift [m/s] "
          "| climb rate [m/s] | max tilt [deg] | mean +power [W] | energy [J/m] | CoT |",
          "|---|---|---|---|---|---|---|---|---|---|"]
  for label, d in runs:
    m = metrics(d, period, args.mass)
    rows.append(f"| {label} | {m['strokes']} | {m['speed']:.4f} | {m['net']:.4f} | {m['drift']:.4f} "
                f"| {m['climb']:+.4f} | {m['tilt']:.1f} | {m['power']:.2f} | {m['jpm']:.1f} | {m['cot']:.3f} |")
  (args.out_dir / "summary.md").write_text("\n".join(rows) + "\n")
  print("\n".join(rows))

  # ---- paths, speed, tilt ----
  fig, ax = plt.subplots(2, 2, figsize=(10, 7), constrained_layout=True)
  for label, d in runs:
    c = color[label]
    fwd = -d["com_y"]
    ax[0, 0].plot(fwd, d["com_z"], color=c, label=label)
    ax[0, 1].plot(fwd, d["com_x"], color=c, label=label)
    # one-stroke moving average of the forward velocity
    t = d["time"]
    k = max(int(round(period / np.median(np.diff(t)))), 1)
    v = np.convolve(-d["vel_y"], np.ones(k) / k, mode="valid")
    ax[1, 0].plot(t[k - 1:], v, color=c, label=label)
    ax[1, 1].plot(t, np.degrees(np.arccos(np.clip(d["up_z"], -1, 1))), color=c, label=label)
  ax[0, 0].set(title="Side view of COM path", xlabel="forward distance, world -y [m]", ylabel="height z [m]")
  ax[0, 1].set(title="Top view of COM path", xlabel="forward distance, world -y [m]", ylabel="sideways x [m]")
  ax[1, 0].set(title="Forward speed, one-stroke moving average", xlabel="time [s]", ylabel="speed along -y [m/s]")
  ax[1, 1].set(title="Shell tilt from upright", xlabel="time [s]", ylabel="tilt [deg]")
  for a in ax[0]:
    a.set_aspect("equal", adjustable="datalim")
  ax[0, 0].legend(loc="best")
  fig.savefig(args.out_dir / "paths.png", dpi=150)
  plt.close(fig)

  # ---- phase-averaged actuator targets, per run ----
  names = actuators(runs[0][1])
  phase = np.arange(args.samples) / args.samples
  for label, d in runs:
    i0, i1, n = window(d, period, last=args.extract_strokes)
    t = d["time"][i0:i1]
    fig, ax = plt.subplots(2, 5, figsize=(14, 5.5), sharex=True, constrained_layout=True)
    fitted = {}
    for j, name in enumerate(names):
      a = ax.flat[j]
      ref = phase_fit(t, d["ref_" + name][i0:i1], period, args.harmonics, args.samples)
      cmd = phase_fit(t, d["ctrl_" + name][i0:i1], period, args.harmonics, args.samples)
      fitted[name] = cmd
      a.plot(phase, np.degrees(ref), color="#8a897f", linewidth=1.5, linestyle="--", label="reference gait")
      a.plot(phase, np.degrees(cmd), color=color[label], label=f"{label}, last {n} strokes")
      a.set_title(name)
      lo, hi = a.get_ylim()
      if hi - lo < 10:  # flat traces: show at least a 10 deg span
        mid = 0.5 * (lo + hi)
        a.set_ylim(mid - 5, mid + 5)
      if j % 5 == 0:
        a.set_ylabel("joint target [deg]")
      if j >= 5:
        a.set_xlabel("stroke phase")
    ax.flat[0].legend(loc="best", fontsize=8)
    fig.suptitle(f"Actuator targets over a stroke: {label} (Fourier fit, {args.harmonics} harmonics)")
    fig.savefig(args.out_dir / f"gait_{label}.png", dpi=150)
    plt.close(fig)

    if label in args.extract:
      table = np.stack([fitted[nm] for nm in names], 1)
      out = args.out_dir / f"gait_{label}.csv"
      np.savetxt(out, table, delimiter=",", header=",".join(names), comments="", fmt="%.6f")
      print(f"wrote {out} ({args.samples} samples, phase 0 = t mod {period:.4f} s = 0)")


if __name__ == "__main__":
  main()
