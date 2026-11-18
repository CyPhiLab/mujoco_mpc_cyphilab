#!/usr/bin/env python3
"""Figures and tables for DESIGN_STUDY.md.

  python design_plots.py --turtle-sim ../../../../turtle_simulator [--mpc label=run.csv:design ...]

Reads results/design_study/*.json (design_study.py search / sweep) and optional MPC logs from
turtle_trajopt (--task_xml=designs/<design>/task.xml; give the design name after ':' so the
motor parameters are known). Writes into results/design_study/:
  ranking.png       best attitude-locked speed per design (gait re-optimized for each)
  reduction.png     speed and electrical energy per metre vs external gear ratio
  envelope.png      main-stroke motor operating points against its torque-speed envelope
  work_split.png    positive work per joint DOF in the fast gaits
  mpc_paths.png     forward distance and tilt of the MPC runs (with --mpc)
  summary.md        the tables used in DESIGN_STUDY.md
"""
import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import design_study as ds  # noqa: E402

OUT = ds.OUT
# reference categorical palette (light), slots 1-3; gray for de-emphasized marks
C1, C2, C3 = "#2a78d6", "#eb6834", "#1baf7a"
GRAY = "#a3a29c"
INK, INK2 = "#0b0b0b", "#52514e"
TURTLE = (0.27, 0.50)  # wild loggerhead / green turtle cruising, m/s
POOL = 0.208  # measured pool speed of the robot as built (XW540-T260, 3S)


def style(plt):
  plt.rcParams.update({
      "font.size": 10, "axes.edgecolor": INK2, "axes.labelcolor": INK, "xtick.color": INK2,
      "ytick.color": INK2, "axes.spines.top": False, "axes.spines.right": False,
      "axes.grid": True, "grid.color": "#e6e5e0", "grid.linewidth": 0.8, "axes.axisbelow": True,
      "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb", "savefig.facecolor": "#fcfcfb",
      "legend.frameon": False,
  })


def load_all():
  recs = {}
  for p in sorted(OUT.glob("*.json")):
    r = json.loads(p.read_text())
    if r.get("best"):
      recs[p.stem] = r
  return recs


def front_idx(b):
  return [i for i, a in enumerate(b["actuators"]) if a not in ds.ACTUATORS[6:]]


def label_of(name, r):
  lab = r["design"]["label"]
  m = re.search(r"_(flipper_scale|g_roll|blade_scale)([\d.]+)$", name)
  if m:
    what = {"flipper_scale": "flipper x", "g_roll": "stroke gear ", "blade_scale": "blade stiffness x"}[m.group(1)]
    lab = lab.replace("baseline: ", "") + f", {what}{m.group(2)}"
  if name.endswith("_thermal"):
    lab += " [RMS <= rated]"
  return lab


def describe(r, name=""):
  b, d = r["best"], r["design"]
  fi = front_idx(b)
  x = b["x"]
  gear = []
  for k in ("g_roll", "g_twist"):
    v = x.get(k, d[k])
    if abs(v - 1) > 1e-3 or k in x:
      gear.append(f"{'stroke' if k == 'g_roll' else 'twist'} {v:.2f}:1")
  return dict(
      speed=b["speed"], bl=b["bl_s"], mech_jpm=b["mech_jpm"], elec_jpm=b["elec_jpm"],
      elec_w=b["elec_power"], mech_w=b["mech_power"], cot=b["cot_elec"],
      tau_pk=max(b["tau_peak"][i] for i in fi), tau_rms=max(b["tau_rms"][i] for i in fi),
      w_pk=max(b["w_peak"][i] for i in fi), rms_rated=b["rms_over_rated"], mass=b["mass"],
      stroke=b["period"], gear=", ".join(gear) or "direct", label=label_of(name, r) if name else d["label"],
      front=d["front"],
  )


def calibration():
  p = OUT.parent / "calibration.json"
  if not p.exists():
    return None
  c = json.loads(p.read_text())
  return c.get("xw540_3s", {}).get("factor")


# ----------------------------------------------------------------------------------------------
def fig_ranking(plt, recs, names, base="xw540_3s"):
  rows = [(n, describe(recs[n], n)) for n in names if n in recs]
  rows.sort(key=lambda r: r[1]["speed"])
  fig, ax = plt.subplots(figsize=(11, 0.42 * len(rows) + 1.6))
  ax.axvspan(*TURTLE, color="#e6e5e0", zorder=0)
  ax.text(TURTLE[1] - 0.004, -0.9, "wild turtles cruise 0.27-0.50 m/s", color=INK2, fontsize=8, ha="right",
          va="center")
  for i, (n, r) in enumerate(rows):
    color = GRAY if n.startswith("xw540") else C1
    if n == base:
      color = INK2
    ax.barh(i, r["speed"], height=0.62, color=color)
    ax.text(r["speed"] + 0.004, i, f"{r['speed']:.3f} m/s  ({r['bl']:.2f} BL/s)  {r['elec_w']:.0f} W"
            + (f"  RMS {r['rms_rated']:.1f}x rated" if r["rms_rated"] else ""), va="center",
            fontsize=8, color=INK)
  ax.set_yticks(range(len(rows)), [r[1]["label"] for r in rows], fontsize=8)
  ax.set_xlabel("speed, attitude locked, gait re-optimized [m/s]")
  ax.set_xlim(0, max(r[1]["speed"] for r in rows) * 1.75)
  ax.set_ylim(-1.4, len(rows) - 0.5)
  ax.grid(axis="y", visible=False)
  ax.set_title("Attitude-locked speed, gait re-optimized per design (gray: XW540-T260, blue: AK45-10)",
               fontsize=10, loc="left", color=INK)
  fig.tight_layout()
  fig.savefig(OUT / "ranking.png", dpi=150)
  plt.close(fig)


def fig_reduction(plt, recs):
  series = [("ak45_serial", "AK45-10, serial", C1, 1.0), ("ak45_diff", "AK45-10, differential", C2, 1.0),
            ("xw540_3s_diff", "XW540 3S, differential", C3, 1.0)]
  fig, axes = plt.subplots(1, 2, figsize=(10, 3.8))
  for base, label, color, g0 in series:
    pts = []
    for n, r in recs.items():
      m = re.fullmatch(re.escape(base) + r"(_g_roll([\d.]+))?", n)
      if not m:
        continue
      g = float(m.group(2)) if m.group(2) else r["design"]["g_roll"]
      pts.append((g, r["best"]["speed"], r["best"]["elec_jpm"]))
    if not pts:
      continue
    pts.sort()
    g, v, e = map(np.array, zip(*pts))
    axes[0].plot(g, v, "-o", color=color, lw=2, ms=6, label=label)
    axes[1].plot(g, e, "-o", color=color, lw=2, ms=6, label=label)
    axes[0].annotate(label, (g[-1], v[-1]), xytext=(6, 0), textcoords="offset points", color=INK2, fontsize=8,
                     va="center")
  axes[0].axhspan(*TURTLE, color="#e6e5e0", zorder=0)
  axes[0].set_xlabel("external ratio on the stroke motor(s) (motor turns per joint turn)")
  axes[0].set_ylabel("speed, attitude locked [m/s]")
  axes[1].set_xlabel("external ratio on the stroke motor(s)")
  axes[1].set_ylabel("electrical energy per metre [J/m]")
  axes[1].legend(fontsize=8)
  for ax in axes:
    ax.set_xlim(left=0)
  axes[0].set_ylim(bottom=0)
  axes[1].set_ylim(bottom=0)
  fig.suptitle("Stroke-gear sweep, peak-limited (gait re-optimized at every ratio; twist motor direct)",
               fontsize=10, x=0.01, ha="left")
  fig.tight_layout()
  fig.savefig(OUT / "reduction.png", dpi=150)
  plt.close(fig)


def fig_envelope(plt, turtle_sim, recs, pairs):
  """pairs: list of (run name, actuator) to plot."""
  fig, axes = plt.subplots(1, len(pairs), figsize=(4.2 * len(pairs), 3.6), squeeze=False)
  for ax, (n, act) in zip(axes[0], pairs):
    if n not in recs:
      continue
    r = recs[n]
    d = ds.Design(**{**r["design"], "search": tuple(map(tuple, r["design"]["search"]))})
    x = r["best"]["x"]
    res, dd = ds.evaluate(turtle_sim, d, x, {}, strokes=5, log=True)
    names = res["actuators"]
    i = names.index(act)
    nu = len(names)
    log = res["log"]
    F = log[:, 1 + 3 + 10 + i]
    w = log[:, 1 + 3 + 10 + nu + i]
    _, acts = ds.build(dd)
    eta = acts[i]["eta"]
    m = ds.MOTORS[acts[i]["motor"]]
    tau = F / eta
    ws = np.linspace(0, m["w0"] * 1.02, 200)
    env = np.minimum(m["tau_peak"], m["tau_stall"] * (1 - ws / m["w0"]))
    ax.fill_between(ws, 0, np.clip(env, 0, None), color="#e6e5e0", lw=0, label="motor envelope (motoring)")
    if m["tau_rated"]:
      ax.axhline(m["tau_rated"], color=INK2, lw=1, ls="--")
      ax.text(m["w0"] * 0.98, m["tau_rated"], "rated (continuous)", ha="right", va="bottom", fontsize=7,
              color=INK2)
    mot = tau * w > 0
    ax.plot(np.abs(w[mot]), np.abs(tau[mot]), ".", ms=3, color=C1, label="motoring")
    ax.plot(np.abs(w[~mot]), np.abs(tau[~mot]), ".", ms=3, color=C2, label="braking")
    rms = np.sqrt(np.mean(tau ** 2))
    ax.axhline(rms, color=C1, lw=1)
    ax.text(0.02 * m["w0"], rms, f"RMS {rms:.1f} N m", va="bottom", fontsize=7, color=INK)
    ax.set_xlim(0, m["w0"] * 1.05)
    ax.set_ylim(0, m["tau_peak"] * 1.25)
    ax.set_xlabel("motor speed |w| [rad/s]")
    ax.set_ylabel("motor torque |tau| [N m]")
    ax.set_title(f"{r['design']['label']}\n{act}, stroke {res['period']:.2f} s", fontsize=8, loc="left")
  axes[0][0].legend(fontsize=7, loc="upper right")
  fig.tight_layout()
  fig.savefig(OUT / "envelope.png", dpi=150)
  plt.close(fig)


def fig_work_split(plt, recs, names):
  rows = [(n, recs[n]["best"]["joint_work"]) for n in names if n in recs]
  fig, ax = plt.subplots(figsize=(7.5, 0.45 * len(rows) + 1.2))
  for i, (n, jw) in enumerate(rows):
    tot = sum(v for k, v in jw.items() if k.startswith("shoulder"))
    yaw = (jw["shoulderL1"] + jw["shoulderR1"]) / tot
    roll = (jw["shoulderL2"] + jw["shoulderR2"]) / tot
    tw = (jw["shoulderL3"] + jw["shoulderR3"]) / tot
    left = 0
    for frac, color, lab in ((roll, C1, "stroke (joint 2)"), (tw, C2, "twist (joint 3)"), (yaw, C3, "sweep (joint 1)")):
      ax.barh(i, frac, left=left, color=color, height=0.6, edgecolor="#fcfcfb", lw=2, label=lab if i == 0 else None)
      left += frac
    ax.text(1.01, i, f"stroke {100 * roll:.0f}%", va="center", fontsize=8, color=INK)
  ax.set_yticks(range(len(rows)), [label_of(n, recs[n]) for n, _ in rows], fontsize=8)
  ax.set_xlim(0, 1.18)
  ax.set_xlabel("share of positive joint work, front limbs")
  ax.legend(fontsize=8, ncol=3, loc="lower left", bbox_to_anchor=(0, 1.0))
  ax.grid(axis="y", visible=False)
  fig.tight_layout()
  fig.savefig(OUT / "work_split.png", dpi=150)
  plt.close(fig)


PARETO = [  # (design, max-speed run, label, color, dashed)
    ("xw540_3s", "xw540_3s", "current robot: XW540 3S", INK2, False),
    ("xw540_3s_flip", None, "XW540 3S, flipper size free", INK2, True),
    ("xw540_14v8", "xw540_14v8", "XW540 4S (14.8 V)", C3, False),
    ("xw540_3s_diff_red", "xw540_3s_diff_red", "XW540 3S + differential + speed-up", C2, False),
    ("ak45_serial_red", "ak45_serial_red_thermal", "AK45-10 serial + gears", "#eda100", False),
    ("ak45_diff_red", "ak45_diff_red_thermal", "AK45-10 differential + gears", C1, False),
    ("ak45_diff_bigflip", "ak45_diff_bigflip_thermal", "AK45-10 differential + gears, flipper free", C1, True),
]
HOTEL = 10.0  # W, electronics (Raspberry Pi 5, cameras, sensors, servo standby): ~24 W measured - motors


def pareto_points(recs, design, endpoint):
  pts = []
  for n, r in recs.items():
    m = re.fullmatch(re.escape(design) + r"(_thermal)?_e([\d.]+)", n)
    if m and r.get("best"):
      b = r["best"]
      pts.append((b["speed"], b["elec_jpm"], b["elec_power"], b["x"], n))
  if endpoint and endpoint in recs:
    b = recs[endpoint]["best"]
    pts.append((b["speed"], b["elec_jpm"], b["elec_power"], b["x"], endpoint))
  pts.sort()
  # keep the non-dominated points (a faster point that costs less replaces slower ones)
  out, best = [], np.inf
  for p in sorted(pts, key=lambda p: -p[0]):
    if p[1] < best:
      out.append(p)
      best = p[1]
  return out[::-1]


def fig_pareto(plt, recs):
  fig, axes = plt.subplots(1, 2, figsize=(11.5, 4.2))
  rows = []
  for design, endpoint, label, color, dashed in PARETO:
    pts = pareto_points(recs, design, endpoint)
    if len(pts) < 2:
      continue
    v = np.array([p[0] for p in pts])
    e = np.array([p[1] for p in pts])
    tot = e + HOTEL / v
    ls = "--" if dashed else "-"
    lw = 1.5 if dashed else 2
    axes[0].plot(v, e, ls, color=color, lw=lw, marker="o", ms=4, label=label)
    axes[1].plot(v, tot, ls, color=color, lw=lw, marker="o", ms=4, label=label)
    i = int(np.argmin(tot))
    axes[1].plot(v[i], tot[i], "o", ms=9, color=color, mec="#fcfcfb", mew=2)
    rows.append((label, pts, i))
  axes[0].axvspan(*TURTLE, color="#e6e5e0", zorder=0)
  axes[1].axvspan(*TURTLE, color="#e6e5e0", zorder=0)
  axes[0].set_xlabel("speed, attitude locked [m/s]")
  axes[0].set_ylabel("motor electrical energy per metre [J/m]")
  axes[1].set_xlabel("speed, attitude locked [m/s]")
  axes[1].set_ylabel(f"with {HOTEL:g} W electronics [J/m]")
  axes[1].set_ylim(0, 600)
  axes[0].set_ylim(bottom=0)
  axes[0].legend(fontsize=7.5, loc="upper left")
  for ax in axes:
    ax.set_xlim(0, 0.37)
  axes[1].set_title("large dots: most efficient (range-optimal) speed", fontsize=8, loc="left", color=INK2)
  fig.suptitle("Speed-efficiency Pareto fronts (energy minimized at each speed; gait and gears re-optimized; "
               "AK45 at RMS <= rated)", fontsize=10, x=0.01, ha="left")
  fig.tight_layout()
  fig.savefig(OUT / "pareto.png", dpi=150)
  plt.close(fig)
  md = ["## Speed-efficiency Pareto fronts (attitude locked)", "",
        f"Electrical energy per metre of the motors, minimized at each speed; 'total' adds {HOTEL:g} W of electronics.",
        "", "| design | speed m/s | motor J/m | motor W | total J/m | stroke s | knobs |", "|---|---|---|---|---|---|---|"]
  for label, pts, i in rows:
    for k, p in enumerate(pts):
      x = p[3]
      knobs = ", ".join(f"{a} {x[a]:.2f}" for a in ("g_roll", "g_twist", "flipper_scale") if a in x)
      star = " **(range-optimal)**" if k == i else ""
      md.append(f"| {label}{star} | {p[0]:.3f} | {p[1]:.0f} | {p[2]:.1f} | {p[1] + HOTEL / p[0]:.0f} | "
                f"{ds.gs.BASE_PERIOD / x['cadence']:.2f} | {knobs} |")
  return md


# ----------------------------------------------------------------------------------------------
def mpc_metrics(path, design):
  """Stroke-averaged metrics of a turtle_trajopt log of a design variant."""
  log = np.genfromtxt(path, delimiter=",", names=True)
  xml = (ds.DESIGNS_DIR / design / "model.xml").read_text()
  gaitxml = (ds.DESIGNS_DIR / design / "reference_gait.xml").read_text()
  cad = float(re.search(r"play at Cadence=(\d+\.\d+)", gaitxml).group(1))
  period = ds.gs.BASE_PERIOD / cad
  acts = [n[len("force_"):] for n in log.dtype.names if n.startswith("force_")]
  par = {a: [float(v) for v in re.search(r'name="motor_' + a + r'" data="([^"]+)"', xml).group(1).split()]
         for a in acts}
  t = log["time"]
  n = int(np.floor((t[-1] - 2 * period) / period))
  t0, t1 = 2 * period, 2 * period + n * period
  sel = (t >= t0) & (t <= t1)
  i0, i1 = np.searchsorted(t, t0), np.searchsorted(t, t1) - 1
  com = np.stack([log["com_x"], log["com_y"], log["com_z"]], 1)
  disp = com[i1] - com[i0]
  T = t[i1] - t[i0]
  along = -disp[1]
  dt = np.diff(t[sel]).mean()
  mech = elec = 0.0
  peak = {}
  rms = {}
  wpk = {}
  for a in acts:
    tp, ts, w0, eta, km = par[a][:5]
    F, w = log["force_" + a][sel], log["vel_" + a][sel]
    tau = F / eta
    pm = np.where(F * w > 0, F * w / eta, F * w * eta)
    cu = (tau / km) ** 2
    mech += np.clip(pm, 0, None).sum() * dt
    elec += np.clip(pm + cu, 0, None).sum() * dt
    peak[a], rms[a], wpk[a] = np.abs(tau).max(), np.sqrt((tau ** 2).mean()), np.abs(w).max()
  tilt = np.degrees(np.arccos(np.clip(log["up_z"][sel], -1, 1))).max()
  front = [a for a in acts if a not in ds.ACTUATORS[6:]]
  mass = None
  return dict(speed=along / T, bl=along / T / ds.BODY_LENGTH, drift=np.sqrt(max(disp @ disp - along ** 2, 0)) / T,
              tilt=tilt, mech_w=mech / T, elec_w=elec / T, mech_jpm=mech / along, elec_jpm=elec / along,
              tau_pk=max(peak[a] for a in front), tau_rms=max(rms[a] for a in front),
              w_pk=max(wpk[a] for a in front), strokes=n, period=period, log=log, t0=t0)


def fig_mpc(plt, runs):
  fig, axes = plt.subplots(1, 2, figsize=(10, 3.6))
  colors = [INK2, C1, C2, C3]
  for (label, m), color in zip(runs, colors):
    log = m["log"]
    axes[0].plot(log["time"], -(log["com_y"] - log["com_y"][0]), color=color, lw=2, label=label)
    axes[1].plot(log["time"], np.degrees(np.arccos(np.clip(log["up_z"], -1, 1))), color=color, lw=1.5, label=label)
  axes[0].set_xlabel("time [s]")
  axes[0].set_ylabel("forward distance [m]")
  axes[1].set_xlabel("time [s]")
  axes[1].set_ylabel("tilt from upright [deg]")
  axes[0].legend(fontsize=8)
  fig.suptitle("MPC on the free body (straight, level swimming)", fontsize=10, x=0.01, ha="left")
  fig.tight_layout()
  fig.savefig(OUT / "mpc_paths.png", dpi=150)
  plt.close(fig)


# ----------------------------------------------------------------------------------------------
def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--turtle-sim", required=True)
  ap.add_argument("--mpc", nargs="*", default=[], help="label=log.csv:design")
  ap.add_argument("--ranking", nargs="*", default=None, help="runs to rank (default: all non-sweep runs)")
  ap.add_argument("--envelope", nargs="*", default=["xw540_3s:shoulderL2", "ak45_serial:shoulderL2"],
                  help="run:actuator pairs")
  ap.add_argument("--work", nargs="*", default=["xw540_3s", "xw540_14v8", "ak45_serial", "ak45_diff"])
  args = ap.parse_args()
  import matplotlib
  matplotlib.use("Agg")
  import matplotlib.pyplot as plt
  style(plt)
  recs = load_all()
  names = args.ranking or [n for n in recs if "_g_roll" not in n and "_blade" not in n and "_e0" not in n]
  fig_ranking(plt, recs, names)
  fig_reduction(plt, recs)
  fig_envelope(plt, args.turtle_sim, recs, [tuple(p.split(":")) for p in args.envelope])
  fig_work_split(plt, recs, args.work)

  base = describe(recs["xw540_3s"], "xw540_3s")["speed"] if "xw540_3s" in recs else None
  lines = ["| design | speed m/s (BL/s) | vs baseline | pool-scaled m/s | stroke s | gear | elec W | elec J/m | CoT | "
           "peak / RMS motor torque N m | RMS/rated | peak motor speed rad/s | mass kg |",
           "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
  for n in sorted(names, key=lambda n: -recs[n]["best"]["speed"]):
    r = describe(recs[n], n)
    rr = f"{r['rms_rated']:.2f}" if r["rms_rated"] is not None else "n/a"
    ratio = r["speed"] / base if base else float("nan")
    lines.append(f"| `{n}` {r['label']} | {r['speed']:.3f} ({r['bl']:.2f}) | {ratio:.2f}x | {POOL * ratio:.2f} | "
                 f"{r['stroke']:.2f} | {r['gear']} | {r['elec_w']:.0f} | {r['elec_jpm']:.0f} | {r['cot']:.2f} | "
                 f"{r['tau_pk']:.1f} / {r['tau_rms']:.1f} | {rr} | {r['w_pk']:.1f} | {r['mass']:.2f} |")
  md = ["## Attitude-locked searches", "", *lines, ""]
  md += fig_pareto(plt, recs) + [""]

  if args.mpc:
    runs = []
    md += ["## MPC (free body)", "",
           "| run | speed m/s (BL/s) | drift m/s | max tilt deg | elec W | elec J/m | mech J/m | "
           "peak / RMS motor torque N m | peak motor speed rad/s | strokes |", "|---|---|---|---|---|---|---|---|---|---|"]
    for item in args.mpc:
      label, rest = item.split("=", 1)
      path, design = rest.rsplit(":", 1)
      m = mpc_metrics(path, design)
      runs.append((label, m))
      md.append(f"| {label} | {m['speed']:.3f} ({m['bl']:.2f}) | {m['drift']:.3f} | {m['tilt']:.0f} | {m['elec_w']:.0f} | "
                f"{m['elec_jpm']:.0f} | {m['mech_jpm']:.0f} | {m['tau_pk']:.1f} / {m['tau_rms']:.1f} | {m['w_pk']:.1f} | "
                f"{m['strokes']} |")
    fig_mpc(plt, runs)
  (OUT / "summary.md").write_text("\n".join(md) + "\n")
  print("\n".join(md))


if __name__ == "__main__":
  main()
