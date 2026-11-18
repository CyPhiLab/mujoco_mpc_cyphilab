#!/usr/bin/env python3
"""Calibration against the real robot (pool, XW540-T260 on 3S, Van der Geest gait):
0.208 m/s and 24.0 W mean electrical power (arXiv:2602.21389 and the team).

Plays the reference gait shape (turtle_sim.py's scaling: yaw 1/2, roll 5/6, twist 1/3 of the
biological amplitudes) at increasing stroke rate on the baseline design, attitude locked (a
stand-in for the robot's rear-flipper pitch stabilization), and reports the fastest speed the
motors allow. The ratio measured / simulated is the fluid-model calibration factor.

  python calibrate.py --turtle-sim ../../../../turtle_simulator
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import design_study as ds  # noqa: E402

REAL_SPEED = 0.208  # m/s
REAL_POWER = 24.0   # W, whole robot (electronics included)


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--turtle-sim", required=True)
  ap.add_argument("--designs", nargs="+", default=["xw540_3s", "xw540_12v"])
  ap.add_argument("--cadence", type=float, nargs="+", default=[1.0, 1.25, 1.5, 1.75, 2.0, 2.25, 2.5, 3.0])
  args = ap.parse_args()
  out = {}
  for name in args.designs:
    d = ds.DESIGNS[name]
    rows = []
    cache = {}
    for c in args.cadence:
      x = dict(cadence=c, yaw=0.5, roll=5 / 6, twist=1 / 3, shift=0.0)
      r, _ = ds.evaluate(args.turtle_sim, d, x, cache, strokes=6)
      rows.append(dict(cadence=c, period=r["period"], speed=r["speed"], elec_power=r["elec_power"],
                       mech_power=r["mech_power"], w_peak=max(r["w_peak"][:6]), sat=max(r["sat_frac"][:6])))
      print(f"{name} cadence {c:4.2f} (stroke {r['period']:.2f} s): {r['speed']:.4f} m/s, "
            f"mech {r['mech_power']:.1f} W, elec (motors) {r['elec_power']:.1f} W, "
            f"peak motor speed {rows[-1]['w_peak']:.2f} rad/s, saturated {100 * rows[-1]['sat']:.0f}% of the time",
            flush=True)
    best = max(rows, key=lambda r: r["speed"])
    out[name] = dict(rows=rows, best=best, factor=REAL_SPEED / best["speed"])
    print(f"{name}: fastest reference-shape gait {best['speed']:.4f} m/s at cadence {best['cadence']:g} -> "
          f"calibration factor {REAL_SPEED / best['speed']:.2f}")
  ds.OUT.mkdir(parents=True, exist_ok=True)
  (ds.OUT.parent / "calibration.json").write_text(json.dumps(out, indent=1))


if __name__ == "__main__":
  main()
