#!/usr/bin/env python3
"""Plot the PaddleCrabSwimming robot position after a MuJoCo MPC run.

Reads the "time,x,y,z" log written automatically by the task at runtime
(paddle_crab_swimming_pos_log.csv, created next to wherever the `mjpc`
binary is executed from) and plots:
  - a top-down (x,y) trajectory with start/end markers
  - x, y, z position vs. time

Usage:
    python3 plot_position.py [csv_path] [-o output.png] [--show]
"""
import argparse
import csv
import sys
from pathlib import Path

DEFAULT_CSV = "paddle_crab_swimming_pos_log.csv"


def load_csv(path):
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = [[float(x) for x in row] for row in reader if row]
    return header, rows


def drop_leading_garbage(rows):
    """The task logs once before MuJoCo evaluates the sensors, so the first
    sample is (0,0,0) rather than the spawn pose. Differencing across it yields
    a nonsense velocity spike."""
    if len(rows) > 1 and rows[0][1] == 0.0 and rows[0][2] == 0.0 and rows[0][3] == 0.0:
        return rows[1:]
    return rows


def speed_stats(rows, window=0.5):
    times = [r[0] for r in rows]
    xs = [r[1] for r in rows]
    ys = [r[2] for r in rows]
    zs = [r[3] for r in rows]

    speeds = []
    for i in range(1, len(rows)):
        dt = times[i] - times[i - 1]
        if dt <= 0:
            continue
        dx, dy, dz = xs[i] - xs[i - 1], ys[i] - ys[i - 1], zs[i] - zs[i - 1]
        speeds.append(((dx * dx + dy * dy + dz * dz) ** 0.5 / dt, times[i]))

    if not speeds:
        return

    total_t = times[-1] - times[0]
    net = ((xs[-1] - xs[0]) ** 2 + (ys[-1] - ys[0]) ** 2 + (zs[-1] - zs[0]) ** 2) ** 0.5

    # Windowed average speed: instantaneous finite differences are dominated by
    # the paddling oscillation, so peak sustained speed is the useful number.
    best_win, best_t = 0.0, 0.0
    j = 0
    for i in range(len(rows)):
        while times[i] - times[j] > window:
            j += 1
        if times[i] - times[j] >= window * 0.9:
            d = ((xs[i] - xs[j]) ** 2 + (ys[i] - ys[j]) ** 2 + (zs[i] - zs[j]) ** 2) ** 0.5
            v = d / (times[i] - times[j])
            if v > best_win:
                best_win, best_t = v, times[j]

    peak, peak_t = max(speeds)
    print()
    # print(f"Duration:          {total_t:.2f} s over {len(rows)} samples")
    # print(f"Net displacement:  {net:.4f} m  "
        #   f"(dx={xs[-1]-xs[0]:+.4f}, dy={ys[-1]-ys[0]:+.4f}, dz={zs[-1]-zs[0]:+.4f})")
    print(f"Average speed:     {net/total_t:.4f} m/s   "
          f"(net displacement / time)")
    # print(f"Forward speed:     {-(ys[-1]-ys[0])/total_t:+.4f} m/s  along -Y "
    #       f"(paddle thrust direction)")
    # print(f"Peak {window:.1f}s speed:   {best_win:.4f} m/s  starting at t={best_t:.2f} s")
    # print(f"Peak instantaneous:{peak:.4f} m/s  at t={peak_t:.2f} s  "
        #   f"(includes paddling oscillation)")


def plot_position(rows, output):
    # Imported lazily so the speed report still works without matplotlib.
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed - skipping plot (stats still shown below)")
        return None

    times = [r[0] for r in rows]
    xs = [r[1] for r in rows]
    ys = [r[2] for r in rows]
    zs = [r[3] for r in rows]

    fig, (ax_traj, ax_time) = plt.subplots(1, 2, figsize=(12, 5))

    ax_traj.plot(xs, ys, "-", color="tab:blue", alpha=0.7)
    ax_traj.plot(xs[0], ys[0], "go", label="start")
    ax_traj.plot(xs[-1], ys[-1], "ro", label="end")
    ax_traj.set_xlabel("x (m)")
    ax_traj.set_ylabel("y (m)")
    ax_traj.set_title("Top-down trajectory")
    ax_traj.axis("equal")
    ax_traj.grid(True, alpha=0.3)
    ax_traj.legend()

    ax_time.plot(times, xs, label="x")
    ax_time.plot(times, ys, label="y")
    ax_time.plot(times, zs, label="z")
    ax_time.set_xlabel("Simulation time (s)")
    ax_time.set_ylabel("Position (m)")
    ax_time.set_title("Position over time")
    ax_time.grid(True, alpha=0.3)
    ax_time.legend()

    fig.tight_layout()
    fig.savefig(output)
    print(f"Saved plot to {output}")
    print(f"Start: ({xs[0]:.3f}, {ys[0]:.3f}, {zs[0]:.3f})")
    print(f"End:   ({xs[-1]:.3f}, {ys[-1]:.3f}, {zs[-1]:.3f})")
    return plt


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv_path", nargs="?", default=DEFAULT_CSV,
                         help=f"Path to the position CSV file (default: {DEFAULT_CSV})")
    parser.add_argument("-o", "--output", default=None,
                         help="Output image path (default: <csv_path>.png)")
    parser.add_argument("--show", action="store_true",
                         help="Also display the plot interactively")
    args = parser.parse_args()

    csv_path = Path(args.csv_path)
    if not csv_path.exists():
        sys.exit(f"CSV file not found: {csv_path}\n"
                  "Run the mjpc binary first to generate it.")

    header, rows = load_csv(csv_path)
    if not rows:
        sys.exit(f"No data rows found in {csv_path}")

    rows = drop_leading_garbage(rows)

    output = args.output or str(csv_path.with_suffix(".png"))
    plt = plot_position(rows, output)
    speed_stats(rows)

    if args.show and plt is not None:
        plt.show()


if __name__ == "__main__":
    main()
