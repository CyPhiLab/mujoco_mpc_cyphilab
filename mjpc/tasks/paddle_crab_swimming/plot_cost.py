#!/usr/bin/env python3
"""Plot the PaddleCrabSwimming cost after a MuJoCo MPC run.

Supports two CSV formats:
  - Single-run log written automatically by the task at runtime
    ("time,cost" -> mjpc/tasks/paddle_crab_swimming/paddle_crab_swimming_cost_log.csv,
    created next to wherever the `mjpc` binary is executed from).
  - Multi-speed sweep produced by cost_sweep.cc
    ("step,cost_v0.1,cost_v0.2,...").

Usage:
    python3 plot_cost.py [csv_path] [-o output.png] [--show]
"""
import argparse
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt

DEFAULT_CSV = "paddle_crab_swimming_cost_log.csv"


def load_csv(path):
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = [[float(x) for x in row] for row in reader if row]
    return header, rows


def plot_single_run(rows, output):
    times = [r[0] for r in rows]
    costs = [r[1] for r in rows]
    plt.figure(figsize=(8, 5))
    plt.plot(times, costs)
    plt.xlabel("Simulation time (s)")
    plt.ylabel("Total cost")
    plt.title("PaddleCrabSwimming cost over time")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(output)
    print(f"Saved plot to {output}")


def plot_sweep(header, rows, output):
    steps = [r[0] for r in rows]
    plt.figure(figsize=(9, 6))
    for col_idx in range(1, len(header)):
        values = [r[col_idx] for r in rows]
        plt.plot(steps, values, label=header[col_idx])
    plt.xlabel("Control step")
    plt.ylabel("Total cost")
    plt.title("PaddleCrabSwimming cost sweep")
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=8)
    plt.tight_layout()
    plt.savefig(output)
    print(f"Saved plot to {output}")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv_path", nargs="?", default=DEFAULT_CSV,
                         help=f"Path to the cost CSV file (default: {DEFAULT_CSV})")
    parser.add_argument("-o", "--output", default=None,
                         help="Output image path (default: <csv_path>.png)")
    parser.add_argument("--show", action="store_true",
                         help="Also display the plot interactively")
    args = parser.parse_args()

    csv_path = Path(args.csv_path)
    if not csv_path.exists():
        sys.exit(f"CSV file not found: {csv_path}\n"
                  "Run the mjpc binary (or cost_sweep) first to generate it.")

    header, rows = load_csv(csv_path)
    if not rows:
        sys.exit(f"No data rows found in {csv_path}")

    output = args.output or str(csv_path.with_suffix(".png"))

    if len(header) == 2 and header[0].strip().lower() == "time":
        plot_single_run(rows, output)
    else:
        plot_sweep(header, rows, output)

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
