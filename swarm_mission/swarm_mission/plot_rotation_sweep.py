#!/usr/bin/env python3
"""
Plot the scan manipulation rotation-rate sweep.

Reads summary.csv written by slam_failure_logger across N runs and produces:
  - fig_rotation_sweep.pdf  (the IEEE-ready figure)
  - fig_rotation_sweep.png  (quick-look)

X axis: rotation rate (deg/s), log scale so 0.5 / 1 / 2 / 5 spread nicely.
Y axis: time-to-SLAM-failure (seconds), from attack start.
Markers: mean of n trials, error bars = ± std (or min/max if only 2-3 trials).
A horizontal dashed line marks the test-window timeout; runs that survived
the window are plotted at the timeout and annotated with "survived".

Usage:
  python3 plot_rotation_sweep.py \
      --csv ~/results/scan_sweep/summary.csv \
      --out ~/results/scan_sweep/fig_rotation_sweep
  # creates fig_rotation_sweep.pdf and fig_rotation_sweep.png
"""
import argparse
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


SURVIVED = {"survived_window", "unknown"}


def load_summary(path: Path):
    """Return {rate_dps: [(ttf, fail_signal, run_id), ...]}."""
    rows = defaultdict(list)
    with open(path) as f:
        reader = csv.DictReader(f)
        for r in reader:
            try:
                rate = float(r["rotation_rate_dps"])
                ttf = float(r["time_to_failure_s"])
            except (KeyError, ValueError):
                continue
            rows[rate].append((ttf, r.get("fail_signal", ""), r.get("run_id", "")))
    return rows


def summarise(rows):
    """Return (rates_sorted, means, stds, n_trials, n_survived_per_rate)."""
    rates = sorted(rows.keys())
    means, stds, ns, surv = [], [], [], []
    for r in rates:
        vals = [ttf for ttf, _, _ in rows[r]]
        sigs = [sig for _, sig, _ in rows[r]]
        means.append(float(np.mean(vals)))
        stds.append(float(np.std(vals, ddof=0)) if len(vals) > 1 else 0.0)
        ns.append(len(vals))
        surv.append(sum(1 for s in sigs if s in SURVIVED))
    return rates, means, stds, ns, surv


def plot(rows, out_stem: Path, test_window_s: float):
    rates, means, stds, ns, surv = summarise(rows)
    if not rates:
        print("ERROR: no rows found in summary.csv", file=sys.stderr)
        sys.exit(1)

    fig, ax = plt.subplots(figsize=(5.5, 4.0))
    rates_arr = np.array(rates)
    means_arr = np.array(means)
    stds_arr = np.array(stds)

    ax.errorbar(
        rates_arr, means_arr, yerr=stds_arr,
        fmt="o-", color="#c62828",
        ecolor="#c62828", elinewidth=1.2, capsize=4,
        markersize=6, linewidth=1.5, label="Time to SLAM failure (mean ± std)",
    )

    # Horizontal line for the test-window timeout.
    ax.axhline(
        test_window_s, linestyle="--", color="#555", linewidth=1.0,
        label=f"Test window ({test_window_s:.0f}s)",
    )

    # Annotate survivors.
    for r, m, s, n in zip(rates, means, surv, ns):
        if s > 0:
            ax.annotate(
                f"{s}/{n} survived",
                xy=(r, m), xytext=(6, 6),
                textcoords="offset points", fontsize=8, color="#555",
            )

    ax.set_xscale("log")
    ax.set_xticks(rates_arr)
    ax.set_xticklabels([f"{r:g}" for r in rates_arr])
    ax.set_xlabel("Rotation rate (deg/s)")
    ax.set_ylabel("Time to SLAM failure (s)")
    max_ttf = max(means_arr + stds_arr)
    # If no trials survived, don't waste y-axis space on the test-window line
    if not any(surv):
        ax.set_ylim(0, max_ttf * 1.5)
    else:
        ax.set_ylim(0, test_window_s * 1.1)
    ax.set_title("LiDAR scan manipulation: SLAM failure vs rotation rate")
    ax.grid(True, which="both", linestyle=":", linewidth=0.6, alpha=0.6)
    ax.legend(loc="best", fontsize=9, frameon=False)

    # Footer with trial counts.
    counts_str = "  ".join(f"{r:g}°/s (n={n})" for r, n in zip(rates, ns))
    fig.text(0.01, 0.01, counts_str, fontsize=7, color="#666")

    fig.tight_layout()
    pdf = out_stem.with_suffix(".pdf")
    png = out_stem.with_suffix(".png")
    fig.savefig(pdf)
    fig.savefig(png, dpi=180)
    print(f"wrote {pdf}")
    print(f"wrote {png}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True, type=Path,
                    help="path to summary.csv from slam_failure_logger")
    ap.add_argument("--out", required=True, type=Path,
                    help="output stem (no extension) — .pdf and .png are appended")
    ap.add_argument("--test-window-s", type=float, default=150.0,
                    help="test window used in the sweep (for the horizontal line)")
    args = ap.parse_args()

    rows = load_summary(args.csv)
    plot(rows, args.out, args.test_window_s)


if __name__ == "__main__":
    main()
