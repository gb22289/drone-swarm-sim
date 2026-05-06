#!/usr/bin/env python3
"""
analyse_lite_results.py
=======================
Aggregates per-trial CSVs from run_lite_sweep.sh into a single summary
table (mean +/- std for each (N, attack) cell), and emits two figures:

  1. coverage_gap vs N for each attack mode
  2. mission_time vs N for each attack mode (throughput-ceiling check)

Also prints a calibration table comparing lite-sim N=2 means to the
full-stack dissertation values for the validation paragraph.

Usage:
    python3 analyse_lite_results.py --input ~/lite_sim_results --output ./figs
"""

import argparse
import csv
import os
import statistics
import sys
from collections import defaultdict


# Reference values from the dissertation full stack at N=2
FULL_STACK_REF = {
    'coverage_spoof':    {'mean_gap': 12.0, 'std_gap': 0.71, 'n': 5},
    'phantom_drone':     {'mean_gap': 10.4, 'std_gap': 1.34, 'n': 5},
    'selective_denial':  {'mean_gap': 11.4, 'std_gap': 0.55, 'n': 5},
}


def load_csvs(input_dir):
    """Returns dict[(N, attack)] -> list of dict rows."""
    by_cell = defaultdict(list)
    for fname in sorted(os.listdir(input_dir)):
        if not fname.endswith('.csv'):
            continue
        path = os.path.join(input_dir, fname)
        with open(path) as f:
            for row in csv.DictReader(f):
                try:
                    n = int(row['n_drones'])
                    atk = row['scenario']
                    row['coverage_gap'] = int(row['coverage_gap'])
                    row['gap_pct'] = float(row['gap_pct'])
                    row['false_claims'] = int(row['false_claims'])
                    row['mission_time_s'] = float(row['mission_time_s'])
                    by_cell[(n, atk)].append(row)
                except (KeyError, ValueError):
                    continue
    return by_cell


def summarise(by_cell):
    """Returns list of summary dicts."""
    summaries = []
    for (n, atk), rows in sorted(by_cell.items()):
        gaps = [r['coverage_gap'] for r in rows]
        mts = [r['mission_time_s'] for r in rows]
        fc = [r['false_claims'] for r in rows]
        if not gaps:
            continue
        summaries.append({
            'n_drones': n,
            'attack': atk,
            'n_trials': len(gaps),
            'mean_gap': round(statistics.mean(gaps), 2),
            'std_gap': (round(statistics.stdev(gaps), 2)
                        if len(gaps) > 1 else 0.0),
            'mean_gap_pct': round(statistics.mean(
                [r['gap_pct'] for r in rows]), 2),
            'mean_false_claims': round(statistics.mean(fc), 2),
            'mean_mission_time_s': round(statistics.mean(mts), 2),
        })
    return summaries


def print_summary_table(summaries):
    print('\n=== Summary table ===')
    cols = ['n_drones', 'attack', 'n_trials', 'mean_gap', 'std_gap',
            'mean_gap_pct', 'mean_false_claims', 'mean_mission_time_s']
    print('\t'.join(cols))
    for s in summaries:
        print('\t'.join(str(s[c]) for c in cols))


def print_calibration(summaries):
    print('\n=== Calibration vs full-stack (N=2) ===')
    print(f'{"attack":<22}{"lite_mean":>12}{"lite_std":>12}'
          f'{"fs_mean":>12}{"fs_std":>12}{"delta":>10}')
    for s in summaries:
        if s['n_drones'] != 2:
            continue
        ref = FULL_STACK_REF.get(s['attack'])
        if ref is None:
            continue
        delta = s['mean_gap'] - ref['mean_gap']
        print(f'{s["attack"]:<22}'
              f'{s["mean_gap"]:>12.2f}{s["std_gap"]:>12.2f}'
              f'{ref["mean_gap"]:>12.2f}{ref["std_gap"]:>12.2f}'
              f'{delta:>+10.2f}')


def make_plots(summaries, output_dir):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib not available, skipping plots', file=sys.stderr)
        return

    os.makedirs(output_dir, exist_ok=True)

    # Group: attack -> list of (n_drones, mean_gap_pct, std_gap_pct)
    by_attack = defaultdict(list)
    for s in summaries:
        by_attack[s['attack']].append(s)

    # Plot 1 -- coverage gap (%) vs N
    plt.figure(figsize=(7, 4.5))
    for atk, rows in sorted(by_attack.items()):
        rows.sort(key=lambda r: r['n_drones'])
        xs = [r['n_drones'] for r in rows]
        ys = [r['mean_gap_pct'] for r in rows]
        # error bars in pct (approximate via std on absolute gap)
        # std_gap is in WPs; convert to pct using each row's total
        plt.plot(xs, ys, marker='o', label=atk)
    plt.xlabel('Swarm size N')
    plt.ylabel('Mean coverage gap (% of mission)')
    plt.title('Layer 1 attack scaling (lite simulator)')
    plt.legend(loc='best')
    plt.grid(True, alpha=0.3)
    out1 = os.path.join(output_dir, 'fig_scaling_coverage_gap.png')
    plt.savefig(out1, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'wrote {out1}')

    # Plot 2 -- mission time vs N
    plt.figure(figsize=(7, 4.5))
    for atk, rows in sorted(by_attack.items()):
        rows.sort(key=lambda r: r['n_drones'])
        xs = [r['n_drones'] for r in rows]
        ys = [r['mean_mission_time_s'] for r in rows]
        plt.plot(xs, ys, marker='s', label=atk)
    plt.xlabel('Swarm size N')
    plt.ylabel('Mean mission time (s)')
    plt.title('Mission time vs swarm size (lite simulator)')
    plt.legend(loc='best')
    plt.grid(True, alpha=0.3)
    out2 = os.path.join(output_dir, 'fig_scaling_mission_time.png')
    plt.savefig(out2, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'wrote {out2}')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--input', default=os.path.expanduser('~/lite_sim_results'))
    p.add_argument('--output', default='./figs')
    args = p.parse_args()

    by_cell = load_csvs(args.input)
    summaries = summarise(by_cell)
    print_summary_table(summaries)
    print_calibration(summaries)
    make_plots(summaries, args.output)


if __name__ == '__main__':
    main()
