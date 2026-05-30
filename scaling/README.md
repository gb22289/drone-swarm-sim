# Scaling Simulator (lite discrete-event)

This directory contains the lightweight discrete-event simulator used to
generate the swarm-scaling results in Section 5 / Table VII of the IEEE
paper. The full Gazebo + ArduCopter SITL + LIO-SAM stack does not scale
beyond two drones on commodity hardware, so coordination-layer dynamics
at N=5 and N=10 are studied here in a stripped-down ROS2 simulator
that:

- Replaces each drone with a `virtual_drone_lite` node that "flies" by
  sampling travel times from N(3.5, 0.5)s and publishes the **same**
  `/swarm/waypoint_status` JSON envelope used by the full waypoint
  navigator.
- Reuses the production `network_attacker` node unchanged for the
  `coverage_spoof`, `phantom_drone`, and `selective_denial` scenarios.
- Adds two attacker variants only meaningful at N>2:
  - `sybil_attacker_lite` — K parallel phantom identities racing the
    real swarm (Sybil-4 in the paper).
  - `interleaved_attacker_lite` — duplicates real reports with
    `actually_visited=False`; the negative-control scenario.
- Logs ground truth via `gt_logger_lite`, which uses each drone's
  end-of-mission sentinel for authoritative visited-set accounting.

## Files

| File | Purpose |
|------|---------|
| `virtual_drone_lite.py` | Per-drone discrete-event surrogate |
| `gt_logger_lite.py` | Ground-truth aggregator + CSV writer |
| `sybil_attacker_lite.py` | Sybil-4 (parallel phantom identities) |
| `interleaved_attacker_lite.py` | Interleaved injection variant |
| `run_scaling.sh` | Sweep runner — reproduces Table VII |

## Reproducing Table VII

```bash
# from the repo root, with ROS2 sourced and swarm_mission built:
./scaling/run_scaling.sh
```

Per-config CSVs land in `$HOME/lite_sim_results/` and the aggregated
sweep is written to `data/scaling_results.csv`.

Defaults: `REPS=10`, `N_VALUES="2 5 10"`, `WP_PER_DRONE=9`. Override via
environment variables, e.g. `REPS=3 N_VALUES="5 10" ./run_scaling.sh`.

## Output schema (per row)

```
scenario, n_drones, total_wp, gt_visited, reported_visited,
coverage_gap, gap_pct, false_claims, mission_time_s
```

- `gt_visited` — waypoints actually flown to (from drone-sentinel ground truth)
- `reported_visited` — waypoints anyone claimed on the bus (real or spoofed)
- `coverage_gap` — `total_wp - gt_visited`, the metric reported in Table VII
- `false_claims` — count of spoofed/duplicate `/swarm/waypoint_status` messages

## Why "lite"?

The lite simulator is not a substitute for the Gazebo runs. It models
coordination-layer dynamics only — no SLAM, no MAVROS, no physics —
which is exactly the scope of Section 5. For sensor-layer attacks
(scan rotation, IMU injection, point-cloud injection, etc.) the full
two-drone Gazebo stack documented in `docs/SETUP.md` and `docs/LAUNCH.md`
is still required.
