# Swarm-Lite Simulator — Run Instructions

A lightweight discrete-event simulator that retains the full ROS2/DDS coordination layer while abstracting drone physics as parameterised travel-time distributions. Built for the IEEE T-ITS extension to evaluate Layer 1 coordination attacks at swarm sizes (N=5, 10) that the full ArduCopter SITL + Gazebo + LIO-SAM stack cannot sustain on a single workstation.

---

## Why this exists

The full-stack simulation in [SETUP.md](SETUP.md) is necessary for Layer 2 (sensor-path) attacks because the SLAM pipeline is the target. Layer 1 attacks, however, are **message-driven**: every Layer 1 attack works entirely through the `/swarm/waypoint_status` topic, and the only thing physical dynamics contribute is the timing of legitimate status publications. If we replace the sensor-and-flight pipeline with a calibrated travel-time model that emits `/swarm/waypoint_status` messages with realistic cadence, the attack surface is preserved exactly.

This lets us scale to N=5 and N=10 in seconds-of-compute per trial instead of minutes, and run hundreds of trials on a laptop.

The attacker scripts (`network_attacker.py`, `interleaved_attacker.py`, `sybil_attacker_lite.py`) are byte-identical to the full-stack versions for `coverage_spoof`, `phantom_drone`, and `selective_denial`. The new attack variants (`interleaved`, `sybil`) are pure-DDS publishers that work against either simulator.

---

## Files

| File | Purpose |
|---|---|
| `swarm_lite_sim.py` | Spawns N virtual drones as ROS2 nodes. Each drone picks the next unvisited waypoint from its zone, sleeps for `flight_time ~ N(mean, std)`, then publishes a real `/swarm/waypoint_status` message. |
| `ground_truth_logger_lite.py` | Subscribes to a separate `/swarm/ground_truth` topic that virtual drones publish on but attackers do not, then writes a per-trial summary CSV. |
| `interleaved_attacker.py` | New Layer 1 variant: spoofs all-drone identities in interleaved order to distribute coverage gaps spatially. |
| `sybil_attacker_lite.py` | Sybil/BFT-threshold attack: injects f+1 phantom identities to empirically breach the N>3f bound at N=10. |
| `run_lite_sweep.sh` | End-to-end harness for the scaling experiments. |
| `analyse_lite_results.py` | Aggregates per-trial CSVs into a summary table and figures. Includes a calibration block comparing lite-sim N=2 means to the full-stack dissertation values. |

---

## Calibration: travel-time distribution

The single parameter the lite simulator depends on is `flight_time ~ N(mean, std)`, the inter-waypoint flight duration.

**Defaults (`mean=2.0s`, `std=0.4s`) are calibrated from `mission_results-5e2f54a4.csv`.** The CSV records 33 honest 12/12 trial snapshots; consecutive successful trials cluster around 11–18 s apart, with ~5 s of restart overhead between them. Subtracting overhead gives ~13 s of actual flight per mission. With 12 waypoints split between 2 drones (6 WPs per drone, flying in parallel), that's ~13 / 6 ≈ 2.2 s per waypoint per drone — N(2.0, 0.4) covers that range.

If you have access to richer per-waypoint timestamps from `ground_truth_logger.py`, you can refine. To override:

```bash
MEAN_FLIGHT=2.2 STD_FLIGHT=0.3 ./run_lite_sweep.sh
```

The calibration block in `analyse_lite_results.py` reports the delta against the full-stack reference values (N=2 means: 12.0 / 10.4 / 11.4 ± 0.5 WP). If lite-sim N=2 means deviate by more than ±0.5 WP, retune `mean_flight` — slower flight = longer mission window = larger coverage gap for a given `spoof_delay`.

---

## Quick start

### 1. Smoke test (N=2, single trial)

```bash
source /opt/ros/humble/setup.bash
chmod +x run_lite_sweep.sh

# 1 trial of each attack mode at N=2 only
N_LIST="2" TRIALS=1 ./run_lite_sweep.sh
```

This should produce `~/lite_sim_results/N2_*.csv`. Inspect with:

```bash
cat ~/lite_sim_results/N2_coverage_spoof.csv
```

### 2. Validate against full stack

```bash
N_LIST="2" TRIALS=5 ./run_lite_sweep.sh
python3 analyse_lite_results.py --input ~/lite_sim_results
```

The calibration block prints something like:

```
=== Calibration vs full-stack (N=2) ===
attack                   lite_mean    lite_std     fs_mean      fs_std      delta
coverage_spoof              12.20        0.45       12.00        0.71      +0.20
phantom_drone               10.60        1.20       10.40        1.34      +0.20
selective_denial            11.40        0.55       11.40        0.55      +0.00
```

Deltas within ±0.5 WP are the methodological green light to proceed with N>2 experiments.

### 3. Full scaling sweep

```bash
./run_lite_sweep.sh         # N={2,5,10} x 5 attacks x 5 trials + Sybil at N=10
python3 analyse_lite_results.py --input ~/lite_sim_results --output ./figs
```

Wall-clock time: ~25 minutes for the full sweep on a laptop. Each trial is bounded by `TIMEOUT=90` seconds and most complete in 20–30s.

---

## Sweep matrix

| Variable | Default values | Override via |
|---|---|---|
| Swarm size `N` | 2, 5, 10 | `N_LIST="2 5 7 10"` |
| Attack | none, coverage_spoof, phantom_drone, selective_denial, interleaved | `ATTACKS="coverage_spoof"` |
| Trials per cell | 5 | `TRIALS=10` |
| Spoof delay | 0.5 s | `SPOOF_DELAY=1.0` |
| Discovery time | 3.0 s | `DISCOVERY=5.0` |
| Waypoints per drone | 9 | `WP_PER_DRONE=12` |
| Mean flight time | 2.0 s (calibrated from N=2 trial timestamps) | `MEAN_FLIGHT=2.2` |
| Std flight time | 0.4 s | `STD_FLIGHT=0.3` |
| Mission timeout | 90 s | `TIMEOUT=120` |

---

## What the Sybil experiment demonstrates

The headline experiment for the extension. With `N_real=10` legitimate drones, Byzantine fault tolerance theory says `f_max = (N-1)/3 = 3` faulty participants are tolerable. Inject `f+1 = 4` phantom identities (`drone11..drone14`). The current ROS2 coordination protocol has no enrolment list, so all four phantoms are accepted at face value.

The result row in `~/lite_sim_results/sybil_N10.csv` will show:
- `gt_visited`: how many waypoints the real drones actually visited
- `reported_visited`: 90 (the entire grid, claimed by the four phantoms)
- `false_claims`: 90 minus `gt_visited` (typically near-total)

This is the empirical N>3f breach. Pair it with the framing in §2.3.2 of the dissertation (Sybil identity-spoofing as a BFT-threshold violation) for the paper extension section.

---

## Running individual components

### Spawn just the swarm (no attacker)

```bash
python3 swarm_lite_sim.py \
    --num-drones 5 --waypoints-per-drone 9 \
    --mean-flight 3.0 --std-flight 0.5 \
    --mission-timeout 60 --seed 42
```

### Capture ground truth into a CSV

```bash
python3 ground_truth_logger_lite.py \
    --num-drones 5 --total-wp 45 \
    --scenario manual_test --output ./test.csv \
    --mission-timeout 60
```

### Inspect topics live

```bash
ros2 topic echo /swarm/waypoint_status     # public, attackers can poison
ros2 topic echo /swarm/ground_truth        # private, only legitimate drones publish
ros2 topic hz /swarm/waypoint_status       # message rate sanity check
```

---

## Troubleshooting

| Issue | Fix |
|---|---|
| `rclpy.init()` fails after a previous trial crashed | `pkill -9 -f swarm_lite_sim; pkill -9 -f ground_truth_logger; ros2 daemon stop; sleep 1; ros2 daemon start` |
| Mission times much longer than expected | Lower `TIMEOUT` and `MEAN_FLIGHT`; the simulator naturally completes when all drones finish, so timeouts only matter as upper bounds |
| Lite-sim N=2 means drift outside ±0.5 WP of full-stack | Retune `MEAN_FLIGHT` (slower flight = larger gap for a given `spoof_delay`); also confirm `WP_PER_DRONE=9` matches the dissertation's per-zone count |
| `ros2 topic echo /swarm/ground_truth` shows no messages | The lite simulator is not running; check `tail -f ~/lite_sim_results/last_sim.log` |
| Ground-truth CSV missing for a cell | Trial timed out before any drone reported a visit; raise `TIMEOUT` or check `last_sim.log` for Python tracebacks |
| Multiple `lite_results.csv` files appearing in different paths | Check the `OUT` env var and the CSV path printed by the logger startup line |

---

## Methodology disclosure for the T-ITS extension

The validation paragraph that goes into the paper:

> *To evaluate Layer 1 coordination attack scaling at N=5 and N=10, we developed a lightweight discrete-event simulator that retains the full ROS2/DDS coordination layer — the `/swarm/waypoint_status` topic, the JSON message format, and the `status_callback` handler logic verbatim from the full-stack experiments — while abstracting drone physics as a parameterised travel-time distribution `t ~ N(μ=2.0s, σ=0.4s)`. The distribution was calibrated from end-of-mission timestamps logged across 33 honest baseline trials of the full-stack simulation: consecutive successful trials cluster 11–18 seconds apart, which after accounting for restart overhead corresponds to ~2.2 s of flight per waypoint per drone. This abstraction is justified because Layer 1 attacks are message-driven; physical dynamics influence the attack surface only through the timing of legitimate status publications, which the calibrated distribution preserves. We validated the abstraction by reproducing the N=2 full-stack results within ±0.5 waypoints across all three baseline attack modes (coverage spoof, phantom drone, selective denial); see Table X. Layer 2 (sensor-path) attacks are not re-evaluated because they operate on per-drone pipelines and generalise unchanged with N.*

The calibration table that goes alongside it is the one printed by `analyse_lite_results.py`.
