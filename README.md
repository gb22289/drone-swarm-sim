# GPS-Denied Drone Swarm — Security Analysis

Code accompanying the MEng dissertation **"Breaking Trust in the Swarm: A Security Analysis of ROS2-Based Cooperative Drone Systems"** (University of Bristol, COMSM0052).

This repository contains the attack and defence nodes, configuration, experiment scaffolding, and raw data used to characterise vulnerabilities and unintended defences in a ROS2-based GPS-denied drone swarm. The simulation stack comprises Gazebo Harmonic, ArduCopter SITL, MAVROS, and LIO-SAM.

---

## Repository Layout

| Path | Purpose |
|------|---------|
| `swarm_mission/` | ROS2 package — all attack and defence nodes, plus mission/coordination logic |
| `lio_mavros_bridge.py` | 20 Hz republisher bridging LIO-SAM odometry to MAVROS `vision_pose` |
| `lio_sam_patches/` | Diffs to apply over upstream LIO-SAM (clock-jump dt-guards + KDTree bounds checks) |
| `lio_sam_config/` | LIO-SAM YAML configurations for drone1 and drone2 |
| `configs/` | ros_gz_bridge, MAVROS, and Gazebo SDF configurations |
| `scripts/` | Experiment automation, pre-flight validation, plotting, audit |
| `data/` | Raw experimental data (CSVs) |

---

## Attack and Defence Nodes

All nodes live in `swarm_mission/swarm_mission/` and follow a consistent CLI:

```bash
ros2 run swarm_mission <node_name> --ros-args -p <param>:=<value>
```

| Node | Layer | Purpose |
|------|-------|---------|
| `network_attacker` | 1 | Coordination-layer attacker (modes: `coverage_spoof`, `phantom_drone`, `selective_denial`) |
| `lidar_manipulator` | 2 | Gradual scan-rotation attack on the LiDAR topic |
| `pointcloud_injector` | 2 | Phantom-wall injection — negative-result attack |
| `imu_injector` | 2 | Reused-timestamp IMU injection — blocked by dt-guard |
| `imu_injector_v2` | 2 | Advancing-timestamp IMU injection — bypasses dt-guard, blocked by upstream LIO-SAM mechanisms |
| `qos_poisoner` | 2 | DDS QoS-incompatibility attack — blocked by DDS protocol-level isolation |
| `slam_failure_logger` | — | Detector — sustained-span pattern matching on LIO-SAM rosout for TTF measurement |
| `lidar_deskew_shim` | — | Per-point timestamp synthesis to enable LIO-SAM deskew on Gazebo's `gpu_lidar` |
| `waypoint_navigator` | — | Mission node (honest or Byzantine); each drone runs an instance |
| `ground_truth_logger` | — | Independent measurement of mission outcomes for Layer 1 evaluation |

---

## Building

Assumes Ubuntu 22.04 LTS with ROS2 Humble, Gazebo Harmonic, and ArduCopter SITL already installed. Full installation procedure is in dissertation Chapter 3.2.

```bash
# 1. Set up ROS2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. Clone upstream dependencies
git clone https://github.com/TixiaoShan/LIO-SAM.git -b ros2
git clone https://github.com/gazebosim/ros_gz.git -b humble
# (ArduPilot + ardupilot_gazebo into ~/sim/ separately)

# 3. Apply LIO-SAM patches
cd ~/ros2_ws/src/LIO-SAM
patch -p1 < /path/to/this/repo/lio_sam_patches/imu_preintegration_dt_guards.patch
patch -p1 < /path/to/this/repo/lio_sam_patches/mapoptmization_kdtree_check.patch

# 4. Copy submission contents into the workspace
cp -r /path/to/this/repo/swarm_mission ~/ros2_ws/src/
cp /path/to/this/repo/lio_sam_config/*.yaml ~/ros2_ws/src/LIO-SAM/config/
cp /path/to/this/repo/lio_mavros_bridge.py ~/ros2_ws/src/

# 5. Build
cd ~/ros2_ws
export GZ_VERSION=harmonic
colcon build
source install/setup.bash
```

---

## Running

Launch order matters. `/clock` from Gazebo must be flowing before any node with `use_sim_time:=true` starts, and the `lidar_deskew_shim` must be running before LIO-SAM. The dissertation Section 3.2 documents the full sequence; the abridged single-drone version:

```bash
# Terminal 1 — Gazebo
gz sim worlds/iris_warehouse.sdf -r

# Terminal 2 — ros_gz bridge (publishes /clock)
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:=/path/to/configs/bridge.yaml

# Terminal 3 — ArduCopter SITL
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console -I0

# Terminal 4 — MAVROS

# Terminal 5 — LiDAR deskew shim (must precede LIO-SAM)
ros2 run swarm_mission lidar_deskew_shim --ros-args \
  -p input_topic:=/drone1/lidar/points \
  -p output_topic:=/drone1/lidar/points_timed \
  -p scan_period:=0.1

# Terminal 6 — LIO-SAM

# Terminal 7 — LIO-SAM → MAVROS bridge
python3 ~/ros2_ws/src/lio_mavros_bridge.py --ros-args \
  -p drone_ns:=drone1 \
  -r /drone1/mavros/vision_pose/pose:=/mavros/vision_pose/pose
```

### Pre-flight check

Before any trial, verify stack health:

```bash
./scripts/preflight_check.sh
```

Validates IMU and LiDAR rates, vision_pose flow, drone hover position, LIO-SAM odometry magnitude, and the absence of zombie processes from earlier trials.

### Running an attack — examples

```bash
# Layer 1 — coordination spoof (other modes: phantom_drone, selective_denial)
python3 ~/ros2_ws/src/swarm_mission/swarm_mission/network_attacker.py --ros-args \
  -p attack:=coverage_spoof \
  -p target_drone:=drone2 \
  -p discovery_time:=5.0 \
  -p spoof_delay:=0.5

# Layer 2 — gradual scan rotation, with TTF logged via the sweep runner
./scripts/sweep_runner.sh 1.0 1   # 1.0 °/s, run number 1

# Layer 2 — IMU injection with advancing timestamps (defeats dt-guard)
ros2 run swarm_mission imu_injector_v2 --ros-args \
  -p target_drone:=drone1 \
  -p mode:=bias \
  -p accel_bias_x:=5.0 \
  -p baseline_duration:=15.0 \
  -p attack_duration:=45.0 \
  -p recovery_duration:=15.0
```

---

## Data Files

| File | Contents |
|------|----------|
| `data/scan_sweep_summary.csv` | TTF measurements for the rotation-rate sweep (Section 3.5.3), pre and post KDTree-patch |
| `data/layer1_mission_results.csv` | Per-waypoint ground-truth measurements for Layer 1 attacks (Section 3.4) |
| `data/imu_injection_v2_metrics_*.csv` | LIO-SAM and drone drift trajectories during advancing-stamp IMU injection (Section 3.5.5) |

---

## Reproducing the Figures

The dissertation's primary figure (rotation-rate vs. time-to-failure) is regenerated from `data/scan_sweep_summary.csv`:

```bash
python3 scripts/plot_rotation_sweep.py \
  --csv data/scan_sweep_summary.csv \
  --out fig_rotation_sweep \
  --test-window-s 180
```

Output: `fig_rotation_sweep.pdf` and `fig_rotation_sweep.png`.

---

## LIO-SAM Patches

Two patches in `lio_sam_patches/` modify upstream LIO-SAM:

1. **`imu_preintegration_dt_guards.patch`** — Adds `dt > 0` and `dt < 0.02` guards to the three IMU-integration paths in `imuPreintegration.cpp`. Required for stable operation under the multi-SITL / single-Gazebo clock-synchronisation regime documented in dissertation Section 3.2.4. Without these guards, `gtsam::IndeterminantLinearSystemException` crashes occur on clock jumps.

2. **`mapoptmization_kdtree_check.patch`** — Adds bounds checks at four `setInputCloud` call sites in `mapOptmization.cpp` (scan-to-map hot path, global-map visualisation, loop-closure detection, surrounding-keyframe extraction). Eliminates the dominant denial-of-process crashes observed under the scan-rotation attack at low rotation rates. Discussed as a partial defence in Section 3.5.3 — closes ~89% of crashes; one ICP-internal site remains uncovered.

Both patches are also documented inline in their respective `.patch` files.

---

## Methodology Notes

- **Trials use GPS-hold position** (`VISO_TYPE=0`, `GPS1_TYPE=1`) to isolate the attack's effect on the SLAM pipeline from feedback dynamics in ArduCopter's EKF3. The attacker still publishes to `/droneN/lidar/points` and LIO-SAM still processes those scans; only the EKF3 fusion of vision_pose is bypassed.
- **Per-trial cold restarts** are recommended — LIO-SAM's factor graph and accumulated map state can carry corruption from prior trials. The `state_audit.sh` script catalogues lingering state if cleanups appear incomplete.
- **Real-time factor (RTF)** of approximately 47% on the development hardware (WSL2 / consumer laptop) yields a wall-clock LiDAR rate of ~0.94 Hz against a 2 Hz sim-time configuration. Higher-RTF deployments should reproduce qualitatively but with shorter time-to-failure.

---

## Citation

If you use this code, please cite:

> Mohai, P. *Breaking Trust in the Swarm: A Security Analysis of ROS2-Based Cooperative Drone Systems.* MEng dissertation, University of Bristol, 2026.

---

## License

Original code is released under the MIT License (see `LICENSE`). Patches to LIO-SAM are derivative works of LIO-SAM and inherit its BSD-3-Clause license.
