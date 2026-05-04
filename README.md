# GPS-Denied Drone Swarm — Security Analysis (Code Submission)

This repository accompanies the MEng dissertation
*"Breaking Trust in the Swarm: A Security Analysis of ROS2-Based
Cooperative Drone Systems"* (University of Bristol, COMSM0052).

## Repository Layout

| Path | Purpose |
|------|---------|
| `swarm_mission/` | ROS2 package — all attack and defense nodes |
| `lio_mavros_bridge.py` | LIO-SAM odometry → MAVROS vision_pose republisher |
| `lio_sam_patches/` | Diffs to apply over upstream LIO-SAM |
| `lio_sam_config/` | LIO-SAM YAML configurations |
| `configs/` | ros_gz_bridge, MAVROS, and Gazebo configs |
| `scripts/` | Experiment automation, validation, audit scripts |
| `data/` | Experimental data (CSVs) |

## Building

Assumes ROS2 Humble + Gazebo Harmonic + ArduCopter SITL on
Ubuntu 22.04 LTS. See the dissertation Chapter 3.2 for the
full installation procedure, or follow these high-level steps:

```bash
# 1. Set up ROS2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. Clone upstream packages
git clone https://github.com/TixiaoShan/LIO-SAM.git -b ros2
git clone https://github.com/gazebosim/ros_gz.git -b humble
# (and ArduPilot + ardupilot_gazebo into ~/sim/)

# 3. Apply LIO-SAM patches
cd ~/ros2_ws/src/LIO-SAM
patch -p1 < /path/to/this/repo/lio_sam_patches/imu_preintegration_dt_guards.patch
patch -p1 < /path/to/this/repo/lio_sam_patches/mapoptmization_kdtree_check.patch

# 4. Copy this submission's swarm_mission package and configs
cp -r /path/to/this/repo/swarm_mission ~/ros2_ws/src/
cp /path/to/this/repo/lio_sam_config/*.yaml ~/ros2_ws/src/LIO-SAM/config/
cp /path/to/this/repo/lio_mavros_bridge.py ~/ros2_ws/src/

# 5. Build
cd ~/ros2_ws
export GZ_VERSION=harmonic
colcon build
source install/setup.bash
```

## Running the experiments

See `scripts/preflight_check.sh` for the pre-trial validation
protocol and `scripts/sweep_runner.sh` for the rotation-sweep
attack automation. Detailed launch order is in the dissertation
Section 3.2.

## Reproducing the figures

```bash
python3 scripts/plot_rotation_sweep.py \
  --csv data/scan_sweep_summary.csv \
  --out fig_rotation_sweep \
  --test-window-s 180
```

## Citation

If you use this code, please cite the dissertation.

## License

Original code is released under the MIT License. Patches to
LIO-SAM inherit LIO-SAM's BSD-3-Clause license.
