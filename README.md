# README Updates — Deskew Relay + Attack Infrastructure

These are the specific changes to make to your existing README.

---

## Changes to Section 7: ros_gz Bridge Config

Replace the bridge.yaml content with (note `points_raw` for LiDAR):

```yaml
- ros_topic_name: "/drone1/lidar/points_raw"
  gz_topic_name: "/drone1/lidar/points/points"
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  gz_type_name: "gz.msgs.PointCloudPacked"
  direction: GZ_TO_ROS

- ros_topic_name: "/drone1/imu/data"
  gz_topic_name: "/world/iris_warehouse/model/iris_with_gimbal/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS
```

> The LiDAR topic is renamed to `points_raw` so the deskew relay (Section 7b) can add per-point timestamps before LIO-SAM receives the data. Without this, LIO-SAM cannot deskew the scans and the map becomes streaky over time.

---

## NEW Section 7b: LiDAR Deskew Relay

The Gazebo VLP-16 bridge publishes PointCloud2 without a per-point `time` field. LIO-SAM requires this field for motion deskewing — correcting for the drone's movement during each LiDAR sweep. Without it, LIO-SAM prints `Point cloud timestamp not available, deskew function disabled` and the map accumulates registration errors.

The deskew relay sits between the bridge and LIO-SAM:

```
Bridge → /droneX/lidar/points_raw → [deskew relay adds 'time'] → /droneX/lidar/points → LIO-SAM
```

The relay adds a synthetic `time` field to each scan based on ring number (VLP-16 vertical beam index), distributing timestamps from 0 to 0.1s across the sweep. This is part of the `swarm_mission` package.

```bash
# Build (if not already built)
cd ~/ros2_ws && colcon build --packages-select swarm_mission && source install/setup.bash

# Run for each drone (in separate terminals, BEFORE LIO-SAM)
ros2 run swarm_mission lidar_deskew_relay --ros-args -p drone_ns:=drone1
ros2 run swarm_mission lidar_deskew_relay --ros-args -p drone_ns:=drone2
```

Verify the relay is working:

```bash
# Raw from bridge (no 'time' field)
ros2 topic echo /drone1/lidar/points_raw --field fields --once

# After relay (should include 'time' field)
ros2 topic echo /drone1/lidar/points --field fields --once

# Rate should match
ros2 topic hz /drone1/lidar/points      # ~2 Hz
ros2 topic hz /drone1/lidar/points_raw   # ~2 Hz
```

---

## Changes to Section 9: Two-Drone World Setup

Replace the bridge2.yaml content with (note `points_raw` for LiDAR):

```yaml
- ros_topic_name: "/drone2/lidar/points_raw"
  gz_topic_name: "/drone2/lidar/points/points"
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  gz_type_name: "gz.msgs.PointCloudPacked"
  direction: GZ_TO_ROS

- ros_topic_name: "/drone2/imu/data"
  gz_topic_name: "/world/iris_warehouse/model/iris_with_gimbal_2/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS
```

---

## Changes to Section 11: Full Startup Order — Two Drones

Add these new terminals between the bridge and LIO-SAM steps:

```bash
# Terminal 4 — ros_gz bridge drone 1
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$HOME/ros2_ws/bridge.yaml

# Terminal 5 — ros_gz bridge drone 2
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$HOME/ros2_ws/bridge2.yaml

# Terminal 5a — Deskew relay drone 1 (NEW — run BEFORE LIO-SAM)
source ~/ros2_ws/install/setup.bash
ros2 run swarm_mission lidar_deskew_relay --ros-args -p drone_ns:=drone1

# Terminal 5b — Deskew relay drone 2 (NEW — run BEFORE LIO-SAM)
source ~/ros2_ws/install/setup.bash
ros2 run swarm_mission lidar_deskew_relay --ros-args -p drone_ns:=drone2

# Terminal 6 — MAVROS drone 1 (port 14551)
# ... (unchanged)

# Terminal 10 — LIO-SAM drone 1 (unchanged — still reads /drone1/lidar/points)
# Terminal 11 — LIO-SAM drone 2 (unchanged — still reads /drone2/lidar/points)
```

> **Launch order matters:** The deskew relay must be running before LIO-SAM starts, otherwise LIO-SAM receives raw scans without the `time` field and disables deskewing. If you see the deskew warning, restart LIO-SAM (the relay can stay running).

---

## Changes to Section 15: Waypoints

The waypoints.yaml now uses 18 waypoints (9 per drone) with zones split along the x-axis:

- **Drone 1 (IDs 0-8):** Open area, world x = -8 to -13
- **Drone 2 (IDs 9-17):** Shelving area, world x = 1 to 9

Coordinate conversion reminder: `local = world - spawn`
- Drone 1 spawn (-6, 0): world x=-10 → local x=-4 (backward into open area)
- Drone 2 spawn (-3, 0): world x=6 → local x=9 (forward into shelves)

---

## NEW Section 16: Layer 2 Attacks — Navigation Pipeline

### Point Cloud Injection (pointcloud_injector.py)

Publishes crafted PointCloud2 messages to the target drone's LiDAR topic, injecting false geometry (a phantom wall) into LIO-SAM's map.

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

**Topics:**

| Drone | LiDAR topic (attack target) | Pose topic (for coordinate transform) |
|-------|---------------------------|--------------------------------------|
| drone1 | /drone1/lidar/points | /mavros/local_position/pose |
| drone2 | /drone2/lidar/points | /drone2/mavros/local_position/pose |

**Findings:** The phantom wall appears in the LIO-SAM point cloud map (visible in RViz) but does not cause significant odometry drift when published as separate scans. LIO-SAM's ICP scan matching + IMU preintegration prior are robust enough to absorb the additional geometry. The wall is accepted into the map as new features but does not bias the pose estimate. This is documented as a negative result — topic-level point cloud injection alone is insufficient to corrupt LIO-SAM's localisation when the real sensor data provides stronger geometric constraints.

### QoS Profile Poisoning (qos_poisoner.py)

Creates RELIABLE subscribers on LIO-SAM's BEST_EFFORT odometry topic, forcing the DDS middleware to satisfy the stricter QoS policy. This causes the odometry output rate to degrade, starving the vision pose bridge below ArduCopter's EKF3 minimum threshold (0.5 Hz), which triggers a Land Mode failsafe.

**Attacker model:** Network participant that can SUBSCRIBE to any DDS topic (SROS2 disabled). No interception, modification, or reconfiguration of victim systems required.

**How it works:**
1. DDS publishers must satisfy ALL subscribers' QoS policies
2. LIO-SAM publishes odometry as BEST_EFFORT (fire-and-forget, fast)
3. The attacker creates RELIABLE subscribers, forcing buffering and acknowledgement
4. The publisher slows down to satisfy the RELIABLE contract
5. Downstream consumers (MAVROS vision_pose bridge) receive fewer messages
6. ArduCopter's EKF3 detects the rate drop and triggers a failsafe

```bash
# Automated attack with metric collection (3 phases: baseline → attack → recovery)
source ~/ros2_ws/install/setup.bash
ros2 run swarm_mission qos_poisoner --ros-args \
  -p target_drone:=drone1 \
  -p baseline_duration:=15.0 \
  -p attack_duration:=45.0 \
  -p recovery_duration:=15.0 \
  -p num_reliable_subs:=5
```

**Parameters:**

| Parameter | Default | Purpose |
|-----------|---------|---------|
| target_drone | drone1 | Which drone to attack |
| baseline_duration | 15.0 | Seconds of pre-attack measurement |
| attack_duration | 45.0 | Seconds with RELIABLE subscribers active |
| recovery_duration | 15.0 | Seconds of post-attack measurement |
| num_reliable_subs | 5 | Number of RELIABLE subscribers to create |
| rate_window | 3.0 | Sliding window (s) for Hz calculation |
| output_dir | ~ | Directory for CSV output |

**Output:** The node automatically collects metrics and saves to `~/qos_attack_metrics_<drone>_<timestamp>.csv` with columns: timestamp, phase, phase_elapsed_s, vision_pose_hz, odom_hz.

**Manual attack (single command, no metrics):**

```bash
# This alone can trigger the attack — just subscribe with RELIABLE QoS
ros2 topic echo /drone1/lio_sam/mapping/odometry_incremental --qos-reliability reliable
```

**Metrics to collect:**

| Metric | How to measure | Expected result |
|--------|---------------|-----------------|
| Vision pose rate (Hz) | CSV vision_pose_hz column | Drops from ~10 Hz to <0.5 Hz |
| Time to EKF failsafe | Time from attack start to Land Mode | Seconds |
| Rate drop percentage | (baseline - attack) / baseline | >90% |
| Recovery time | Time from attack end to rate restored | Seconds |
| EKF lane switches | ArduPilot logs (EKF3 lane switch msgs) | Increases during attack |
