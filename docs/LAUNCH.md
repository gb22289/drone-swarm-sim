# Launch — Single Drone, Two Drones, GPS-Denied Bootstrap

Assumes [SETUP.md](SETUP.md) is complete. Launch order matters: `/clock` from Gazebo must be flowing before any node with `use_sim_time:=true` starts, and the `lidar_deskew_shim` must be running before LIO-SAM.

## Single-Drone Launch (7 terminals)

Run each in a separate terminal in this order:

```bash
# Terminal 1 — Gazebo
cd ~/sim/ardupilot_gazebo
gz sim worlds/iris_warehouse.sdf -r

# Terminal 2 — ros_gz bridge (MUST come early — publishes /clock, lidar, imu)
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:=$HOME/ros2_ws/bridge.yaml
# Verify: ros2 topic hz /clock  (should be hundreds of Hz)

# Terminal 3 — SITL drone 1
cd ~/sim/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console -I0

# --- In Drone 1 MAVProxy console, add localhost output for MAVROS ---
#   output add 127.0.0.1:14551

# Terminal 4 — MAVROS (drone 1, port 14551)
source /opt/ros/humble/setup.bash
ros2 launch mavros apm.launch fcu_url:=udp://:14551@localhost \
  tgt_system:=1 \
  config_yaml:=$HOME/ros2_ws/mavros_drone1.yaml

# Terminal 5 — LiDAR deskew shim (MUST come before LIO-SAM)
source ~/ros2_ws/install/setup.bash
ros2 run swarm_mission lidar_deskew_shim --ros-args \
  -p input_topic:=/drone1/lidar/points \
  -p output_topic:=/drone1/lidar/points_timed \
  -p scan_period:=0.1
# Verify: ros2 topic echo /drone1/lidar/points_timed --field fields --once
#         should list 'time' as a field

# Terminal 6 — LIO-SAM (reads /drone1/lidar/points_timed)
source ~/ros2_ws/install/setup.bash
ros2 launch lio_sam run.launch.py \
  params_file:=$HOME/ros2_ws/src/LIO-SAM/config/params_drone1.yaml \
  namespace:=drone1
# Watch for absence of "Point cloud timestamp not available" warning
# Let LIO-SAM sit stationary for ~10 seconds for IMU bias estimation

# Terminal 7 — LIO-SAM → MAVROS bridge (drone 1)
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/lio_mavros_bridge.py --ros-args \
  -p drone_ns:=drone1 \
  -r /drone1/mavros/vision_pose/pose:=/mavros/vision_pose/pose
```

> The remap on the bridge command is needed because single-drone MAVROS subscribes on `/mavros/vision_pose/pose` (no `/drone1/` prefix), while the bridge by default publishes to `/drone1/mavros/vision_pose/pose`. Without the remap, vision_pose has 1 subscriber and 0 publishers.

---

## Two-Drone Launch (15 terminals)

Requires the two-drone world setup from [SETUP.md §10](SETUP.md#10-two-drone-world-setup). Each drone needs its own SITL instance, bridge, MAVROS, deskew shim, static TF publishers, LIO-SAM, and bridge node — all namespaced separately.

### Namespace Architecture

| Component | Drone 1 | Drone 2 |
|---|---|---|
| LiDAR raw (post-bridge) | `/drone1/lidar/points` | `/drone2/lidar/points` |
| LiDAR timed (post-shim) | `/drone1/lidar/points_timed` | `/drone2/lidar/points_timed` |
| IMU topic | `/drone1/imu/data` | `/drone2/imu/data` |
| Odometry | `/drone1/lio_sam/mapping/odometry_incremental` | `/drone2/lio_sam/mapping/odometry_incremental` |
| Vision pose | `/drone1/mavros/vision_pose/pose` | `/drone2/mavros/vision_pose/pose` |
| MAVROS state | `/mavros/state` | `/drone2/mavros/state` |
| SITL port (MAVROS) | 14551 | 14561 |
| MAV_SYSID | 1 (default) | **2** (must set manually) |
| TF base frame | `drone1/base_link` | `drone2/base_link` |
| TF lidar frame | `drone1/lidar_link` | `drone2/lidar_link` |

### Launch sequence

```bash
# Terminal 1 — Gazebo (loads both drone models)
cd ~/sim/ardupilot_gazebo
gz sim worlds/iris_warehouse.sdf -r

# Terminal 2 — ros_gz bridge drone 1 (brings up /clock — MUST come early)
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:=$HOME/ros2_ws/bridge.yaml

# Terminal 3 — ros_gz bridge drone 2 (no /clock, already bridged by drone 1)
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:=$HOME/ros2_ws/bridge2.yaml

# Terminal 4 — SITL drone 1 (instance 0, ports 14550/14551)
cd ~/sim/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console -I0

# Terminal 5 — SITL drone 2 (instance 1, ports 14560/14561)
cd ~/sim/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console -I1

# --- MAVProxy output + system ID setup (run in each MAVProxy console) ---
# IMPORTANT: sim_vehicle.py only forwards MAVLink to the Windows host IP.
# MAVROS runs inside WSL and needs a localhost output. Run these BEFORE
# launching MAVROS, in the correct MAVProxy console for each drone.
#
# Drone 1 MAVProxy console (--map --console window):
#   output add 127.0.0.1:14551
#
# Drone 2 MAVProxy console (-I1 window):
#   output add 127.0.0.1:14561
#   param set MAV_SYSID 2
#   param save
#
# MAV_SYSID survives param save; only need to set it once.
# `output add` does NOT survive SITL reboots; re-add after every reboot.

# Terminal 6 — MAVROS drone 1 (port 14551)
source /opt/ros/humble/setup.bash
ros2 launch mavros apm.launch \
  fcu_url:=udp://:14551@localhost \
  tgt_system:=1 \
  config_yaml:=$HOME/ros2_ws/mavros_drone1.yaml

# Terminal 7 — MAVROS drone 2 (port 14561)
source /opt/ros/humble/setup.bash
ros2 launch mavros apm.launch \
  fcu_url:=udp://:14561@localhost \
  tgt_system:=2 \
  namespace:=drone2/mavros \
  config_yaml:=$HOME/ros2_ws/mavros_drone2.yaml

# Terminal 8 — Deskew shim drone 1 (run BEFORE LIO-SAM)
source ~/ros2_ws/install/setup.bash
ros2 run swarm_mission lidar_deskew_shim --ros-args \
  -p input_topic:=/drone1/lidar/points \
  -p output_topic:=/drone1/lidar/points_timed \
  -p scan_period:=0.1

# Terminal 9 — Deskew shim drone 2 (run BEFORE LIO-SAM)
source ~/ros2_ws/install/setup.bash
ros2 run swarm_mission lidar_deskew_shim --ros-args \
  -p input_topic:=/drone2/lidar/points \
  -p output_topic:=/drone2/lidar/points_timed \
  -p scan_period:=0.1

# Terminal 10 — Static TF drone 1
source /opt/ros/humble/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link drone1/base_link &
ros2 run tf2_ros static_transform_publisher 0 0 0.1 0 0 0 drone1/base_link drone1/lidar_link

# Terminal 11 — Static TF drone 2
source /opt/ros/humble/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link drone2/base_link &
ros2 run tf2_ros static_transform_publisher 0 0 0.1 0 0 0 drone2/base_link drone2/lidar_link

# Terminal 12 — LIO-SAM drone 1 (reads points_timed)
# IMPORTANT: Use namespace:= argument to run.launch.py (NOT PushRosNamespace).
# LIO-SAM's run.launch.py sets namespace= on each Node directly, which
# overrides PushRosNamespace. Without this, both drones' LIO-SAM nodes
# collide in the root namespace and crash.
source ~/ros2_ws/install/setup.bash
ros2 launch lio_sam run.launch.py \
  params_file:=$HOME/ros2_ws/src/LIO-SAM/config/params_drone1.yaml \
  namespace:=drone1

# Terminal 13 — LIO-SAM drone 2 (reads points_timed)
source ~/ros2_ws/install/setup.bash
ros2 launch lio_sam run.launch.py \
  params_file:=$HOME/ros2_ws/src/LIO-SAM/config/params_drone2.yaml \
  namespace:=drone2

# Terminal 14 — LIO-SAM → MAVROS bridge drone 1
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/lio_mavros_bridge.py --ros-args \
  -p drone_ns:=drone1 \
  -r /drone1/mavros/vision_pose/pose:=/mavros/vision_pose/pose

# Terminal 15 — LIO-SAM → MAVROS bridge drone 2
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/lio_mavros_bridge.py --ros-args -p drone_ns:=drone2
```

> **Launch order recap:** Gazebo → bridge (`/clock` flowing) → SITL → MAVROS → deskew shims → static TFs → LIO-SAM → lio_mavros_bridge. If LIO-SAM starts before its shim, it caches a "no time field" decision and disables deskew until restart.

### Verify both drones are up

```bash
# /clock flowing
ros2 topic info /clock --verbose | grep "Publisher count"   # = 1

# Sensors flowing
ros2 topic hz /drone1/lidar/points         # ~1Hz (raw)
ros2 topic hz /drone1/lidar/points_timed   # ~1Hz (post-shim)
ros2 topic hz /drone2/lidar/points         # ~1Hz
ros2 topic hz /drone2/lidar/points_timed   # ~1Hz
ros2 topic hz /drone1/imu/data             # ~500Hz
ros2 topic hz /drone2/imu/data             # ~500Hz

# Deskew shim engaged — 'time' field present
ros2 topic echo /drone1/lidar/points_timed --field fields --once
# Must include: PointField(name='time', ...)

# LIO-SAM nodes properly namespaced
ros2 node list | grep lio
# Should show /drone1/lio_sam_* and /drone2/lio_sam_*

# MAVROS connected
ros2 topic echo /mavros/state --once        # connected: true
ros2 topic echo /drone2/mavros/state --once # connected: true

# Vision pose reaching MAVROS
ros2 topic info /mavros/vision_pose/pose
ros2 topic info /drone2/mavros/vision_pose/pose
```

---

## GPS-Denied Bootstrap Procedure

Since LIO-SAM needs motion to initialise, use GPS briefly to get airborne. Run these in **each drone's MAVProxy console** separately. Always run `param fetch` first and wait for it to respond before setting parameters.

### Step 1 — Enable GPS and take off (both drones)

```bash
# In each drone's MAVProxy console:
param fetch
param set GPS1_TYPE 1
param set EK3_SRC1_POSXY 3
param set EK3_SRC1_VELXY 3
param set EK3_SRC1_YAW 1
param set VISO_TYPE 0
mode guided
arm throttle
takeoff 3
```

### Step 2 — Wait for LIO-SAM to initialise

```bash
ros2 topic hz /drone1/lio_sam/mapping/odometry_incremental
ros2 topic hz /drone2/lio_sam/mapping/odometry_incremental
ros2 topic hz /mavros/vision_pose/pose              # ~20Hz
ros2 topic hz /drone2/mavros/vision_pose/pose       # ~20Hz
```

### Step 3 — Switch to LIO-SAM nav (both drones)

Once odometry and vision_pose are flowing, in each MAVProxy console:

```bash
param set VISO_DELAY_MS 50
param set EK3_SRC1_POSXY 6
param set EK3_SRC1_VELXY 6
param set EK3_SRC1_VELZ 0
param set EK3_SRC1_YAW 6
param set GPS1_TYPE 0
param set VISO_TYPE 1
```

You should see **"EKF3 IMU0 is using external nav data"** in the MAVProxy console — appears once per EKF init.

### Step 4 — Verify full pipeline

```bash
ros2 topic hz /drone1/lio_sam/mapping/odometry_incremental
ros2 topic hz /drone2/lio_sam/mapping/odometry_incremental
ros2 topic hz /drone1/mavros/vision_pose/pose
ros2 topic hz /drone2/mavros/vision_pose/pose
ros2 topic echo /mavros/state --once                # armed: true, guided: true
ros2 topic echo /drone2/mavros/state --once
```

> **Note:** For the dissertation experiments, trials use GPS-hold (`VISO_TYPE=0`, `GPS1_TYPE=1`) rather than the full GPS-denied switch, to isolate the attack's effect on the SLAM pipeline from feedback dynamics in the EKF3 fusion of vision_pose. See dissertation Section 3.5.3 methodology subsection.

---

## Pre-flight Check

Before any trial:

```bash
./scripts/preflight_check.sh
```

The script verifies IMU rate, LiDAR rate, vision_pose rate, drone hover position, LIO-SAM odometry magnitude, and the absence of zombie attacker/logger processes. Exit code 0 means safe to proceed.

---

For attack-specific run instructions, proceed to [ATTACKS.md](ATTACKS.md).
