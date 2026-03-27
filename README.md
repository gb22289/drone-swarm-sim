# GPS-Denied Drone Swarm Simulation
## LiDAR-Inertial Odometry with ArduCopter SITL + Gazebo Harmonic + ROS2 Humble

---

## System Requirements

- Ubuntu 22.04 LTS (Jammy) — **do not upgrade to 24.04**
- Gazebo Sim 8.x (Harmonic)
- ArduCopter SITL
- ROS2 Humble
- ARM64 or x86_64

---

## 1. ArduPilot SITL + Gazebo Setup

Assumes `ardupilot` and `ardupilot_gazebo` are cloned under `~/sim/`.

### Launch Gazebo (warehouse world recommended for SLAM)

```bash
cd ~/sim/ardupilot_gazebo
gz sim worlds/iris_warehouse.sdf -r
```

> The `-r` flag auto-runs the simulation. Without it, sensors won't publish.

### Launch SITL (in a separate terminal, after Gazebo is fully loaded)

```bash
cd ~/sim/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```

---

## 2. ArduPilot Parameter Configuration

Run these in MAVProxy after SITL connects:

```bash
# Disable GPS
param set GPS1_TYPE 0
param set GPS2_TYPE 0

# EKF3 — use External Nav (LIO-SAM) as position source
param set EK3_SRC1_POSXY 6
param set EK3_SRC1_VELXY 6
param set EK3_SRC1_POSZ 1        # barometer for altitude
param set EK3_SRC1_YAW 1         # compass for yaw
param set EK3_SRC1_VELZ 0

# Enable EKF3
param set AHRS_EKF_TYPE 3
param set EK3_ENABLE 1
param set EK3_POSNE_M_NSE 0.1

# Enable Visual Odometry input
param set VISO_TYPE 1

# Save
param save nav.parm
```

> **Note:** On first boot, temporarily re-enable GPS to arm and take off
> (`GPS1_TYPE 1`, `EK3_SRC1_POSXY 3`, `EK3_SRC1_VELXY 3`), let LIO-SAM
> initialize, then switch back to external nav mid-flight.

---

## 3. ROS2 Humble Installation

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu jammy main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-humble-mavros ros-humble-mavros-extras

# GeographicLib datasets (required by MAVROS)
sudo wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash install_geographiclib_datasets.sh

# Add to .bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 4. ros_gz Bridge (Gazebo Harmonic ↔ ROS2 Humble)

Build from source (binary package not available for this combination):

```bash
source /opt/ros/humble/setup.bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/gazebosim/ros_gz.git -b humble
cd ~/ros2_ws
export GZ_VERSION=harmonic
sudo rosdep init && rosdep update
rosdep install -r --from-paths src -i -y --rosdistro humble
colcon build --cmake-args -DBUILD_TESTING=OFF
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 5. LIO-SAM Installation

```bash
sudo apt install -y libeigen3-dev libpcl-dev ros-humble-pcl-ros \
  ros-humble-pcl-conversions ros-humble-perception-pcl \
  ros-humble-vision-opencv ros-humble-xacro

# GTSAM
sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt update && sudo apt install -y libgtsam-dev libgtsam-unstable-dev

# Clone and build
cd ~/ros2_ws/src
git clone https://github.com/TixiaoShan/LIO-SAM.git -b ros2
cd ~/ros2_ws
colcon build --packages-select lio_sam --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

### LIO-SAM Configuration (`~/ros2_ws/src/LIO-SAM/config/params.yaml`)

Key values to set:

```yaml
pointCloudTopic: "/lidar/points"
imuTopic: "/imu/data"
sensor: velodyne
N_SCAN: 16
Horizon_SCAN: 1800
lidarMinRange: 0.1
lidarMaxRange: 100.0
imuType: 0
imuRate: 100.0

lidarFrame: "lidar_link"
baselinkFrame: "base_link"
odometryFrame: "odom"
mapFrame: "map"

extrinsicTrans: [0.0, 0.0, 0.0]
extrinsicRot: [1.0, 0.0, 0.0,
               0.0, 1.0, 0.0,
               0.0, 0.0, 1.0]
extrinsicRPY: [1.0, 0.0, 0.0,
               0.0, 1.0, 0.0,
               0.0, 0.0, 1.0]
```

### CRITICAL: Sim Clock + IMU Extrinsics for Gazebo

When running in Gazebo simulation, the following settings are **required** in your params YAML:

```yaml
use_sim_time: true
```

> `use_sim_time: true` must be inside the **single** `/**:  ros__parameters:` block. Do NOT create a separate `/**:` block for it — YAML will silently drop the first block. Verify with `head -6 params_droneN.yaml`.

**Gazebo IMU extrinsics:** Gazebo's IMU uses NED convention (gravity reports as `z: -9.8`). LIO-SAM with `MakeSharedU()` expects ENU (gravity as `z: +9.8`). The `extrinsicRot` must flip Y and Z to convert NED→ENU. However, `extrinsicRPY` must be **identity** because Gazebo's orientation quaternion already reports "level" correctly — flipping it makes LIO-SAM think the drone is upside down.

```yaml
# CORRECT for Gazebo Harmonic IMU:
extrinsicRot: [1.0,  0.0,  0.0,
               0.0, -1.0,  0.0,
               0.0,  0.0, -1.0]
extrinsicRPY: [1.0, 0.0, 0.0,
               0.0, 1.0, 0.0,
               0.0, 0.0, 1.0]
```

> **Symptom of wrong extrinsics:** LIO-SAM odometry z-value plummets to -300+ within seconds, orientation quaternion x ≈ 0.97 (drone appears flipped). Point cloud scatters everywhere in RViz.

### CRITICAL: Feature Thresholds for Indoor Environments

Warehouse environments have many planar surfaces but few geometric edges. Lower the feature thresholds so map optimization can run:

```yaml
edgeThreshold: 0.5              # default 1.0 — lower to extract more edges
edgeFeatureMinValidNum: 2       # default 10 — warehouses have few edges
surfFeatureMinValidNum: 50      # default 100 — plenty of planar surfaces
```

> **Symptom:** "Not enough features! Only N edge and M planar features available" → map optimization stops → IMU drifts unchecked → "Large velocity, reset IMU-preintegration!" every ~3 seconds → eventual GTSAM crash.

---

## 6. VLP-16 LiDAR Sensor (Gazebo Model)

> ⚠️ Add the LiDAR block to **`iris_with_standoffs/model.sdf`**, NOT `iris_with_gimbal/model.sdf`.
> `iris_with_standoffs` is the nested sub-model that contains `base_link`. Adding it to `iris_with_gimbal` causes a naming collision and the sensor never fires.

Add before the final `</model>` in `~/sim/ardupilot_gazebo/models/iris_with_standoffs/model.sdf`:

```xml
<!-- VLP-16 LiDAR -->
<link name="lidar_link">
  <pose>0 0 0.1 0 0 0</pose>
  <inertial>
    <mass>0.1</mass>
    <inertia>
      <ixx>0.000166667</ixx>
      <iyy>0.000166667</iyy>
      <izz>0.000166667</izz>
    </inertia>
  </inertial>
  <sensor name="lidar" type="gpu_lidar">
    <pose>0 0 0 0 0 0</pose>
    <topic>/lidar/points</topic>
    <gz_frame_id>lidar_link</gz_frame_id>
    <update_rate>2</update_rate>
    <lidar>
      <scan>
        <horizontal>
          <samples>1800</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159265</min_angle>
          <max_angle>3.14159265</max_angle>
        </horizontal>
        <vertical>
          <samples>16</samples>
          <resolution>1</resolution>
          <min_angle>-0.261799</min_angle>
          <max_angle>0.261799</max_angle>
        </vertical>
      </scan>
      <range>
        <min>0.1</min>
        <max>100.0</max>
        <resolution>0.001</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </lidar>
    <always_on>1</always_on>
    <visualize>true</visualize>
  </sensor>
</link>
<joint name="lidar_joint" type="fixed">
  <parent>base_link</parent>
  <child>lidar_link</child>
</joint>
```

> **Sensor type must be `gpu_lidar`** — Gazebo Harmonic (gz-sensors 8.x) dropped the CPU `lidar` type. Using `type="lidar"` will register the sensor in the scene but it will never publish data.

> `update_rate` is set to 2 Hz due to ARM64 CPU performance limits.

Verify the sensor is publishing after launch:

```bash
gz topic -e -t /lidar/points/points --duration 5   # should stream binary data
ros2 topic hz /lidar/points                         # should show ~2 Hz
```

---

## 7. ros_gz Bridge Config

### Single drone — `~/ros2_ws/bridge.yaml`

```yaml
- ros_topic_name: "/lidar/points"
  gz_topic_name: "/lidar/points/points"
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  gz_type_name: "gz.msgs.PointCloudPacked"
  direction: GZ_TO_ROS

- ros_topic_name: "/imu/data"
  gz_topic_name: "/world/iris_warehouse/model/iris_with_gimbal/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS

- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS
```

> **CRITICAL:** The `/clock` bridge is required when using `use_sim_time: true`. Without it, ROS2 nodes use wall-clock time while messages carry Gazebo sim time, causing timestamp mismatches that crash LIO-SAM's GTSAM solver.

> If using `iris_runway` world, change `iris_warehouse` to `iris_runway` in the IMU topic path.

### Two-drone bridge configs

`~/ros2_ws/bridge.yaml` (drone 1 — also bridges `/clock`):

```yaml
- ros_topic_name: "/drone1/lidar/points"
  gz_topic_name: "/drone1/lidar/points/points"
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  gz_type_name: "gz.msgs.PointCloudPacked"
  direction: GZ_TO_ROS

- ros_topic_name: "/drone1/imu/data"
  gz_topic_name: "/world/iris_warehouse/model/iris_with_gimbal/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS

- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS
```

`~/ros2_ws/bridge2.yaml` (drone 2 — no `/clock`, already bridged by drone 1):

```yaml
- ros_topic_name: "/drone2/lidar/points"
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

## 8. LIO-SAM → MAVROS Bridge Node

Save as `~/ros2_ws/src/lio_mavros_bridge.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class LioMavrosBridge(Node):
    def __init__(self):
        super().__init__('lio_mavros_bridge')
        self.declare_parameter('drone_ns', 'drone1')
        drone_ns = self.get_parameter('drone_ns').get_parameter_value().string_value
        self.last_stamp_ns = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        odom_topic = f'/{drone_ns}/lio_sam/mapping/odometry_incremental'
        pose_topic = f'/{drone_ns}/mavros/vision_pose/pose'

        self.sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, qos)
        self.pub = self.create_publisher(
            PoseStamped, pose_topic, 10)

        self.get_logger().info(f'Bridge started: {odom_topic} -> {pose_topic}')

    def odom_callback(self, msg):
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if stamp_ns <= self.last_stamp_ns:
            return  # skip backwards or duplicate timestamps
        self.last_stamp_ns = stamp_ns

        pose_msg = PoseStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = 'map'
        pose_msg.pose = msg.pose.pose
        self.pub.publish(pose_msg)

def main():
    rclpy.init()
    node = LioMavrosBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

> **MAVROS namespace mismatch warning:** MAVROS drone 1 may end up subscribing on `/mavros/vision_pose/pose` (without the `drone1/` prefix) even when launched with `ros_namespace:=drone1`. If `ros2 topic info /drone1/mavros/vision_pose/pose` shows `Subscription count: 0`, the bridge is publishing to the wrong topic. Fix with a remap:
>
> ```bash
> python3 ~/ros2_ws/src/lio_mavros_bridge.py --ros-args \
>   -p drone_ns:=drone1 \
>   -r /drone1/mavros/vision_pose/pose:=/mavros/vision_pose/pose
> ```
>
> Check where MAVROS is actually listening with: `ros2 topic list | grep vision_pose`

---

## 9. Two-Drone World Setup

Before launching two drones, you need a second iris model spawned at a different position in the warehouse. The easiest approach is to add a second model include directly in the world SDF.

Edit `~/sim/ardupilot_gazebo/worlds/iris_warehouse.sdf` and add a second drone inside the `<world>` tag (after the first drone's `<include>` block):

```xml
<!-- Drone 2 — offset 3m on Y axis so they don't overlap -->
<include>
  <uri>model://iris_with_gimbal</uri>
  <name>iris_with_gimbal_2</name>
  <pose>0 3 0.2 0 0 0</pose>
</include>
```

> Each drone needs a unique `<name>` — this is what Gazebo uses to namespace its sensor topics. Drone 2's LiDAR will publish to `/drone2/lidar/points/points` and its IMU to the equivalent path. Verify after launch with `gz topic -l | grep iris_with_gimbal_2`.

---

## 10. Full Startup Order — Single Drone

Run each in a separate terminal in this exact order:

```bash
# Terminal 1 — Gazebo
cd ~/sim/ardupilot_gazebo
gz sim worlds/iris_warehouse.sdf -r

# Terminal 2 — SITL drone 1 (wait for Gazebo to fully load first)
cd ~/sim/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console -I0

# Terminal 3 — ros_gz bridge (drone 1)
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$HOME/ros2_ws/bridge.yaml

# Terminal 4 — MAVROS (drone 1, port 14550)
source /opt/ros/humble/setup.bash
ros2 launch mavros apm.launch fcu_url:=udp://:14550@localhost

# Terminal 5 — LIO-SAM (drone 1)
source ~/ros2_ws/install/setup.bash
ros2 launch lio_sam run.launch.py \
  params_file:=$HOME/ros2_ws/src/LIO-SAM/config/params_drone1.yaml \
  namespace:=drone1

# Terminal 6 — LIO-SAM → MAVROS bridge (drone 1)
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/lio_mavros_bridge.py
```

---

## 11. Full Startup Order — Two Drones

Requires the two-drone world SDF setup from Section 9. Each drone needs its own SITL instance, bridge, MAVROS, static TF publishers, LIO-SAM, and bridge node — all namespaced separately.

### Namespace Architecture

| Component | Drone 1 | Drone 2 |
|---|---|---|
| ROS2 namespace | `/drone1/` | `/drone2/` |
| LiDAR topic | `/drone1/lidar/points` | `/drone2/lidar/points` |
| IMU topic | `/drone1/imu/data` | `/drone2/imu/data` |
| Odometry | `/drone1/lio_sam/mapping/odometry_incremental` | `/drone2/lio_sam/mapping/odometry_incremental` |
| Vision pose | `/mavros/vision_pose/pose` (see note) | `/drone2/mavros/vision_pose/pose` |
| SITL instance | `-I0` (ports 14550/14551) | `-I1` (ports 14560/14561) |
| MAVROS FCU URL | `udp://:14551@127.0.0.1:14555` | `udp://:14561@127.0.0.1:14565` |
| TF frames | `drone1/base_link`, `drone1/lidar_link`, `drone1/odom`, `drone1/map` | `drone2/base_link`, `drone2/lidar_link`, `drone2/odom`, `drone2/map` |

> **MAVROS namespace note:** MAVROS drone 1 may not respect `ros_namespace:=drone1` and subscribe on `/mavros/vision_pose/pose` instead of `/drone1/mavros/vision_pose/pose`. Always verify with `ros2 topic list | grep vision_pose` and remap the bridge if needed (see Section 8 note).

### Launch sequence (13 terminals)

```bash
# ── TERMINAL 1 — Gazebo simulation ──────────────────────────────────────────
cd ~/ardupilot_gazebo
gz sim -v4 -r worlds/iris_warehouse.world

# ── TERMINAL 2 — SITL Drone 1 ───────────────────────────────────────────────
cd ~/ardupilot
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
  -I0 --sysid 1 --out=udp:127.0.0.1:14550 \
  --custom-location=51.5074,-0.1278,0,0

# ── TERMINAL 3 — SITL Drone 2 ───────────────────────────────────────────────
cd ~/ardupilot
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
  -I1 --sysid 2 --out=udp:127.0.0.1:14560 \
  --custom-location=51.5074,-0.1248,0,0

# ── TERMINAL 4 — ROS2 bridge (drone1 sensors + clock) ───────────────────────
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  --params-file ~/ros2_ws/bridge.yaml

# ── TERMINAL 5 — ROS2 bridge (drone2 sensors) ───────────────────────────────
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  --params-file ~/ros2_ws/bridge2.yaml

# ── TERMINAL 6 — Static TF publishers (drone1) ──────────────────────────────
source /opt/ros/humble/setup.bash
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 \
  --frame-id base_link --child-frame-id drone1/base_link &
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0.1 --yaw 0 --pitch 0 --roll 0 \
  --frame-id drone1/base_link --child-frame-id drone1/lidar_link &
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 \
  --frame-id map --child-frame-id drone1/map &
wait

# ── TERMINAL 7 — Static TF publishers (drone2) ──────────────────────────────
source /opt/ros/humble/setup.bash
ros2 run tf2_ros static_transform_publisher \
  --x 5 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 \
  --frame-id base_link --child-frame-id drone2/base_link &
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0.1 --yaw 0 --pitch 0 --roll 0 \
  --frame-id drone2/base_link --child-frame-id drone2/lidar_link &
ros2 run tf2_ros static_transform_publisher \
  --x 5 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 \
  --frame-id map --child-frame-id drone2/map &
wait

# ── TERMINAL 8 — MAVROS Drone 1 ─────────────────────────────────────────────
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch mavros apm.launch fcu_url:="udp://:14551@127.0.0.1:14555" \
  ros_namespace:=drone1 \
  tgt_system:=1 \
  params_yaml:=$HOME/ros2_ws/mavros_drone1.yaml

# ── TERMINAL 9 — MAVROS Drone 2 ─────────────────────────────────────────────
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch mavros apm.launch fcu_url:="udp://:14561@127.0.0.1:14565" \
  ros_namespace:=drone2 \
  tgt_system:=2 \
  params_yaml:=$HOME/ros2_ws/mavros_drone2.yaml

# ── TERMINAL 10 — LIO-SAM Drone 1 ───────────────────────────────────────────
# CRITICAL: Use namespace:= argument, NOT PushRosNamespace wrapper.
# LIO-SAM's run.launch.py has a built-in namespace parameter that sets
# namespace on each Node declaration. PushRosNamespace is overridden by
# the explicit namespace= on each Node and has no effect.
source ~/ros2_ws/install/setup.bash
ros2 launch lio_sam run.launch.py \
  params_file:=$HOME/ros2_ws/src/LIO-SAM/config/params_drone1.yaml \
  namespace:=drone1

# ── TERMINAL 11 — LIO-SAM Drone 2 ───────────────────────────────────────────
source ~/ros2_ws/install/setup.bash
ros2 launch lio_sam run.launch.py \
  params_file:=$HOME/ros2_ws/src/LIO-SAM/config/params_drone2.yaml \
  namespace:=drone2

# ── TERMINAL 12 — LIO-MAVROS Bridge Drone 1 ─────────────────────────────────
# NOTE: If MAVROS drone1 subscribes on /mavros/vision_pose/pose instead of
# /drone1/mavros/vision_pose/pose, add the remap:
#   -r /drone1/mavros/vision_pose/pose:=/mavros/vision_pose/pose
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/lio_mavros_bridge.py --ros-args \
  -p drone_ns:=drone1 \
  -r /drone1/mavros/vision_pose/pose:=/mavros/vision_pose/pose

# ── TERMINAL 13 — LIO-MAVROS Bridge Drone 2 ─────────────────────────────────
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/lio_mavros_bridge.py --ros-args -p drone_ns:=drone2
```

### Verify both drones are up

```bash
# Sensors flowing
ros2 topic hz /drone1/lidar/points          # ~2Hz
ros2 topic hz /drone2/lidar/points          # ~2Hz
ros2 topic hz /drone1/imu/data              # ~100-400Hz
ros2 topic hz /drone2/imu/data              # ~100-400Hz

# LIO-SAM nodes properly namespaced (CRITICAL — both should show /drone1/ and /drone2/ prefixes)
ros2 node list | grep lio
# Expected:
#   /drone1/lio_sam_imuPreintegration
#   /drone1/lio_sam_imageProjection
#   /drone1/lio_sam_featureExtraction
#   /drone1/lio_sam_mapOptimization
#   /drone2/lio_sam_imuPreintegration
#   /drone2/lio_sam_imageProjection
#   /drone2/lio_sam_featureExtraction
#   /drone2/lio_sam_mapOptimization
# If you see /lio_sam_imuPreintegration (no namespace prefix), the namespace
# argument was not passed — both drones will collide and crash.

# MAVROS connected
ros2 topic echo /mavros/state --once        # connected: true
ros2 topic echo /drone2/mavros/state --once # connected: true

# Vision pose reaching MAVROS (Publisher AND Subscription count must both be >= 1)
ros2 topic info /mavros/vision_pose/pose
ros2 topic info /drone2/mavros/vision_pose/pose

# Odometry publishing (only after drones are airborne and moving)
ros2 topic hz /drone1/lio_sam/mapping/odometry_incremental
ros2 topic hz /drone2/lio_sam/mapping/odometry_incremental
```

---

## 12. GPS-Denied Bootstrap Procedure

Since LIO-SAM needs motion to initialize, use GPS briefly to get airborne. Run these commands in **each drone's MAVProxy console** separately. Always run `param fetch` first and wait for it to respond before setting parameters.

### Step 1 — Enable GPS and take off (both drones)

```bash
# Run in Drone 1 MAVProxy (--map --console window)
param fetch
param set GPS1_TYPE 1
param set EK3_SRC1_POSXY 3
param set EK3_SRC1_VELXY 3
param set VISO_TYPE 0
mode guided
arm throttle
takeoff 3
```

```bash
# Run in Drone 2 MAVProxy (-I1 window) — same commands
param fetch
param set GPS1_TYPE 1
param set EK3_SRC1_POSXY 3
param set EK3_SRC1_VELXY 3
param set VISO_TYPE 0
mode guided
arm throttle
takeoff 3
```

### Step 2 — Wait for LIO-SAM to initialize

In a separate terminal, watch for odometry to appear:

```bash
ros2 topic hz /drone1/lio_sam/mapping/odometry_incremental
ros2 topic hz /drone2/lio_sam/mapping/odometry_incremental
```

Both should start publishing within a few seconds of the drones moving.

### Step 3 — Switch to LIO-SAM nav (both drones)

Once odometry is publishing, run in **each** MAVProxy console:

```bash
param set EK3_SRC1_POSXY 6
param set EK3_SRC1_VELXY 6
param set EK3_SRC1_YAW 6
param set GPS1_TYPE 0
param set VISO_TYPE 1
```

### Step 4 — Verify full pipeline

```bash
ros2 topic hz /drone1/lio_sam/mapping/odometry_incremental  # publishing ~1.5-2 Hz
ros2 topic hz /drone2/lio_sam/mapping/odometry_incremental  # publishing ~1.5-2 Hz
ros2 topic hz /mavros/vision_pose/pose                      # ~1.5-2 Hz
ros2 topic hz /drone2/mavros/vision_pose/pose               # ~1.5-2 Hz
```

> Check MAVProxy console for "EKF3 IMU0 is using ExternalNav" — this confirms ArduPilot has accepted the LIO-SAM pose data. If it still says "using GPS", wait longer or verify vision_pose subscription count is > 0.

---

## 13. LIO-SAM Code Patches (Gazebo Clock Jump Tolerance)

Two SITL instances competing for the Gazebo physics timestep cause periodic ~0.8s backward clock jumps. These corrupt LIO-SAM's IMU preintegration, causing GTSAM `IndeterminantLinearSystemException` crashes. The following patches to `imuPreintegration.cpp` are **required** for multi-drone simulation.

### Patch 1: Skip backward IMU timestamps + clamp large dt

Three locations in `~/ros2_ws/src/LIO-SAM/src/imuPreintegration.cpp` where `double dt = ...` is computed. After each, add a guard:

**Location A — optimization IMU loop (~line 392):**
```cpp
double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);
if (dt <= 0.0) { lastImuT_opt = imuTime; continue; }
if (dt > 0.02) dt = 0.02;
```

**Location B — IMU queue replay loop (~line 461):**
```cpp
double dt = (lastImuQT < 0) ? (1.0 / 500.0) :(imuTime - lastImuQT);
if (dt <= 0.0) { lastImuQT = imuTime; continue; }
if (dt > 0.02) dt = 0.02;
```

**Location C — imuHandler callback (~line 507):**
```cpp
double dt = (lastImuT_imu < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu);
if (dt <= 0.0) { lastImuT_imu = imuTime; return; }
if (dt > 0.02) dt = 0.02;
```

> Locations A and B are inside `for` loops → use `continue`. Location C is a callback → use `return`.

> The `dt > 0.02` clamp prevents forward clock jumps from integrating gravity for too long. Without it, a 0.8s jump adds ~8 m/s velocity per event → exceeds the 30 m/s threshold after ~4 jumps → "Large velocity, reset IMU-preintegration!" → eventual GTSAM crash.

### Patch 2: Skip backward odometry corrections

Add a member variable and guard to `odometryHandler()`:

**Add member variable** (near `lastImuT_imu` declaration, ~line 211):
```cpp
double lastCorrectionTime = -1;
```

**Add guard** in `odometryHandler()`, right after `currentCorrectionTime` is computed:
```cpp
void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg)
{
    std::lock_guard<std::mutex> lock(mtx);
    double currentCorrectionTime = stamp2Sec(odomMsg->header.stamp);
    if (currentCorrectionTime <= lastCorrectionTime) return;  // ← ADD THIS
    lastCorrectionTime = currentCorrectionTime;                // ← ADD THIS
    // make sure we have imu data to integrate
    if (imuQueOpt.empty())
        return;
    // ... rest of function unchanged
```

### Rebuild after patching

```bash
cd ~/ros2_ws && colcon build --packages-select lio_sam
```

> Only deprecation warnings from GTSAM headers are expected — any actual errors mean a patch was misplaced. Keep a backup: `cp imuPreintegration.cpp imuPreintegration.cpp.bak` before patching.

---

## 14. Autonomous Navigation (GUIDED Mode Waypoints)

Once the drone is airborne and LIO-SAM is active, you can command autonomous movement via MAVROS position setpoints. This is the foundation for the swarm attack experiments.

### Quick test — send a single position command

```bash
# Confirm MAVROS is receiving pose estimates before sending commands
ros2 topic echo /mavros/local_position/pose --once

# Send a single position setpoint (x=5m, y=0, z=3m in local frame)
ros2 topic pub --once /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0, z: 3.0}, orientation: {w: 1.0}}}"
```

> The drone will only move if it is already in GUIDED mode and armed. Run `mode guided` in MAVProxy first.

### Waypoint patrol script

Save as `~/ros2_ws/src/waypoint_patrol.py` and run after the bootstrap procedure:

```python
#!/usr/bin/env python3
"""
Simple waypoint patrol for GPS-denied warehouse navigation.
Requires: drone airborne, LIO-SAM active, MAVROS connected.
Run: python3 waypoint_patrol.py
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
import time

# Waypoints as (x, y, z) in metres — local ENU frame
# Adjust to match your warehouse layout
WAYPOINTS = [
    ( 5.0,  0.0, 3.0),
    ( 5.0,  5.0, 3.0),
    ( 0.0,  5.0, 3.0),
    ( 0.0,  0.0, 3.0),
]

HOLD_TIME = 5.0      # seconds to hold at each waypoint
TOLERANCE = 0.5      # metres — how close counts as "reached"

class WaypointPatrol(Node):
    def __init__(self):
        super().__init__('waypoint_patrol')
        self.state = State()
        self.current_pose = PoseStamped()

        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_cb, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_cb, 10)
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)

        self.set_mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_cli  = self.create_client(CommandBool, '/mavros/cmd/arming')

        self.get_logger().info('Waypoint patrol node started')

    def state_cb(self, msg):
        self.state = msg

    def pose_cb(self, msg):
        self.current_pose = msg

    def distance_to(self, x, y, z):
        p = self.current_pose.pose.position
        return ((p.x - x)**2 + (p.y - y)**2 + (p.z - z)**2) ** 0.5

    def send_setpoint(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.setpoint_pub.publish(msg)

    def run_patrol(self):
        self.get_logger().info('Starting patrol...')
        for i, (x, y, z) in enumerate(WAYPOINTS):
            self.get_logger().info(f'Heading to waypoint {i+1}: ({x}, {y}, {z})')
            while self.distance_to(x, y, z) > TOLERANCE:
                self.send_setpoint(x, y, z)
                time.sleep(0.1)
                rclpy.spin_once(self, timeout_sec=0)
            self.get_logger().info(f'Reached waypoint {i+1} — holding {HOLD_TIME}s')
            hold_end = time.time() + HOLD_TIME
            while time.time() < hold_end:
                self.send_setpoint(x, y, z)
                time.sleep(0.1)
                rclpy.spin_once(self, timeout_sec=0)
        self.get_logger().info('Patrol complete')

def main():
    rclpy.init()
    node = WaypointPatrol()
    # Brief wait for connections
    time.sleep(2.0)
    node.run_patrol()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Run it:

```bash
python3 ~/ros2_ws/src/waypoint_patrol.py
```

Monitor progress:

```bash
# Watch drone position update in real time
ros2 topic echo /mavros/local_position/pose

# Confirm LIO-SAM is still tracking during movement
ros2 topic hz /drone1/lio_sam/mapping/odometry_incremental
```

> **If the drone doesn't move:** confirm it is in GUIDED mode (`mode guided` in MAVProxy) and that `/mavros/local_position/pose` is publishing. The drone must be receiving a continuous stream of setpoints — a single publish is not sufficient for GUIDED mode.

---

## Architecture Overview

```
Gazebo Sim (iris_warehouse)
    │
    ├── VLP-16 LiDAR → /drone1/lidar/points/points (gz topic)
    ├── IMU sensor   → /world/.../imu (gz topic)
    └── /clock       → /clock (gz topic)
           │
      ros_gz_bridge (+ /clock bridge for use_sim_time)
           │
    ├── /drone1/lidar/points  (sensor_msgs/PointCloud2)
    ├── /drone1/imu/data      (sensor_msgs/Imu)
    └── /clock                (rosgraph_msgs/Clock)
           │
        LIO-SAM (namespace:=drone1)
           │
    /drone1/lio_sam/mapping/odometry_incremental
           │
    lio_mavros_bridge.py (with remap if needed)
           │
    /mavros/vision_pose/pose
           │
        MAVROS (EKF3 ExternalNav)
           │
    ArduCopter SITL (EK3_SRC1_POSXY=6)
           │
    /mavros/setpoint_position/local  ← waypoint_patrol.py
```

---

## Known Issues & Limitations

### Gazebo clock jumps (two-SITL limitation)

Two ArduCopter SITL instances each try to step the Gazebo simulation, causing periodic ~0.8s backward clock jumps on `/clock`. This is a fundamental limitation of the ArduPilot Gazebo plugin architecture. Mitigations applied: dt guards + clamping in LIO-SAM (Section 13), backward-timestamp rejection in bridge node and odometry handler. Document in thesis methodology as a known simulation artefact.

### "Point cloud timestamp not available, deskew function disabled"

Gazebo's `PointCloudPacked` messages don't include per-point timestamps. LIO-SAM's deskewing (motion compensation within a single scan) is disabled. This causes additional drift but is acceptable for the attack demonstration use case.

### "Large velocity, reset IMU-preintegration"

Periodic resets (~every 30s) caused by residual clock jitter exceeding the 30 m/s velocity threshold. LIO-SAM self-recovers within ~1 second when map optimization provides the next correction. Normal on the ground (no features) — should reduce significantly once airborne and moving.

### "Not enough features" in warehouse

Indoor environments have many planar surfaces but few geometric edges. Use lowered thresholds from Section 5. If the drone is hovering stationary, LIO-SAM has no new geometric information and will drift on IMU alone — always keep the drone moving.

---

## Troubleshooting

| Issue | Fix |
|---|---|
| `gtsam::IndeterminantLinearSystemException` crash | Apply clock jump patches from Section 13 and rebuild |
| `PreArm: VisOdom: not healthy` | LIO-SAM not publishing — use GPS bootstrap procedure (Section 12) |
| `AHRS: waiting for home` | GPS not locked — restart SITL after setting `GPS1_TYPE 1` |
| `EKF3 IMU stopped aiding` | Re-enable compass: `COMPASS_ENABLE 1`, `EK3_SRC1_YAW 1` |
| `param set` Unknown setting | Run `param fetch` first to refresh cache |
| LiDAR not in ROS2 | Check bridge is running after Gazebo loads; verify gz topic is `/lidar/points/points` |
| LIO-SAM no odometry on flat ground | Switch to warehouse world — runway is too featureless for SLAM |
| `ros-humble-ros-gzharmonic` not found | Build ros_gz from source with `GZ_VERSION=harmonic` |
| LiDAR sensor registered but zero messages | Ensure `type="gpu_lidar"` in model SDF — `type="lidar"` is not supported in Gazebo Harmonic |
| LiDAR link renamed to `lidar_link(1)` | LiDAR block is in wrong model file — must be in `iris_with_standoffs`, not `iris_with_gimbal` |
| Drone ignores setpoint commands | Must be in GUIDED mode (`mode guided` in MAVProxy) and continuously publishing setpoints |
| `/mavros/local_position/pose` not publishing | MAVROS not receiving vision pose — check lio_mavros_bridge.py is running |
| LIO-SAM odometry z plummets to -300+ | Wrong IMU extrinsics — see Section 5 for correct Gazebo values |
| LIO-SAM nodes collide (duplicate names, crash on drone2 launch) | Use `namespace:=droneN` argument to `run.launch.py`, NOT `PushRosNamespace` wrapper |
| Vision pose `Subscription count: 0` | MAVROS namespace mismatch — remap bridge output (see Section 8 note) |
| `use_sim_time` not taking effect | Verify single `/**:` block in YAML — duplicate blocks cause silent override |
| Duplicate `ros__parameters:` under one `/**:` | YAML takes last key — merge into single `ros__parameters:` block |

---

## Correct params_drone1.yaml (complete)

```yaml
/**:
  ros__parameters:
    use_sim_time: true
    # Topics
    pointCloudTopic: "/drone1/lidar/points"
    imuTopic: "/drone1/imu/data"
    odomTopic: "odometry/imu"
    gpsTopic: "odometry/gpsz"
    # Frames
    lidarFrame: "drone1/lidar_link"
    baselinkFrame: "drone1/base_link"
    odometryFrame: "drone1/odom"
    mapFrame: "drone1/map"
    imuFrame: "drone1/imu_link"
    imuType: 0
    imuRate: 100.0
    # GPS Settings
    useImuHeadingInitialization: false
    useGpsElevation: false
    gpsCovThreshold: 2.0
    poseCovThreshold: 25.0
    # Export settings
    savePCD: false
    savePCDDirectory: "/Downloads/LOAM/"
    # Sensor Settings
    sensor: velodyne
    N_SCAN: 16
    Horizon_SCAN: 360
    downsampleRate: 1
    lidarMinRange: 0.1
    lidarMaxRange: 100.0
    # IMU Settings — noise increased 10x for Gazebo clock jitter tolerance
    imuAccNoise: 0.1
    imuGyrNoise: 0.01
    imuAccBiasN: 0.0064
    imuGyrBiasN: 0.00356
    imuGravity: 9.80511
    imuRPYWeight: 0.01
    # Extrinsics — NED→ENU for accel/gyro, identity for orientation
    extrinsicTrans: [0.0, 0.0, 0.1]
    extrinsicRot: [1.0,  0.0,  0.0,
                   0.0, -1.0,  0.0,
                   0.0,  0.0, -1.0]
    extrinsicRPY: [1.0, 0.0, 0.0,
                   0.0, 1.0, 0.0,
                   0.0, 0.0, 1.0]
    # LOAM feature threshold — lowered for indoor warehouse
    edgeThreshold: 0.5
    surfThreshold: 0.1
    edgeFeatureMinValidNum: 2
    surfFeatureMinValidNum: 50
    # voxel filter params
    odometrySurfLeafSize: 0.4
    mappingCornerLeafSize: 0.2
    mappingSurfLeafSize: 0.4
    # robot motion constraint
    z_tollerance: 1000.0
    rotation_tollerance: 1000.0
    # CPU Params
    numberOfCores: 4
    mappingProcessInterval: 0.15
    # Surrounding map
    surroundingkeyframeAddingDistThreshold: 1.0
    surroundingkeyframeAddingAngleThreshold: 0.2
    surroundingKeyframeDensity: 2.0
    surroundingKeyframeSearchRadius: 50.0
    # Loop closure
    loopClosureEnableFlag: true
    loopClosureFrequency: 1.0
    surroundingKeyframeSize: 50
    historyKeyframeSearchRadius: 15.0
    historyKeyframeSearchTimeDiff: 30.0
    historyKeyframeSearchNum: 25
    historyKeyframeFitnessScore: 0.3
    # Visualization
    globalMapVisualizationSearchRadius: 1000.0
    globalMapVisualizationPoseDensity: 10.0
    globalMapVisualizationLeafSize: 1.0
```

> **For params_drone2.yaml:** Copy the above and replace all `drone1` with `drone2`. Ensure there is only ONE `/**:` block and ONE `ros__parameters:` key. No duplicate keys.
