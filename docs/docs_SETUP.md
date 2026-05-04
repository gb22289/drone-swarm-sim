# Setup — Full Installation Procedure

This document covers the complete installation procedure for the simulation stack. Once this is done, see [LAUNCH.md](LAUNCH.md) for launch sequences and [ATTACKS.md](ATTACKS.md) for running individual attacks.

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

# Apply patches from this submission
patch -p1 -d ~/ros2_ws/src/LIO-SAM \
  < /path/to/this/repo/lio_sam_patches/imu_preintegration_dt_guards.patch
patch -p1 -d ~/ros2_ws/src/LIO-SAM \
  < /path/to/this/repo/lio_sam_patches/mapoptmization_kdtree_check.patch

cd ~/ros2_ws
colcon build --packages-select lio_sam --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

### LIO-SAM Configuration

Use `lio_sam_config/params_drone1.yaml` and `params_drone2.yaml` from this submission. Key values:

```yaml
pointCloudTopic: "/drone1/lidar/points_timed"   # NOTE: timed, post-shim
imuTopic: "/drone1/imu/data"
sensor: velodyne
N_SCAN: 16
Horizon_SCAN: 360                                # matches the SDF below
lidarMinRange: 0.1
lidarMaxRange: 100.0
imuType: 0
imuRate: 100.0

lidarFrame: "lidar_link"
baselinkFrame: "base_link"
odometryFrame: "odom"
mapFrame: "map"

# Extrinsics for Gazebo IMU (NED → ENU on accel/gyro, identity on orientation)
extrinsicTrans: [0.0, 0.0, 0.0]
extrinsicRot: [1.0,  0.0,  0.0,
               0.0, -1.0,  0.0,
               0.0,  0.0, -1.0]
extrinsicRPY: [1.0, 0.0, 0.0,
               0.0, 1.0, 0.0,
               0.0, 0.0, 1.0]

# Lower thresholds for indoor warehouse
edgeThreshold: 0.5
edgeFeatureMinValidNum: 2
surfFeatureMinValidNum: 50
```

Place under `~/ros2_ws/install/lio_sam/share/lio_sam/config/` or rebuild after editing source-tree configs.

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
          <samples>360</samples>
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
>
> **`<samples>360</samples>`** balances RTF performance and SLAM feature density on consumer hardware. Higher values (1800 = real VLP-16) are more realistic but cost RTF; lower values (90, 180) cost angular resolution. Whatever you set here must match `Horizon_SCAN` in `params_droneN.yaml`.
>
> `update_rate` is set to 2 Hz for ARM64 / WSL2 RTF reasons.

Verify the sensor is publishing after launch:

```bash
gz topic -e -t /lidar/points/points --duration 5   # should stream binary data
ros2 topic hz /drone1/lidar/points                 # ~1 Hz at 47% RTF
```

---

## 7. ros_gz Bridge Config

Create `~/ros2_ws/bridge.yaml`:

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

> **Critical — `/clock` mapping.** Both `ros_topic_name` and `gz_topic_name`
> must be exactly `/clock`. Mapping to a different ROS name (e.g.
> `/clock_raw`) will cause nodes running with `use_sim_time:=true`
> to subscribe to a topic with zero publishers and hang waiting for sim time.
> Symptoms: `Large velocity, reset IMU-preintegration!` warnings, MAVROS
> `Time jump detected`, EKF3 lane switches, LIO-SAM diverging during hover.
> Verify with:
> ```
> ros2 topic info /clock --verbose | grep "Publisher count"
> # Must be: Publisher count: 1
> ros2 topic hz /clock --window 100   # hundreds of Hz
> ```

---

## 8. LiDAR Deskew Shim

Gazebo's `gpu_lidar` sensor publishes `PointCloud2` messages without a per-point `time` field. LIO-SAM requires this field for motion deskewing. Without it, LIO-SAM logs `Point cloud timestamp not available, deskew function disabled, system will drift significantly!` and the pose drifts during hover.

The shim sits between the bridge and LIO-SAM:

```
Gazebo → ros_gz_bridge → /drone1/lidar/points   (raw)
                       → [deskew shim adds 'time']
                       → /drone1/lidar/points_timed
                       → LIO-SAM
```

The shim is part of the `swarm_mission` package. After building:

```bash
ros2 run swarm_mission lidar_deskew_shim --ros-args \
  -p input_topic:=/drone1/lidar/points \
  -p output_topic:=/drone1/lidar/points_timed \
  -p scan_period:=0.1
```

For drone 2, swap `drone1` for `drone2` in both topic names. The shim must be running before LIO-SAM starts.

---

## 9. LIO-SAM → MAVROS Bridge Node

Save as `~/ros2_ws/src/lio_mavros_bridge.py` (already in this submission):

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class LioMavrosBridge(Node):
    def __init__(self):
        super().__init__('lio_mavros_bridge')
        self.sub = self.create_subscription(
            Odometry,
            '/lio_sam/mapping/odometry',
            self.odom_callback,
            qos_profile_sensor_data)
        self.pub = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            10)
        self.get_logger().info('LIO-SAM → MAVROS bridge started')

    def odom_callback(self, msg):
        pose_msg = PoseStamped()
        # Rewrite stamp to wall-clock — EKF3 rejects sim-time stamps.
        pose_msg.header.stamp = self.get_clock().now().to_msg()
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

> **The timestamp rewrite is load-bearing.** LIO-SAM publishes with sim-time stamps; MAVROS and EKF3 run in wall-time and reject vision_pose with deviating stamps. Without the rewrite, EKF3 never accepts vision updates.

---

## 10. Two-Drone World Setup

Add a second drone to `~/sim/ardupilot_gazebo/worlds/iris_warehouse.sdf`:

```xml
<!-- Drone 2 — offset 3m on Y axis so they don't overlap -->
<include>
  <uri>model://iris_with_gimbal</uri>
  <name>iris_with_gimbal_2</name>
  <pose>0 3 0.2 0 0 0</pose>
</include>
```

> Each drone needs a unique `<name>` — Gazebo uses it to namespace sensor topics. Drone 2's LiDAR publishes to `/drone2/lidar/points/points` and IMU to the equivalent path. Verify with `gz topic -l | grep iris_with_gimbal_2` after launch.

Create `~/ros2_ws/bridge2.yaml`. **Do not include `/clock` in this file** — the drone-1 bridge handles it; two `/clock` publishers cause DDS race conditions.

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

Verify the IMU topic path with `gz topic -l | grep imu | grep gimbal_2`.

Create per-drone MAVROS config files:

```bash
cat > ~/ros2_ws/mavros_drone1.yaml << 'EOF'
/**:
  ros__parameters:
    tf_prefix: "drone1"
    use_sim_time: true
EOF

cat > ~/ros2_ws/mavros_drone2.yaml << 'EOF'
/**:
  ros__parameters:
    tf_prefix: "drone2"
    use_sim_time: true
EOF
```

---

Once all sections of this document are complete, proceed to [LAUNCH.md](LAUNCH.md) for the launch sequence.
