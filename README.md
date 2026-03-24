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

---

## 6. VLP-16 LiDAR Sensor (Gazebo Model)

Add to `~/sim/ardupilot_gazebo/models/iris_with_gimbal/model.sdf` before `</model>`:

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
  <sensor name="lidar" type="lidar">
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
  <parent>iris_with_standoffs::base_link</parent>
  <child>lidar_link</child>
</joint>
```

> Using `type="lidar"` (CPU-based) instead of `gpu_lidar` for ARM64 compatibility.
> `update_rate` set to 2 Hz due to ARM64 CPU performance limits.

---

## 7. ros_gz Bridge Config

Create `~/ros2_ws/bridge.yaml`:

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
```

> If using `iris_runway` world, change `iris_warehouse` to `iris_runway` in the IMU topic path.

---

## 8. LIO-SAM → MAVROS Bridge Node

Save as `~/ros2_ws/src/lio_mavros_bridge.py`:

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

---

## 9. Full Startup Order

Run each in a separate terminal in this exact order:

```bash
# 1. Gazebo
cd ~/sim/ardupilot_gazebo
gz sim worlds/iris_warehouse.sdf -r

# 2. SITL (wait for Gazebo to fully load first)
cd ~/sim/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console

# 3. ros_gz bridge
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$HOME/ros2_ws/bridge.yaml

# 4. MAVROS
source /opt/ros/humble/setup.bash
ros2 launch mavros apm.launch fcu_url:=udp://:14550@localhost

# 5. LIO-SAM
source ~/ros2_ws/install/setup.bash
ros2 launch lio_sam run.launch.py

# 6. LIO-SAM → MAVROS bridge node
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/lio_mavros_bridge.py
```

---

## 10. GPS-Denied Bootstrap Procedure

Since LIO-SAM needs motion to initialize, use GPS briefly to get airborne:

```bash
# In MAVProxy — temporarily enable GPS
param set GPS1_TYPE 1
param set EK3_SRC1_POSXY 3
param set EK3_SRC1_VELXY 3
param set VISO_TYPE 0

# Arm and take off
mode guided
arm throttle
takeoff 3

# Once LIO-SAM starts publishing odometry — switch to LiDAR nav
param set EK3_SRC1_POSXY 6
param set EK3_SRC1_VELXY 6
param set GPS1_TYPE 0
param set VISO_TYPE 1
```

Verify LIO-SAM is publishing:
```bash
ros2 topic hz /lio_sam/mapping/odometry   # should show data
ros2 topic hz /mavros/vision_pose/pose    # should show ~2Hz
```

---

## Architecture Overview

```
Gazebo Sim (iris_warehouse)
    │
    ├── VLP-16 LiDAR → /lidar/points/points (gz topic)
    └── IMU sensor   → /world/.../imu (gz topic)
           │
      ros_gz_bridge
           │
    ├── /lidar/points  (sensor_msgs/PointCloud2)
    └── /imu/data      (sensor_msgs/Imu)
           │
        LIO-SAM
           │
    /lio_sam/mapping/odometry
           │
    lio_mavros_bridge.py
           │
    /mavros/vision_pose/pose
           │
        MAVROS
           │
    ArduCopter SITL (EKF3 External Nav)
```

---

## Troubleshooting

| Issue | Fix |
|---|---|
| `PreArm: VisOdom: not healthy` | LIO-SAM not publishing — use GPS bootstrap procedure |
| `AHRS: waiting for home` | GPS not locked — restart SITL after setting `GPS1_TYPE 1` |
| `EKF3 IMU stopped aiding` | Re-enable compass: `COMPASS_ENABLE 1`, `EK3_SRC1_YAW 1` |
| `param set` Unknown setting | Run `param fetch` first to refresh cache |
| LiDAR not in ROS2 | Check bridge is running after Gazebo loads; use `/lidar/points/points` as gz topic |
| LIO-SAM no odometry on flat ground | Switch to warehouse world — runway is too featureless for SLAM |
| `ros-humble-ros-gzharmonic` not found | Build ros_gz from source with `GZ_VERSION=harmonic` |
