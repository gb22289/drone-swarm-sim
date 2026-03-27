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

Also create a second bridge config at `~/ros2_ws/bridge2.yaml`:

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

> Check the exact Gazebo IMU topic path for drone 2 with: `gz topic -l | grep imu | grep gimbal_2`

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
ros2 launch lio_sam run.launch.py

# Terminal 6 — LIO-SAM → MAVROS bridge (drone 1)
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/lio_mavros_bridge.py
```

---

## 11. Full Startup Order — Two Drones

Requires the two-drone world SDF setup from Section 9. Each drone needs its own SITL instance, bridge, MAVROS, LIO-SAM, and bridge node — all namespaced separately.

```bash
# Terminal 1 — Gazebo (loads both drone models)
cd ~/sim/ardupilot_gazebo
gz sim worlds/iris_warehouse.sdf -r

# Terminal 2 — SITL drone 1 (instance 0, ports 14550/14551)
cd ~/sim/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console -I0

# Terminal 3 — SITL drone 2 (instance 1, ports 14560/14561)
cd ~/sim/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console -I1

# Terminal 4 — ros_gz bridge drone 1
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$HOME/ros2_ws/bridge.yaml

# Terminal 5 — ros_gz bridge drone 2
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$HOME/ros2_ws/bridge2.yaml

# Terminal 6 — MAVROS drone 1 (port 14550)
source /opt/ros/humble/setup.bash
ros2 launch mavros apm.launch fcu_url:=udp://:14550@localhost

# Terminal 7 — MAVROS drone 2 (port 14560)
source /opt/ros/humble/setup.bash
ros2 launch mavros apm.launch \
  fcu_url:=udp://:14560@localhost \
  tgt_system:=2 \
  __ns:=/drone2

# Terminal 8 — LIO-SAM drone 1
source ~/ros2_ws/install/setup.bash
ros2 launch lio_sam run.launch.py

# Terminal 9 — LIO-SAM drone 2 (remapped to drone2 topics)
source ~/ros2_ws/install/setup.bash
ros2 launch lio_sam run.launch.py \
  --ros-args \
  -r /lidar/points:=/drone2/lidar/points \
  -r /imu/data:=/drone2/imu/data \
  -r /lio_sam/mapping/odometry:=/drone2/lio_sam/mapping/odometry \
  --params-file ~/ros2_ws/src/LIO-SAM/config/params.yaml

# Terminal 10 — LIO-SAM → MAVROS bridge drone 1
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/lio_mavros_bridge.py

# Terminal 11 — LIO-SAM → MAVROS bridge drone 2
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/lio_mavros_bridge.py \
  --ros-args \
  -r /lio_sam/mapping/odometry:=/drone2/lio_sam/mapping/odometry \
  -r /mavros/vision_pose/pose:=/drone2/mavros/vision_pose/pose
```

Verify both drones are connected:
```bash
# Drone 1
ros2 topic hz /mavros/state
ros2 topic hz /lio_sam/mapping/odometry

# Drone 2
ros2 topic hz /drone2/mavros/state 2>/dev/null || ros2 topic echo /drone2/mavros/state --once
ros2 topic hz /drone2/lio_sam/mapping/odometry
```

> **MAVProxy note:** Each SITL instance opens its own MAVProxy console. Drone 1 is on the window that launched with `--map --console`. Drone 2's console is the plain window from `-I1`. Run bootstrap and parameter commands in the correct window for each drone.

---

## 12. GPS-Denied Bootstrap Procedure

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

## 13. Autonomous Navigation (GUIDED Mode Waypoints)

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
ros2 topic hz /lio_sam/mapping/odometry
```

> **If the drone doesn't move:** confirm it is in GUIDED mode (`mode guided` in MAVProxy) and that `/mavros/local_position/pose` is publishing. The drone must be receiving a continuous stream of setpoints — a single publish is not sufficient for GUIDED mode.

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
           │
    /mavros/setpoint_position/local  ← waypoint_patrol.py sends targets here
```

---

## Troubleshooting

| Issue | Fix |
|---|---|
| `PreArm: VisOdom: not healthy` | LIO-SAM not publishing — use GPS bootstrap procedure |
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
