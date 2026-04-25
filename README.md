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

### LIO-SAM Configuration (`~/ros2_ws/src/LIO-SAM/config/params_droneN.yaml`)

Key values to set. Note the LiDAR topic is `points_timed` — this is the
output of the deskew shim (Section 7b), not the raw bridge output.

```yaml
pointCloudTopic: "/drone1/lidar/points_timed"
imuTopic: "/drone1/imu/data"
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
ros2 topic hz /drone1/lidar/points                  # should show ~1 Hz (after bridge)
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
> (LIO-SAM, static_transform_publisher, tf2 listener, etc.) to subscribe to
> a topic with zero publishers and hang waiting for sim time. Symptoms when
> broken: `Large velocity, reset IMU-preintegration!` warnings, MAVROS
> `Time jump detected`, EKF3 lane switches, LIO-SAM diverging to huge
> position magnitudes during hover. Verify with:
> ```
> ros2 topic info /clock --verbose | grep "Publisher count"
> # Must be: Publisher count: 1
> ros2 topic hz /clock --window 100
> # Should be hundreds of Hz
> ```

---

## 7b. LiDAR Deskew Shim

Gazebo's `gpu_lidar` sensor publishes `PointCloud2` messages without a
per-point `time` field. LIO-SAM requires this field to perform motion
deskewing — correcting each point's position for drone motion during the
scan sweep. Without it, LIO-SAM prints the warning
`Point cloud timestamp not available, deskew function disabled, system
will drift significantly!` at startup and the pose estimate drifts during
hover, eventually triggering `Large velocity` resets and SLAM divergence
within ~90 seconds.

The deskew shim sits between the bridge and LIO-SAM:

```
Gazebo → ros_gz_bridge → /drone1/lidar/points   (raw, no time field)
                       → [deskew shim adds 'time'] 
                       → /drone1/lidar/points_timed
                       → LIO-SAM
```

The shim handles mixed-datatype point clouds (the Gazebo output includes
`x,y,z,intensity` as float32 and `ring` as uint16) via a numpy structured
dtype, and synthesises a per-point `time` field in `[0, scan_period)`
based on azimuth angle, mimicking the firing order of a real rotating
LiDAR. It also guards against NaN/Inf coordinates (present in some
out-of-range rays and in manipulated scans from attack nodes), which if
left unhandled propagate into the time field, disable deskew on that
scan, and can segfault PCL's KDTree in LIO-SAM's mapOptimization.

```bash
# Build (after copying lidar_deskew_shim.py into swarm_mission package
# and registering it as a console_script in setup.py)
cd ~/ros2_ws && colcon build --packages-select swarm_mission && source install/setup.bash

# Run — one per drone, in a separate terminal, BEFORE LIO-SAM starts
ros2 run swarm_mission lidar_deskew_shim --ros-args \
  -p input_topic:=/drone1/lidar/points \
  -p output_topic:=/drone1/lidar/points_timed \
  -p scan_period:=0.1

# Drone 2 equivalent
ros2 run swarm_mission lidar_deskew_shim --ros-args \
  -p input_topic:=/drone2/lidar/points \
  -p output_topic:=/drone2/lidar/points_timed \
  -p scan_period:=0.1
```

Verify the shim is working:

```bash
# Raw bridge output (no 'time' field)
ros2 topic echo /drone1/lidar/points --field fields --once
# Fields: x, y, z, intensity, ring

# After shim (should include 'time' field)
ros2 topic echo /drone1/lidar/points_timed --field fields --once
# Fields: x, y, z, intensity, ring, time

# Rates should match
ros2 topic hz /drone1/lidar/points         # ~1 Hz (wall-clock, at ~47% RTF)
ros2 topic hz /drone1/lidar/points_timed   # ~1 Hz
```

On startup the shim logs its parsed schema once, plus frame progress:

```
Deskew shim active: /drone1/lidar/points -> /drone1/lidar/points_timed (scan_period=0.1s)
Parsed schema: point_step=32, fields=[x, y, z, intensity, ring], first_point_raw=(...)
frame 1: 5760 pts, time range [0.0000, 0.0997]s
```

> **Why synthetic timing rather than real per-point times?** Gazebo's
> `gpu_lidar` fires all rays at the same simulation-step instant, so
> there are no "real" per-point timestamps to forward. The shim models
> the firing order of a rotating LiDAR (e.g., Velodyne VLP-16), which is
> the convention LIO-SAM was designed for. Document this as a
> methodology choice: *"LiDAR point timestamps are synthesised from
> azimuth angle to model rotating-scanner firing order, as Gazebo's
> `gpu_lidar` sensor emits all rays at the simulation-step instant."*

> **Effect on attack surface:** LIO-SAM with deskew enabled is actually
> *more* vulnerable to scan-rotation attacks than the deskew-disabled
> variant — deskew gives the attacker a route to slip manipulated poses
> past the first line of defense, so the factor graph accepts rotated
> scans and diverges visibly rather than simply stalling. This is a
> counterintuitive security finding worth noting in Chapter 4.

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
        # Rewrite stamp to wall-clock. MAVROS/EKF3 run in wall-time; vision_pose
        # messages carrying sim-time stamps are rejected as stale.
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

> **Timestamp rewrite is load-bearing.** LIO-SAM publishes with sim-time
> stamps (because it runs with `use_sim_time:=true`). MAVROS and
> ArduCopter's EKF3 run in wall-time and reject vision_pose messages
> whose stamps deviate from the current wall-clock. This node must
> rewrite `header.stamp` to `self.get_clock().now()` before publishing.
> Without the rewrite, EKF3 never accepts vision updates and `EK3_SRC1`
> stays frozen.

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

Also create a second bridge config at `~/ros2_ws/bridge2.yaml`. `/clock`
is bridged by the drone 1 bridge — do not include it here (two publishers
on `/clock` cause race conditions in DDS).

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
> The deskew shim (Section 7b) reads from `/drone2/lidar/points` and republishes to `/drone2/lidar/points_timed` — that's what LIO-SAM drone 2 subscribes to.

---

## 10. Full Startup Order — Single Drone

Run each in a separate terminal in this exact order. **Order matters:**
`/clock` must be flowing before any node with `use_sim_time:=true` starts,
and the deskew shim must be running before LIO-SAM starts.

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
python3 ~/ros2_ws/src/lio_mavros_bridge.py
```

---

## 11. Full Startup Order — Two Drones

Requires the two-drone world SDF setup from Section 9. Each drone needs its own SITL instance, bridge, MAVROS, deskew shim, static TF publishers, LIO-SAM, and bridge node — all namespaced separately.

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

### One-time config (run once, not every launch)

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

### Launch sequence

```bash
# Terminal 1 — Gazebo (loads both drone models)
cd ~/sim/ardupilot_gazebo
gz sim worlds/iris_warehouse.sdf -r

# Terminal 2 — ros_gz bridge drone 1 (brings up /clock — MUST come early)
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:=$HOME/ros2_ws/bridge.yaml
# Verify: ros2 topic hz /clock

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
# MAV_SYSID: Both SITL instances default to system ID 1. Drone 2's MAVROS
# uses tgt_system:=2, so it will ignore packets from system 1. You MUST
# set MAV_SYSID to 2 on drone 2's SITL. MAV_SYSID survives param save.
#
# Verify with: output  (lists all active outputs)
# NOTE: output add does NOT survive SITL reboots. Re-add after every reboot.

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
# IMPORTANT: MAVROS drone1 may subscribe on /mavros/vision_pose/pose (no drone1/
# prefix) even when launched with ros_namespace. The remap below fixes this.
# Verify with: ros2 topic info /mavros/vision_pose/pose (Subscription count: 1)
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/lio_mavros_bridge.py --ros-args \
  -p drone_ns:=drone1 \
  -r /drone1/mavros/vision_pose/pose:=/mavros/vision_pose/pose

# Terminal 15 — LIO-SAM → MAVROS bridge drone 2
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/lio_mavros_bridge.py --ros-args -p drone_ns:=drone2
```

> **Launch order recap:** Gazebo → bridge (/clock flowing) → SITL → MAVROS
> → deskew shims → static TFs → LIO-SAM → lio_mavros_bridge. If LIO-SAM
> starts before its shim, it will bind to an untimed topic and emit the
> deskew warning. Kill and relaunch LIO-SAM (not the shim) if this
> happens.

### Verify both drones are up

```bash
# /clock flowing (catches bridge misconfig)
ros2 topic info /clock --verbose | grep "Publisher count"   # = 1

# Sensors flowing
ros2 topic hz /drone1/lidar/points         # ~1Hz (raw)
ros2 topic hz /drone1/lidar/points_timed   # ~1Hz (post-shim)
ros2 topic hz /drone2/lidar/points         # ~1Hz
ros2 topic hz /drone2/lidar/points_timed   # ~1Hz
ros2 topic hz /drone1/imu/data             # ~500Hz
ros2 topic hz /drone2/imu/data             # ~500Hz

# Deskew shim working — 'time' field present
ros2 topic echo /drone1/lidar/points_timed --field fields --once
# Must include: PointField(name='time', ...)

# LIO-SAM nodes properly namespaced (CRITICAL check)
ros2 node list | grep lio
# Should show /drone1/lio_sam_* and /drone2/lio_sam_*

# MAVROS connected
ros2 topic echo /mavros/state --once        # connected: true
ros2 topic echo /drone2/mavros/state --once # connected: true

# Vision pose reaching MAVROS (both Publisher and Subscription count >= 1)
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
param set EK3_SRC1_YAW 1
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
param set EK3_SRC1_YAW 1
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
Also verify vision_pose is reaching MAVROS:

```bash
ros2 topic hz /mavros/vision_pose/pose              # ~20Hz
ros2 topic hz /drone2/mavros/vision_pose/pose       # ~20Hz
```

### Step 3 — Switch to LIO-SAM nav (both drones)

Once odometry is publishing **and** `ros2 topic hz /mavros/vision_pose/pose` shows ~20 Hz, run in **each** MAVProxy console:

```bash
param set VISO_DELAY_MS 50
param set EK3_SRC1_POSXY 6
param set EK3_SRC1_VELXY 6
param set EK3_SRC1_VELZ 0
param set EK3_SRC1_YAW 6
param set GPS1_TYPE 0
param set VISO_TYPE 1
```

> `VISO_DELAY_MS 50` tells the EKF the expected latency of vision data. `EK3_SRC1_VELZ 0` disables GPS velocity Z (since GPS is now off). You should see **"EKF3 IMU0 is using external nav data"** in the MAVProxy console — this message only appears once per EKF initialization.

### Step 4 — Verify full pipeline

```bash
ros2 topic hz /drone1/lio_sam/mapping/odometry_incremental  # publishing
ros2 topic hz /drone2/lio_sam/mapping/odometry_incremental  # publishing
ros2 topic hz /drone1/mavros/vision_pose/pose               # ~20Hz
ros2 topic hz /drone2/mavros/vision_pose/pose               # ~20Hz
ros2 topic echo /mavros/state --once                        # armed: true, guided: true
ros2 topic echo /drone2/mavros/state --once                 # armed: true, guided: true
```

---

## 13. LIO-SAM Patches

### Patch 1: IMU dt guards (`imuPreintegration.cpp`)

Three places where `double dt = ...` is computed. After each, add guards
to handle non-monotonic timestamps from sim-time irregularities:

**Location A — optimization IMU loop (~line 392):**
```cpp
double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);
if (dt <= 0.0) { lastImuT_opt = imuTime; continue; }  // ← ADD
if (dt > 0.02) dt = 0.02;                               // ← ADD
```

**Location B — IMU queue replay loop (~line 461):**
```cpp
double dt = (lastImuQT < 0) ? (1.0 / 500.0) :(imuTime - lastImuQT);
if (dt <= 0.0) { lastImuQT = imuTime; continue; }  // ← ADD
if (dt > 0.02) dt = 0.02;                            // ← ADD
```

**Location C — imuHandler callback (~line 507):**
```cpp
double dt = (lastImuT_imu < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu);
if (dt <= 0.0) { lastImuT_imu = imuTime; return; }  // ← ADD (return, not continue)
if (dt > 0.02) dt = 0.02;                             // ← ADD
```

> **Why 0.02?** A 0.8s forward jump integrates gravity: 9.8 × 0.8 = 7.84 m/s. After 4 jumps → exceeds 30 m/s threshold → "Large velocity" reset → GTSAM crash.

### Patch 2: Odometry correction guard (`imuPreintegration.cpp`)

**Add member variable** near `lastImuT_imu` (~line 211):
```cpp
double lastImuT_imu = -1;
double lastCorrectionTime = -1;   // ← ADD
```

**Add guard** in `odometryHandler()` after `currentCorrectionTime`:
```cpp
double currentCorrectionTime = stamp2Sec(odomMsg->header.stamp);
if (currentCorrectionTime <= lastCorrectionTime) return;  // ← ADD
lastCorrectionTime = currentCorrectionTime;                // ← ADD
```

### Patch 3: IMU extrinsics for Gazebo

Gazebo IMU uses NED (`z = -9.8` when level). LIO-SAM expects ENU (`z = +9.8`). `extrinsicRot` flips accel/gyro. `extrinsicRPY` must be **identity** — Gazebo orientation is already correct; flipping it makes LIO-SAM think the drone is upside down.

In both `params_drone1.yaml` and `params_drone2.yaml`:
```yaml
extrinsicRot: [1.0,  0.0,  0.0,
               0.0, -1.0,  0.0,
               0.0,  0.0, -1.0]
extrinsicRPY: [1.0, 0.0, 0.0,
               0.0, 1.0, 0.0,
               0.0, 0.0, 1.0]
```

### Patch 4: Feature thresholds for indoor environments

```yaml
edgeThreshold: 0.5              # default 1.0
edgeFeatureMinValidNum: 2       # default 10
surfFeatureMinValidNum: 50      # default 100
```

### Patch 5: Sim time

Must be inside the **single** `/**:  ros__parameters:` block:
```yaml
/**:
  ros__parameters:
    use_sim_time: true
    # ... all other params
```

> **YAML pitfall:** Duplicate `/**:` blocks or duplicate `ros__parameters:` keys cause silent override — only the last survives.

### Rebuild

```bash
cd ~/ros2_ws && colcon build --packages-select lio_sam
```

---

## 14. Clock Synchronisation — Three Domains

Three distinct time domains coexist in this simulation:

1. **Gazebo sim-time** — published on `/clock` via ros_gz_bridge. Used by
   any ROS2 node launched with `use_sim_time:=true` (LIO-SAM, tf2
   listeners, static_transform_publisher, etc.).
2. **Wall-time** — used by MAVROS and RViz by default. Progresses at
   real clock speed regardless of simulation RTF.
3. **ArduCopter SITL internal time** — mostly wall-time, but delivered
   to MAVROS via MAVLink messages whose stamps reflect the FCU's
   internal notion of time.

### Consequences

- **LIO-SAM → MAVROS bridge** must rewrite timestamps to wall-clock
  before publishing `vision_pose`. EKF3 rejects vision_pose with
  sim-time stamps as stale. See Section 8.
- **LIO-SAM IMU preintegration** requires monotonic sim-time from
  `/clock`. When `/clock` stalls (bridge misconfig, RTF collapse) or
  jumps, imuPreintegration's `dt` computation produces negative or huge
  values, triggering `Large velocity, reset IMU-preintegration!`. The
  dt guards in Section 13 Patch 1 catch these and avoid GTSAM crashes,
  but the underlying `/clock` flow still needs to be healthy.
- **IMU source** used by LIO-SAM is `/drone1/imu/data` from Gazebo
  (~500 Hz at sim-time, ~235 Hz wall-clock at 47% RTF), **not**
  `/mavros/imu/data` which is bottlenecked through MAVLink stream rates
  (~2 Hz on ArduPilot defaults). This is intentional — LIO-SAM needs
  high-rate IMU for factor-graph optimization, and the Gazebo sensor
  plugin bypasses MAVLink entirely.

### Verifying clock health

```bash
# /clock has a publisher (bridge is mapping it correctly)
ros2 topic info /clock --verbose | grep "Publisher count"   # = 1
# Should NEVER be 0 — if it is, bridge.yaml is pointing to the wrong topic

# /clock is flowing at a sensible rate
ros2 topic hz /clock --window 500
# Expect several hundred Hz; 0.2 Hz or silence indicates bridge failure

# Gazebo RTF (check GUI window)
# Should be > 30% for stable operation
```

---

## 15. Autonomous Navigation (GUIDED Mode Waypoints)

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

---

## Architecture Overview

```
Gazebo Sim (iris_warehouse)
    │
    ├── VLP-16 LiDAR → /droneN/lidar/points/points (gz topic)
    ├── IMU sensor   → /world/.../imu                (gz topic)
    └── /clock                                       (gz topic)
           │
      ros_gz_bridge
           │
    ├── /droneN/lidar/points  (sensor_msgs/PointCloud2, no time field)
    ├── /droneN/imu/data      (sensor_msgs/Imu)
    └── /clock                (rosgraph_msgs/Clock, required by use_sim_time)
           │
    lidar_deskew_shim         (adds per-point 'time' field from azimuth)
           │
    /droneN/lidar/points_timed (sensor_msgs/PointCloud2 with time)
           │
        LIO-SAM                (deskew enabled; stable hover, factor graph converges)
           │
    /droneN/lio_sam/mapping/odometry
           │
    lio_mavros_bridge.py       (rewrites stamp to wall-clock)
           │
    /mavros/vision_pose/pose
           │
        MAVROS
           │
    ArduCopter SITL            (EKF3 External Nav via VISO_TYPE=1)
           │
    /mavros/setpoint_position/local  ← waypoint_patrol / navigator / attackers
```

---

## 16. Cooperative Inspection Mission (Byzantine Fault Experiments)

GPS-denied environments force drones to rely on cooperative data sharing — there is no external oracle to verify position claims. This mission exploits that trust dependency: drones divide a warehouse into waypoint zones, share completion status, and skip waypoints reported as visited by partners. A Byzantine drone can exploit this by falsely reporting coverage, leaving blind spots.

### Install the mission package

```bash
cp -r ~/ros2_ws/src/swarm_mission ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select swarm_mission
source install/setup.bash
```

### Architecture

```
/swarm/waypoint_status   (std_msgs/String, JSON)
    ^                         ^
    |  publishes               |  publishes
    |                         |
[drone1/waypoint_navigator]  [drone2/waypoint_navigator]
    |  subscribes              |  subscribes
    v                         v
/swarm/waypoint_status   (reads partner reports, skips covered waypoints)
    |
    v
[ground_truth_logger]   (compares reports vs actual LIO-SAM positions)
    |
    v
mission_results.csv     (thesis data: reported vs actual coverage)
```

Each drone reads its assigned waypoints from `config/waypoints.yaml`, flies to them via MAVROS setpoints, and publishes completion to the shared topic. The ground truth logger independently tracks drone positions from LIO-SAM odometry to verify what was actually visited.

### Prerequisites before launching mission

1. Both drones airborne via bootstrap procedure (Section 12)
2. Both drones in **GUIDED mode** — run `mode guided` in both MAVProxy consoles
3. Verify MAVROS local_position is publishing for both drones:
   ```bash
   ros2 topic hz /mavros/local_position/pose         # drone 1
   ros2 topic hz /drone2/mavros/local_position/pose   # drone 2
   ```

### Key technical notes

- **Setpoint rate:** ArduCopter GUIDED mode requires continuous setpoints at ≥10 Hz. The navigator uses a 20 Hz timer.
- **Position source:** Navigator reads from MAVROS `local_position/pose` (EKF output frame), NOT LIO-SAM odometry. This ensures setpoints and position readings are in the same frame.
- **Coordinate conversion:** Waypoints in `config/waypoints.yaml` are in Gazebo world coordinates. The navigator converts them to MAVROS local frame using spawn position offsets: `local = world - spawn`.
- **Spawn positions:** Drone 1 spawns at (-6, 0), drone 2 at (-3, 0) in the warehouse SDF. These must be passed as parameters.
- **No use_sim_time:** Do NOT pass `use_sim_time:=true` to the navigator nodes — it throttles the 20 Hz setpoint timer and ArduCopter stops responding.

### Run — Honest scenario (baseline)

After completing prerequisites above, run in separate terminals:

```bash
# Terminal 16 — Build (if not already built)
cd ~/ros2_ws && colcon build --packages-select swarm_mission && source install/setup.bash

# Terminal 17 — Drone 1 navigator
source ~/ros2_ws/install/setup.bash
ros2 run swarm_mission waypoint_navigator --ros-args \
  -p drone_id:=drone1 -p byzantine:=false \
  -p spawn_x:=-6.0 -p spawn_y:=0.0 \
  -p config_file:=$HOME/ros2_ws/src/swarm_mission/config/waypoints.yaml

# Terminal 18 — Drone 2 navigator
source ~/ros2_ws/install/setup.bash
ros2 run swarm_mission waypoint_navigator --ros-args \
  -p drone_id:=drone2 -p byzantine:=false \
  -p spawn_x:=-3.0 -p spawn_y:=0.0 \
  -p config_file:=$HOME/ros2_ws/src/swarm_mission/config/waypoints.yaml

# Terminal 19 — Ground truth logger
source ~/ros2_ws/install/setup.bash
ros2 run swarm_mission ground_truth_logger --ros-args \
  -p config_file:=$HOME/ros2_ws/src/swarm_mission/config/waypoints.yaml
```

### Run — Byzantine scenario (drone 2 lies)

Same as above, but change drone 2's terminal to:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run swarm_mission waypoint_navigator --ros-args \
  -p drone_id:=drone2 -p byzantine:=true \
  -p spawn_x:=-3.0 -p spawn_y:=0.0 \
  -p config_file:=$HOME/ros2_ws/src/swarm_mission/config/waypoints.yaml
```

In Byzantine mode, drone 2 immediately reports all its assigned waypoints as visited without flying to them. Drone 1 sees these reports and skips them, believing full coverage was achieved. The ground truth logger records the gap.

### Expected thesis results

| Scenario | Reported coverage | Actual coverage | Coverage gap |
|---|---|---|---|
| Honest (baseline) | 18/18 (100%) | 18/18 (100%) | 0 |
| Byzantine (drone 2) | 18/18 (100%) | 9/18 (50%) | 9 waypoints |

Results are appended to `~/ros2_ws/mission_results.csv` with per-waypoint breakdown and timestamps.

### Waypoint configuration

The `config/waypoints.yaml` uses 18 waypoints (9 per drone) with zones split along the x-axis:

- **Drone 1 (IDs 0-8):** Open area, world x = -8 to -13
- **Drone 2 (IDs 9-17):** Shelving area, world x = 1 to 9

Coordinate conversion reminder: `local = world - spawn`
- Drone 1 spawn (-6, 0): world x=-10 → local x=-4 (backward into open area)
- Drone 2 spawn (-3, 0): world x=6 → local x=9 (forward into shelves)

---

## 17. Layer 2 Attacks — Navigation Pipeline

These attacks target the LiDAR-inertial navigation pipeline (LIO-SAM → MAVROS → ArduCopter EKF3). They operate under the attacker model of a network participant that can publish/subscribe to any DDS topic (SROS2 disabled), but cannot intercept, modify, or reconfigure victim systems.

### Scan Manipulation — Gradual Rotation (lidar_manipulator)

Subscribes to the target drone's LiDAR topic, rotates the point cloud
around Z by a ramping angle (configurable rate in °/s), and republishes
on the same topic. LIO-SAM consumes both real and manipulated scans;
scan-to-map ICP converges to progressively wrong poses, the factor
graph accumulates bias, and imuPreintegration's dt/velocity guards
eventually fire, halting the pipeline.

With deskew enabled (Section 7b), LIO-SAM's internal map visibly
corrupts during the attack — multiple keyframe warehouse clusters
appear along a trajectory the drone never flew. Time-to-failure (TTF)
from attack start to `Waiting for IMU data` loop is ~1-2 seconds
across rotation rates 1-5 °/s.

```bash
source ~/ros2_ws/install/setup.bash
ros2 run swarm_mission lidar_manipulator --ros-args \
  -p target_drone:=drone1 \
  -p rotation_rate_dps:=1.0 \
  -p translation_rate_mps:=0.05
```

### Scan Manipulation Sweep Runner

`sweep_runner.sh` automates multi-trial experiments:

```bash
./sweep_runner.sh <rotation_rate_dps> <run_number>
# example: ./sweep_runner.sh 1.0 1
```

Runs a single trial with the given rotation rate and run ID. Writes a
row to `~/results/scan_sweep/summary.csv` containing the TTF and other
metrics. Preflight checks zombie processes before starting.

### Pre-flight Check

`preflight_check.sh` verifies the stack is healthy before running a
trial (IMU rate, LiDAR rate, vision_pose rate, drone pose, LIO-SAM odom
magnitude, zombie attackers, DDS shm). Exit code 0 = healthy.

```bash
./preflight_check.sh && ./sweep_runner.sh 1.0 1
```

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

**Findings:** The phantom wall appears in the LIO-SAM point cloud map (visible in RViz) but does not cause significant odometry drift when published as separate scans. LIO-SAM's ICP scan matching + IMU preintegration prior are robust enough to absorb the additional geometry. This is documented as a negative result — topic-level point cloud injection alone is insufficient to corrupt LIO-SAM's localisation when the real sensor data provides stronger geometric constraints.

### IMU Data Injection (imu_injector / imu_injector_v2)

Publishes fake IMU messages to the target drone's IMU topic. LIO-SAM
subscribes to `/droneN/imu/data` (the Gazebo-side stream, not MAVROS),
so any DDS publisher with topic access can inject alongside the real
sensor data.

#### Original — reused-timestamp variant (`imu_injector.py`)

The original implementation reuses the most recent real IMU message's
header stamp on each fake publication. This is the version described in
Section 3.5.5 of the dissertation as a negative result.

```bash
ros2 run swarm_mission imu_injector --ros-args \
  -p target_drone:=drone1 \
  -p mode:=spike \
  -p injection_rate:=500.0 \
  -p attack_duration:=30.0
```

**Findings (original):** blocked by LIO-SAM's `dt <= 0` guard in
imuPreintegration (Section 13 Patch 1). Each fake message arrives with a
timestamp identical to a real message LIO-SAM has already processed,
producing `dt = 0` which the patched guard rejects. No detectable drift
or crash.

> **Defense attribution caveat:** the blocking guard is a custom patch
> added to LIO-SAM during this work, not an upstream defense. The
> "blocked" classification therefore attributes the defense to the
> modification, not to the system as deployed. The v2 variant below
> tests this attribution empirically.

#### Advancing-timestamp bypass (`imu_injector_v2.py`)

Same payload modes (`bias`, `spike`, `flip`) but each injected message
carries a fresh sim-time timestamp from `self.get_clock().now()` rather
than reusing the latest real IMU's stamp. This guarantees every
injected message has `dt > 0` against the previously processed message,
bypassing the dt-guard entirely. Also subscribes to LIO-SAM's odometry
topic during the trial and records pose drift in the per-second metric
row — so the bypass's effect on LIO-SAM is measured directly rather
than inferred from drone behaviour.

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
| injection_rate | 500.0 | Hz — match or exceed real IMU rate (~500 Hz Gazebo) |
| baseline_duration | 15.0 | Seconds of pre-attack measurement |
| attack_duration | 45.0 | Seconds with injection active |
| recovery_duration | 15.0 | Seconds of post-attack measurement |
| output_dir | $HOME | CSV output location |

**Trial protocol:**

1. Cold restart full stack (Section 10)
2. Take off in GPS-hold mode (`VISO_TYPE=0`, GPS on) — same isolation
   choice as the rotation sweep, so the experiment measures the attack's
   primary effect on LIO-SAM independently of EKF3 vision-pose fusion
3. Run preflight, confirm clean baseline
4. Fire the v2 injector
5. Watch the live `[A]` log lines for `LIO drift` — the headline number
6. Cold restart between trials (the same map-corruption considerations
   apply as in the scan-manipulation sweep)

**Verdict matrix** — printed at trial end, also derivable from the CSV:

| LIO-SAM drift (m) | Drone drift (m) | Interpretation |
|------------------:|----------------:|---|
| < 1.0 | < 1.0 | dt-guard wasn't the only defense — LIO-SAM-internal velocity guard or factor-graph constraint intercepts. The original "blocked" classification stands but for a different reason |
| > 1.0 | < 1.0 | LIO-SAM corrupted; EKF3 innovation gate caught the bad vision_pose. Defense reattributes from "LIO-SAM dt-guard" to "EKF3 innovation gate" |
| > 1.0 | > 1.0 | Both layers failed — full attack success, dt-guard reclassified as insufficient |

**Output CSV:** `~/imu_injection_v2_metrics_<drone>_<timestamp>.csv` with
columns `timestamp, phase, phase_elapsed_s, inject_count, drone_drift_m,
lio_sam_drift_m, drone_x/y/z, lio_x/y/z`.

### QoS Profile Poisoning (qos_poisoner.py)

Creates RELIABLE subscribers on LIO-SAM's BEST_EFFORT odometry topic, forcing the DDS middleware to satisfy the stricter QoS policy. This causes the odometry output rate to degrade, starving the vision pose bridge below ArduCopter's EKF3 minimum threshold (0.5 Hz), which triggers a Land Mode failsafe.

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

**Manual attack (single command, no metrics):**

```bash
ros2 topic echo /drone1/lio_sam/mapping/odometry_incremental --qos-reliability reliable
```

---

## Troubleshooting

| Issue | Fix |
|---|---|
| `/clock` has 0 publishers | bridge.yaml is publishing to `/clock_raw` or similar. Fix `ros_topic_name` to `/clock` (Section 7), restart bridge |
| `Point cloud timestamp not available, deskew function disabled` | Deskew shim not running or started after LIO-SAM. Start shim first, then restart LIO-SAM (Section 7b) |
| LIO-SAM odom diverges to 100+ metres during hover | Deskew disabled. Ensure shim is running and LIO-SAM's `pointCloudTopic` is `/droneN/lidar/points_timed` |
| `Large velocity, reset IMU-preintegration!` warnings during baseline | `/clock` not flowing correctly — check Section 7 bridge config |
| `gtsam::IndeterminantLinearSystemException` crash | Apply clock jump patches from Section 13 and rebuild |
| `pcl::KdTreeFLANN::setInputCloud` empty cloud + mapOptimization segfault | Shim producing NaN time fields — ensure shim uses latest NaN-guard version (np.nan_to_num on output) |
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
| `/mavros/local_position/pose` not publishing | MAVROS not receiving vision pose — check `lio_mavros_bridge.py` is running AND rewriting stamp to wall-clock |
| LIO-SAM odometry z plummets to -300+ | Wrong extrinsics — `extrinsicRot` must flip NED→ENU, `extrinsicRPY` must be identity (Section 13) |
| LIO-SAM nodes collide / crash on drone2 launch | Use `namespace:=droneN` in launch command, not `PushRosNamespace` wrapper |
| Vision pose `Subscription count: 0` | MAVROS namespace mismatch — add remap to bridge (Terminal 14) |
| `use_sim_time` not taking effect | Check for duplicate `/**:` blocks or duplicate `ros__parameters:` keys in YAML |
| "Not enough features" → drift | Lower `edgeFeatureMinValidNum` to 2, `edgeThreshold` to 0.5 |
| MAVROS `connected: false` persists | Run `output add 127.0.0.1:14551` (drone 1) or `14561` (drone 2) in the correct MAVProxy console |
| MAVROS drone 2 `detected remote address 1.1` | Both SITL instances default to MAV_SYSID=1. Run `param set MAV_SYSID 2` and `param save` in drone 2's MAVProxy |
| Sweep runner reports zombie nodes | Kill survivors: `pkill -9 -f "swarm_mission.lidar_manipulator"` and same for logger; then `ros2 daemon stop; sleep 1; ros2 daemon start` to flush graph cache |
| MAVROS `Time jump detected` / EKF3 lane switches | Cosmetic under WSL2 clock irregularities; not blocking unless `/clock` rate drops below ~100 Hz |
| MAVROS IMU rate 1.9 Hz vs Gazebo IMU 470 Hz | Expected — LIO-SAM uses `/droneN/imu/data` (Gazebo sensor direct), not `/mavros/imu/data`. Don't try to "fix" the MAVROS rate |
| `imu_injector_v2` shows zero LIO drift during attack | Verify `use_sim_time` is propagating to the injector — `self.get_clock().now()` must return sim-time, not wall-time, or the bypass won't work as intended. Also raise `accel_bias_x` from 5.0 to 10.0 to amplify the effect |
| Multiple `imu_injector` instances zombie after Ctrl+C | Same cleanup pattern as the scan attacker: `pkill -9 -f "swarm_mission.imu_injector"` then `ros2 daemon stop; sleep 1; ros2 daemon start` |
| `imu_injector_v2` doesn't see LIO odometry | Check the subscription topic matches your namespace: `/drone1/lio_sam/mapping/odometry` (no `_incremental` suffix). v2 uses the post-mapOptimization output, not the imuPreintegration intermediate |
