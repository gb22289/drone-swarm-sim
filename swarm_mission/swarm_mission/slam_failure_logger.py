#!/usr/bin/env python3
"""
SLAM Failure Logger — detects the scan-manipulation failure signal.

Failure is declared on the first observation of ANY of:
  1. "Large velocity, reset IMU-preintegration!"   from lio_sam_imuPreintegration
  2. "Waiting for IMU data"                         from lio_sam_imageProjection
     (we require this to repeat ≥3 times within 3s so a single transient hit
      doesn't trigger; the failure mode is an indefinite loop)
  3. odometry stream stops arriving for ≥3 seconds
     (dead-man timer — SLAM has silently died without producing either warning)

Also listens to /attack/status messages from the attacker to anchor the
attack_start_time.

Writes one CSV row to the specified output path:
  rotation_rate, run_id, attack_start_t, first_injection_t,
  first_fail_t, fail_signal, time_to_failure_s, duration_s

Usage:
  ros2 run <pkg> slam_failure_logger --ros-args \
      -p rotation_rate_dps:=2.0 \
      -p run_id:=rate_2.0_run_1 \
      -p output_csv:=/tmp/results/scan_sweep/summary.csv \
      -p test_window_s:=120.0
"""
import csv
import os
import re
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from rcl_interfaces.msg import Log
from nav_msgs.msg import Odometry
from std_msgs.msg import String


LARGE_VEL_RE = re.compile(r"Large velocity.*reset IMU-preintegration", re.IGNORECASE)
WAITING_IMU_RE = re.compile(r"Waiting for IMU data", re.IGNORECASE)

# Anchors emitted by the existing lidar_manipulator node (mode=drift).
ATTACK_PHASE_RE = re.compile(r"ATTACK PHASE.*scan manipulation", re.IGNORECASE)
RECOVERY_PHASE_RE = re.compile(r"RECOVERY PHASE.*Manipulation stopped", re.IGNORECASE)


class SlamFailureLogger(Node):
    def __init__(self):
        super().__init__("slam_failure_logger")

        self.declare_parameter("rotation_rate_dps", 1.0)
        self.declare_parameter("run_id", "run")
        self.declare_parameter("output_csv", "/tmp/scan_sweep_summary.csv")
        self.declare_parameter("odom_topic", "/drone1/lio_sam/mapping/odometry")
        self.declare_parameter("test_window_s", 120.0)
        self.declare_parameter("odom_silence_s", 3.0)
        # Sustained loop: require hits to span at least min_span_s seconds
        # with no inter-hit gap larger than max_gap_s. A brief burst at attack
        # onset (many hits in <1s then silence) no longer counts.
        self.declare_parameter("waiting_imu_min_count", 10)
        self.declare_parameter("waiting_imu_window_s", 30.0)   # trailing buffer
        self.declare_parameter("waiting_imu_min_span_s", 5.0)
        self.declare_parameter("waiting_imu_max_gap_s", 2.0)

        self.rate_dps = float(self.get_parameter("rotation_rate_dps").value)
        self.run_id = str(self.get_parameter("run_id").value)
        self.output_csv = Path(str(self.get_parameter("output_csv").value))
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.test_window = float(self.get_parameter("test_window_s").value)
        self.odom_silence = float(self.get_parameter("odom_silence_s").value)
        self.waiting_min = int(self.get_parameter("waiting_imu_min_count").value)
        self.waiting_window = float(self.get_parameter("waiting_imu_window_s").value)
        self.waiting_min_span = float(self.get_parameter("waiting_imu_min_span_s").value)
        self.waiting_max_gap = float(self.get_parameter("waiting_imu_max_gap_s").value)

        # Event tracking.
        self.logger_start = time.time()
        self.attack_start_t = None
        self.first_injection_t = None
        self.first_fail_t = None
        self.fail_signal = None
        self.last_odom_t = None
        self.waiting_hits = []  # list of wall-clock timestamps

        # /rosout is transient_local durability.
        rosout_qos = QoSProfile(
            depth=1000,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Log, "/rosout", self.rosout_cb, rosout_qos)

        # LIO-SAM publishes odometry with BEST_EFFORT reliability.
        # A RELIABLE subscriber would refuse the connection ("incompatible QoS")
        # and last_odom_t would stay None forever, disarming the dead-man check.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            Odometry, self.odom_topic, self.odom_cb, sensor_qos
        )
        self.create_subscription(
            String, "/attack/status", self.attack_status_cb, 10
        )

        # 2 Hz housekeeping.
        self.create_timer(0.5, self.tick)

        self.get_logger().warn("=" * 60)
        self.get_logger().warn("SLAM FAILURE LOGGER")
        self.get_logger().warn(f"  run_id          : {self.run_id}")
        self.get_logger().warn(f"  rotation_rate   : {self.rate_dps} dps")
        self.get_logger().warn(f"  odom topic      : {self.odom_topic}")
        self.get_logger().warn(f"  test window     : {self.test_window}s")
        self.get_logger().warn(f"  output csv      : {self.output_csv}")
        self.get_logger().warn("=" * 60)

    def attack_status_cb(self, msg: String):
        # Kept for forward-compatibility with attackers that publish
        # /attack/status directly. The existing lidar_manipulator does not,
        # so anchoring is done via /rosout in rosout_cb instead.
        data = msg.data
        if "event=armed" in data and self.attack_start_t is None:
            self.attack_start_t = time.time()
            self.get_logger().info(f"[ANCHOR] attack armed at {self.attack_start_t:.3f}")
        if "event=first_injection" in data and self.first_injection_t is None:
            self.first_injection_t = time.time()
            self.get_logger().info(
                f"[ANCHOR] first injection at {self.first_injection_t:.3f}"
            )

    def odom_cb(self, _msg: Odometry):
        self.last_odom_t = time.time()

    def rosout_cb(self, msg: Log):
        name = msg.name or ""
        text = msg.msg or ""
        now = time.time()

        # Anchor events from the attacker (lidar_manipulator).
        if "lidar_manipulator" in name or ATTACK_PHASE_RE.search(text):
            if ATTACK_PHASE_RE.search(text) and self.attack_start_t is None:
                self.attack_start_t = now
                # First injection and attack start are effectively simultaneous
                # for lidar_manipulator (it starts injecting on every incoming
                # scan once ATTACK phase begins).
                if self.first_injection_t is None:
                    self.first_injection_t = now
                self.get_logger().info(
                    f"[ANCHOR] attack phase started at t+"
                    f"{now - self.logger_start:.2f}s"
                )
                return
            if RECOVERY_PHASE_RE.search(text):
                self.get_logger().info(
                    f"[ANCHOR] attacker entered recovery at t+"
                    f"{now - self.logger_start:.2f}s"
                )
                return

        # Failure signatures must come from LIO-SAM nodes specifically so
        # unrelated nodes saying "Waiting for IMU data" don't trigger us.
        if "lio_sam" not in name:
            return

        if LARGE_VEL_RE.search(text) and self.first_fail_t is None:
            self.first_fail_t = now
            self.fail_signal = "large_velocity_reset"
            self.get_logger().warn(
                f"[FAIL] large-velocity reset from {name} at t+"
                f"{now - self.logger_start:.2f}s"
            )

        if WAITING_IMU_RE.search(text):
            self.waiting_hits.append(now)
            # Drop hits older than the trailing buffer.
            cutoff = now - self.waiting_window
            self.waiting_hits = [t for t in self.waiting_hits if t >= cutoff]

            if self.first_fail_t is not None:
                return
            if len(self.waiting_hits) < self.waiting_min:
                return
            span = self.waiting_hits[-1] - self.waiting_hits[0]
            if span < self.waiting_min_span:
                return
            # Check that the hits are actually continuous — no long quiet
            # stretches. A brief burst that then goes silent for >max_gap
            # doesn't count as a loop.
            gaps = [b - a for a, b in zip(self.waiting_hits, self.waiting_hits[1:])]
            max_gap = max(gaps) if gaps else 0.0
            if max_gap > self.waiting_max_gap:
                return

            self.first_fail_t = self.waiting_hits[0]
            self.fail_signal = "waiting_for_imu_loop"
            self.get_logger().warn(
                f"[FAIL] waiting-for-IMU loop confirmed "
                f"({len(self.waiting_hits)} hits spanning {span:.1f}s, "
                f"max gap {max_gap:.1f}s)"
            )

    def tick(self):
        now = time.time()

        # Odometry dead-man.
        if (
            self.last_odom_t is not None
            and self.first_fail_t is None
            and self.attack_start_t is not None
            and (now - self.last_odom_t) > self.odom_silence
        ):
            # Only arm this check once we've seen at least one odom message AND
            # the attack has started, to avoid false positives during warmup.
            self.first_fail_t = self.last_odom_t
            self.fail_signal = "odometry_silence"
            self.get_logger().warn(
                f"[FAIL] odometry silent for >{self.odom_silence}s "
                f"(last msg at t+{self.last_odom_t - self.logger_start:.2f}s)"
            )

        # Finish conditions: failure detected, OR test window elapsed.
        finished = self.first_fail_t is not None
        elapsed_since_attack = (
            (now - self.attack_start_t) if self.attack_start_t else 0.0
        )
        timed_out = (
            self.attack_start_t is not None
            and elapsed_since_attack > self.test_window
        )
        if finished or timed_out:
            if timed_out and not finished:
                self.fail_signal = "survived_window"
            self.write_row(now)
            self.get_logger().warn("[DONE] shutting down logger")
            # Trigger clean shutdown.
            rclpy.shutdown()

    def write_row(self, now: float):
        self.output_csv.parent.mkdir(parents=True, exist_ok=True)
        write_header = not self.output_csv.exists()

        anchor = self.first_injection_t or self.attack_start_t or self.logger_start
        if self.first_fail_t is not None:
            ttf = self.first_fail_t - anchor
        else:
            ttf = now - anchor

        row = {
            "rotation_rate_dps": self.rate_dps,
            "run_id": self.run_id,
            "attack_start_t": f"{self.attack_start_t:.3f}" if self.attack_start_t else "",
            "first_injection_t": f"{self.first_injection_t:.3f}" if self.first_injection_t else "",
            "first_fail_t": f"{self.first_fail_t:.3f}" if self.first_fail_t else "",
            "fail_signal": self.fail_signal or "unknown",
            "time_to_failure_s": f"{ttf:.3f}",
            "duration_s": f"{now - self.logger_start:.3f}",
        }

        with open(self.output_csv, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(row.keys()))
            if write_header:
                writer.writeheader()
            writer.writerow(row)

        self.get_logger().warn(f"[CSV] appended row: {row}")


def main():
    rclpy.init()
    node = SlamFailureLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
