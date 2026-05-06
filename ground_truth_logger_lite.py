#!/usr/bin/env python3
"""
ground_truth_logger_lite.py
===========================
Companion to swarm_lite_sim.py. Subscribes to two channels:

  * /swarm/ground_truth     -- the tamper-proof channel that virtual
                               drones publish their *real* visits to.
                               Attackers do NOT publish here.
  * /swarm/waypoint_status  -- the public coordination channel that
                               attackers can poison freely.

When the mission ends (timeout reached or no new events for a quiet
window), it appends a single summary row to a CSV. Schema:

    scenario, n_drones, total_wp, gt_visited, reported_visited,
    coverage_gap, gap_pct, false_claims, mission_time_s

`coverage_gap` is the dissertation's primary metric: the count of
waypoints the swarm believes were covered but were not actually
visited. `false_claims` is reported_visited minus gt_visited.

Usage
-----
python3 ground_truth_logger_lite.py \
        --num-drones 5 --total-wp 45 \
        --scenario coverage_spoof --output ./lite_results.csv \
        --mission-timeout 90
"""

import argparse
import csv
import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GroundTruthLogger(Node):
    def __init__(self, n_drones, total_wp, scenario_label,
                 output_csv, mission_timeout=120.0,
                 quiet_window_s=5.0):
        super().__init__('gt_logger_lite')
        self.n_drones = n_drones
        self.total_wp = total_wp
        self.scenario = scenario_label
        self.output_csv = output_csv
        self.mission_timeout = mission_timeout
        self.quiet_window_s = quiet_window_s

        self.gt_visits = set()         # what really happened
        self.reported_visits = set()    # what /swarm/waypoint_status claims
        self.start = time.time()
        self.last_event = self.start
        self.flushed = False

        self.create_subscription(
            String, '/swarm/ground_truth', self.gt_cb, 50)
        self.create_subscription(
            String, '/swarm/waypoint_status', self.status_cb, 50)
        self.create_timer(0.5, self.check_done)

        self.get_logger().info(
            f'gt_logger_lite started (N={n_drones}, total_wp={total_wp}, '
            f'scenario={scenario_label}, output={output_csv})')

    def gt_cb(self, msg):
        try:
            d = json.loads(msg.data)
            # Sentinel from the simulator: mission complete, flush immediately
            if d.get('event') == 'mission_complete':
                if not self.flushed:
                    elapsed = time.time() - self.start
                    self.write_summary(elapsed)
                    self.flushed = True
                    self.get_logger().info(
                        'Mission-complete sentinel received. Summary written.')
                    rclpy.shutdown()
                return
            self.gt_visits.add(d['waypoint_id'])
            self.last_event = time.time()
        except Exception:
            pass

    def status_cb(self, msg):
        try:
            d = json.loads(msg.data)
            self.reported_visits.add(d['waypoint_id'])
        except Exception:
            pass

    def check_done(self):
        if self.flushed:
            return
        elapsed = time.time() - self.start
        idle = time.time() - self.last_event
        if elapsed > self.mission_timeout or \
           (idle > self.quiet_window_s and elapsed > self.quiet_window_s):
            self.write_summary(elapsed)
            self.flushed = True
            self.get_logger().info('Summary written. Shutting down.')
            rclpy.shutdown()

    def write_summary(self, mission_time):
        gap = self.total_wp - len(self.gt_visits)
        false_claims = len(self.reported_visits - self.gt_visits)
        row = {
            'scenario': self.scenario,
            'n_drones': self.n_drones,
            'total_wp': self.total_wp,
            'gt_visited': len(self.gt_visits),
            'reported_visited': len(self.reported_visits),
            'coverage_gap': gap,
            'gap_pct': round(100.0 * gap / max(1, self.total_wp), 2),
            'false_claims': false_claims,
            'mission_time_s': round(mission_time, 2),
        }
        new_file = not os.path.exists(self.output_csv)
        with open(self.output_csv, 'a', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(row.keys()))
            if new_file:
                w.writeheader()
            w.writerow(row)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--num-drones', type=int, required=True)
    p.add_argument('--total-wp', type=int, required=True)
    p.add_argument('--scenario', type=str, default='honest')
    p.add_argument('--output', type=str, default='lite_results.csv')
    p.add_argument('--mission-timeout', type=float, default=120.0)
    p.add_argument('--quiet-window', type=float, default=8.0,
                   help='Seconds with no new events to declare end-of-mission '
                        '(fallback only; the simulator normally signals '
                        'completion via a sentinel on /swarm/ground_truth)')
    args = p.parse_args()

    rclpy.init()
    n = GroundTruthLogger(args.num_drones, args.total_wp,
                          args.scenario, args.output,
                          args.mission_timeout, args.quiet_window)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
