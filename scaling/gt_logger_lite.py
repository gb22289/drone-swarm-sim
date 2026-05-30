#!/usr/bin/env python3
"""
Ground-truth logger for the lite scaling simulator. Subscribes to
`/swarm/waypoint_status`, distinguishes attacker-spoofed claims from
real virtual-drone visits using the `actually_visited` flag and the
mission-complete sentinel from each drone, and writes a single summary
row to a CSV when all expected drones have reported in.

Output schema (one row per run, appended):
  scenario, n_drones, total_wp, gt_visited, reported_visited,
  coverage_gap, gap_pct, false_claims, mission_time_s
"""
import csv
import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GtLoggerLite(Node):
    def __init__(self):
        super().__init__('gt_logger_lite')

        self.declare_parameter('n_drones', 10)
        self.declare_parameter('total_wp', 90)
        self.declare_parameter('scenario', 'none')
        self.declare_parameter('output_csv', '/tmp/lite_sim.csv')

        gp = self.get_parameter
        self.n_drones = gp('n_drones').get_parameter_value().integer_value
        self.total_wp = gp('total_wp').get_parameter_value().integer_value
        self.scenario = gp('scenario').get_parameter_value().string_value
        self.output_csv = gp('output_csv').get_parameter_value().string_value

        self.start_t = time.time()
        self.gt_visited = set()                # truly-visited waypoints
        self.reported_visited = set()          # everything seen on the bus
        self.false_claims = 0                  # actually_visited=False, or claim a wp no real drone visited
        self.complete_drones = set()

        self.sub = self.create_subscription(
            String, '/swarm/waypoint_status', self._on_msg, 50)
        self.flush = self.create_timer(0.5, self._maybe_finalize)

        self.get_logger().info(
            f"gt_logger_lite started (N={self.n_drones}, total_wp={self.total_wp}, "
            f"scenario={self.scenario}, output={self.output_csv})"
        )

    def _on_msg(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        # End-of-mission sentinel from a virtual drone — authoritative GT.
        if data.get('event') == 'mission_complete':
            drone_id = data.get('drone_id')
            if drone_id:
                self.complete_drones.add(drone_id)
                for wp in data.get('visited', []):
                    if isinstance(wp, int):
                        self.gt_visited.add(wp)
            return

        wp = data.get('waypoint_id')
        if not isinstance(wp, int):
            return
        self.reported_visited.add(wp)
        # Anything carrying actually_visited=False is by definition a false claim.
        if data.get('actually_visited', True) is False:
            self.false_claims += 1

    def _maybe_finalize(self):
        if len(self.complete_drones) < self.n_drones:
            return
        # All virtual drones reported in.
        self.flush.cancel()

        # Reconcile: any reported wp that wasn't actually visited counts as a
        # false claim too (covers spoofs that lied with actually_visited=True).
        spoofed = self.reported_visited - self.gt_visited
        false_claims = max(self.false_claims, len(spoofed))

        coverage_gap = max(0, self.total_wp - len(self.gt_visited))
        gap_pct = round(100.0 * coverage_gap / self.total_wp, 2)
        mission_time = round(time.time() - self.start_t, 2)

        row = {
            'scenario': self.scenario,
            'n_drones': self.n_drones,
            'total_wp': self.total_wp,
            'gt_visited': len(self.gt_visited),
            'reported_visited': len(self.reported_visited),
            'coverage_gap': coverage_gap,
            'gap_pct': gap_pct,
            'false_claims': false_claims,
            'mission_time_s': mission_time,
        }
        self._append_csv(row)
        self.get_logger().info("Mission-complete sentinel received. Summary written.")
        # Let the runner script tear us down.

    def _append_csv(self, row):
        os.makedirs(os.path.dirname(self.output_csv) or '.', exist_ok=True)
        new_file = not os.path.exists(self.output_csv)
        with open(self.output_csv, 'a', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(row.keys()))
            if new_file:
                w.writeheader()
            w.writerow(row)


def main():
    rclpy.init()
    node = GtLoggerLite()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
