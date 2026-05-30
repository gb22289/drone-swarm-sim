#!/usr/bin/env python3
"""
Lite-sim virtual drone — discrete-event surrogate for the full Gazebo
swarm. Each instance owns a contiguous zone of waypoints and "flies" to
each one in turn; flight time is sampled from N(mean, stddev). When a
drone reaches its waypoint it publishes a `/swarm/waypoint_status`
message in the exact same JSON envelope used by the full waypoint_navigator,
so the same attackers and ground-truth logger work unchanged.

This is what was used to produce the N=5 and N=10 scaling rows in the
paper (Table VII). It models coordination-layer dynamics only — no SLAM,
no MAVROS, no Gazebo — so a full sweep runs in a few minutes per
configuration on commodity hardware.
"""
import json
import random
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VirtualDroneLite(Node):
    def __init__(self):
        super().__init__('virtual_drone_lite')

        self.declare_parameter('drone_id', 'drone1')
        self.declare_parameter('zone_start', 0)
        self.declare_parameter('zone_end', 9)            # exclusive
        self.declare_parameter('total_wp', 90)
        self.declare_parameter('flight_mean_s', 3.5)
        self.declare_parameter('flight_stddev_s', 0.5)
        self.declare_parameter('seed', 0)

        gp = self.get_parameter
        self.drone_id = gp('drone_id').get_parameter_value().string_value
        self.zone_start = gp('zone_start').get_parameter_value().integer_value
        self.zone_end = gp('zone_end').get_parameter_value().integer_value
        self.total_wp = gp('total_wp').get_parameter_value().integer_value
        self.flight_mean = gp('flight_mean_s').get_parameter_value().double_value
        self.flight_stddev = gp('flight_stddev_s').get_parameter_value().double_value
        seed = gp('seed').get_parameter_value().integer_value

        # Per-drone RNG so a (seed, drone_id) pair is reproducible.
        self.rng = random.Random((seed, self.drone_id))

        self.zone = list(range(self.zone_start, self.zone_end))
        self.reported_known = set()                   # all wp ids we've heard about
        self.visited = []                             # wp ids we actually visited
        self.zone_idx = 0
        self.next_event_t = None
        self.done = False

        self.pub = self.create_publisher(String, '/swarm/waypoint_status', 10)
        self.sub = self.create_subscription(
            String, '/swarm/waypoint_status', self._eavesdrop, 50)
        self.tick = self.create_timer(0.05, self._step)

        self.get_logger().info(
            f"{self.drone_id} ready, zone={self.zone}, "
            f"flight ~ N({self.flight_mean}, {self.flight_stddev})s"
        )
        self._schedule_next()

    # ------------------------------------------------------------------
    # Coordination channel — same envelope as the full waypoint_navigator
    # ------------------------------------------------------------------
    def _eavesdrop(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        wp = data.get('waypoint_id')
        if isinstance(wp, int):
            self.reported_known.add(wp)

    def _publish(self, wp_id):
        msg = String()
        msg.data = json.dumps({
            'drone_id': self.drone_id,
            'waypoint_id': wp_id,
            'timestamp': time.time(),
            'actually_visited': True,
        })
        self.pub.publish(msg)

    # ------------------------------------------------------------------
    # Travel-time event loop
    # ------------------------------------------------------------------
    def _schedule_next(self):
        if self.zone_idx >= len(self.zone):
            self._finish()
            return
        # Truncated normal: clamp to >= 0.1s so the scheduler stays sane.
        dt = max(0.1, self.rng.gauss(self.flight_mean, self.flight_stddev))
        self.next_event_t = time.time() + dt

    def _step(self):
        if self.done or self.next_event_t is None:
            return
        if time.time() < self.next_event_t:
            return
        wp = self.zone[self.zone_idx]
        self.visited.append(wp)
        self._publish(wp)
        self.zone_idx += 1
        self._schedule_next()

    def _finish(self):
        if self.done:
            return
        self.done = True
        self.next_event_t = None
        self.get_logger().info(
            f"{self.drone_id} mission complete "
            f"(visited={len(self.visited)} reported_known={len(self.reported_known)})"
        )
        # Sentinel for the gt_logger to detect end-of-mission.
        sentinel = String()
        sentinel.data = json.dumps({
            'drone_id': self.drone_id,
            'event': 'mission_complete',
            'visited': self.visited,
        })
        self.pub.publish(sentinel)


def main():
    rclpy.init()
    node = VirtualDroneLite()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
