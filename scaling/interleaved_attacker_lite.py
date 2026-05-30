#!/usr/bin/env python3
"""
Interleaved injection variant for the lite scaling simulator.

Where the Sybil-4 attacker tries to outrun the real swarm with parallel
phantom identities, the interleaved attacker reuses *existing* drone
identities (drone1 .. droneN) and waits for the real visit before
injecting a duplicate report. Because each fake claim is paired with a
genuine visit to the same waypoint, no coverage gap actually opens — the
attack succeeds in injecting traffic but is a no-op against the
ground-truth metric.

This variant is included to show that not every coordination-layer
attack scales to a coverage gap: the result is the negative control row
in Table VII.
"""
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class InterleavedAttackerLite(Node):
    def __init__(self):
        super().__init__('interleaved_attacker_lite')

        self.declare_parameter('n_real_drones', 10)
        self.declare_parameter('total_wp', 90)
        self.declare_parameter('discovery_time_s', 3.0)
        self.declare_parameter('echo_delay_s', 0.1)

        gp = self.get_parameter
        self.n_real = gp('n_real_drones').get_parameter_value().integer_value
        self.total_wp = gp('total_wp').get_parameter_value().integer_value
        self.discovery_time = gp('discovery_time_s').get_parameter_value().double_value
        self.echo_delay = gp('echo_delay_s').get_parameter_value().double_value

        self.pub = self.create_publisher(String, '/swarm/waypoint_status', 10)
        self.sub = self.create_subscription(
            String, '/swarm/waypoint_status', self._on_msg, 50)
        self.start_t = time.time()
        self.pending = []   # list of (fire_at, drone_id, wp_id)
        self.echoed = set()
        self.tick = self.create_timer(0.05, self._drain)

        self.get_logger().info(
            f"Interleaved attacker armed: N_real={self.n_real}, "
            f"total_wp={self.total_wp}, discovery={self.discovery_time}s, "
            f"echo_delay={self.echo_delay}s"
        )

    def _on_msg(self, msg):
        if time.time() - self.start_t < self.discovery_time:
            return
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if data.get('event') == 'mission_complete':
            return
        wp = data.get('waypoint_id')
        drone_id = data.get('drone_id', '')
        if not isinstance(wp, int) or not drone_id:
            return
        # Only echo real-drone reports, never our own echoes.
        if (drone_id, wp) in self.echoed:
            return
        self.echoed.add((drone_id, wp))
        self.pending.append((time.time() + self.echo_delay, drone_id, wp))

    def _drain(self):
        now = time.time()
        still = []
        for fire_at, drone_id, wp in self.pending:
            if now < fire_at:
                still.append((fire_at, drone_id, wp))
                continue
            msg = String()
            msg.data = json.dumps({
                'drone_id': drone_id,
                'waypoint_id': wp,
                'timestamp': now,
                # Honest echo — we don't claim a fake visit.
                'actually_visited': False,
            })
            self.pub.publish(msg)
        self.pending = still


def main():
    rclpy.init()
    node = InterleavedAttackerLite()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
