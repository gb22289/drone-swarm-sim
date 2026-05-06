#!/usr/bin/env python3
"""
swarm_lite_sim.py
=================
Lightweight discrete-event swarm simulator for Layer 1 coordination
attack scaling experiments. Replaces the ArduCopter SITL + Gazebo +
LIO-SAM stack with a parameterised travel-time model, while keeping
the ROS 2 / DDS coordination layer (topic, QoS, JSON payload, and
status_callback handler logic) byte-identical to the full stack.

Each VirtualDrone:
  * picks the next unvisited waypoint from its assigned zone
  * sleeps for travel_time ~ N(mean_flight_s, std_flight_s)
  * publishes a real /swarm/waypoint_status message
    (same JSON format as the full-stack waypoint_navigator)
  * subscribes to /swarm/waypoint_status and updates partner_visits
    from any drone_id != self.drone_id
  * also publishes the same payload on /swarm/ground_truth, a separate
    topic that only the ground_truth_logger_lite subscribes to
    (the attacker scripts publish on /swarm/waypoint_status only and
    so cannot poison the ground-truth channel).

The existing attacker scripts (network_attacker.py, etc.) work
unchanged because the coordination topic interface is identical.

Usage
-----
ros2 run python3 swarm_lite_sim.py --num-drones 5 --waypoints-per-drone 9 \
        --mean-flight 3.0 --std-flight 0.5 --mission-timeout 90 --seed 1
"""

import argparse
import json
import random
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


# ---------------------------------------------------------------------------
# Zone allocation
# ---------------------------------------------------------------------------
def static_contiguous(num_drones, waypoints_per_drone):
    """Drone i (0-indexed) is assigned waypoints [i*K, (i+1)*K).

    Matches the dissertation's full-stack allocation (drone1: 0-8,
    drone2: 9-17) and is the default for direct comparability.
    """
    return [list(range(i * waypoints_per_drone,
                       (i + 1) * waypoints_per_drone))
            for i in range(num_drones)]


def static_round_robin(num_drones, waypoints_per_drone):
    """Drone i (0-indexed) gets waypoints {i, i+N, i+2N, ...}.

    Strided allocation. Available via --zone-mode round_robin for
    the optional follow-up experiment on whether dynamic allocation
    affects attack efficacy.
    """
    total = num_drones * waypoints_per_drone
    return [[w for w in range(total) if w % num_drones == i]
            for i in range(num_drones)]


ZONE_ALLOCATORS = {
    'contiguous': static_contiguous,
    'round_robin': static_round_robin,
}


# ---------------------------------------------------------------------------
# Virtual drone node
# ---------------------------------------------------------------------------
class VirtualDrone(Node):
    def __init__(self, drone_index, zone,
                 mean_flight_s=3.5, std_flight_s=0.5,
                 takeoff_delay_s=1.0):
        super().__init__(f'virtual_drone_{drone_index}')
        self.drone_id = f'drone{drone_index}'
        self.zone = list(zone)
        self.mean = mean_flight_s
        self.std = std_flight_s

        self.visited = set()
        # Renamed from partner_visited to reflect the dissertation's
        # actual semantics: ALL reports about ANY drone visiting ANY
        # waypoint are accepted at face value, including reports forged
        # under self.drone_id. This is the core vulnerability.
        self.reported_visited = set()
        self.travel_remaining = takeoff_delay_s
        self.current_target = None
        self.mission_complete = False

        self.status_pub = self.create_publisher(
            String, '/swarm/waypoint_status', 10)
        self.gt_pub = self.create_publisher(
            String, '/swarm/ground_truth', 10)

        self.create_subscription(
            String, '/swarm/waypoint_status',
            self.status_callback, 10)

        self.tick_period = 0.1
        self.create_timer(self.tick_period, self.tick)
        self.get_logger().info(
            f'{self.drone_id} ready, zone={self.zone}, '
            f'flight ~ N({self.mean}, {self.std})s')

    # -- core loop -----------------------------------------------------
    def pick_next(self):
        for wp in self.zone:
            if wp not in self.visited and wp not in self.reported_visited:
                return wp
        return None

    def tick(self):
        if self.mission_complete:
            return

        if self.current_target is None:
            nxt = self.pick_next()
            if nxt is None:
                self.mission_complete = True
                self.get_logger().info(
                    f'{self.drone_id} mission complete '
                    f'(visited={len(self.visited)} '
                    f'reported_known={len(self.reported_visited)})')
                return
            self.current_target = nxt
            self.travel_remaining = max(0.5,
                                         random.gauss(self.mean, self.std))
            return

        self.travel_remaining -= self.tick_period
        if self.travel_remaining <= 0.0:
            self.publish_arrival(self.current_target)
            self.visited.add(self.current_target)
            self.current_target = None

    # -- pub/sub -------------------------------------------------------
    def publish_arrival(self, wp):
        payload = json.dumps({'drone_id': self.drone_id,
                              'waypoint_id': wp})
        msg = String()
        msg.data = payload
        self.status_pub.publish(msg)
        self.gt_pub.publish(msg)

    def status_callback(self, msg):
        """Mirrors the dissertation's waypoint_navigator.status_callback:
        accepts any drone_id/waypoint_id pair without source verification.
        Crucially, this includes reports attributed to self.drone_id --
        the protocol cannot tell a forged self-report from a legitimate
        echo of its own publication. This is the property that the
        selective_denial attack and the central vulnerability claim
        of the dissertation rely on."""
        try:
            d = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        wp = d.get('waypoint_id')
        if wp is not None:
            self.reported_visited.add(wp)


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser()
    p.add_argument('--num-drones', type=int, default=2)
    p.add_argument('--waypoints-per-drone', type=int, default=9)
    p.add_argument('--mean-flight', type=float, default=3.5,
                   help='Mean inter-waypoint flight time in seconds. '
                        'Derived from the attacker injection budget: at '
                        'spoof_delay=0.5s and discovery=3s, the attacker '
                        'finishes injecting 18 WPs at t=12s. For drone1 to '
                        'visit ~3 of its 9 zone WPs before being fully '
                        'poisoned (matching dissertation gap=12), we need '
                        'T >= 8/3s analytically, plus ~30% headroom for '
                        'tick-quantisation slack (~T=3.5).')
    p.add_argument('--std-flight', type=float, default=0.5,
                   help='Std-dev of flight time in seconds')
    p.add_argument('--takeoff-delay', type=float, default=1.0,
                   help='Initial delay before first leg (s)')
    p.add_argument('--mission-timeout', type=float, default=120.0)
    p.add_argument('--seed', type=int, default=None,
                   help='RNG seed for reproducibility')
    p.add_argument('--zone-mode', type=str, default='contiguous',
                   choices=['contiguous', 'round_robin'],
                   help='Zone allocation: contiguous (default, matches '
                        'full-stack dissertation) or round_robin (strided)')
    args = p.parse_args()

    if args.seed is not None:
        random.seed(args.seed)

    rclpy.init()

    allocator = ZONE_ALLOCATORS[args.zone_mode]
    zones = allocator(args.num_drones, args.waypoints_per_drone)
    drones = [
        VirtualDrone(i + 1, zones[i],
                     mean_flight_s=args.mean_flight,
                     std_flight_s=args.std_flight,
                     takeoff_delay_s=args.takeoff_delay)
        for i in range(args.num_drones)
    ]

    ex = MultiThreadedExecutor(num_threads=max(2, args.num_drones + 1))
    for d in drones:
        ex.add_node(d)

    start = time.time()
    try:
        while rclpy.ok():
            ex.spin_once(timeout_sec=0.1)
            elapsed = time.time() - start
            if all(d.mission_complete for d in drones):
                print(f'\n[lite_sim] All drones complete at t={elapsed:.1f}s')
                # Publish mission-complete sentinel on /swarm/ground_truth so
                # the logger flushes its summary deterministically rather than
                # waiting for an idle window.
                sentinel = String()
                sentinel.data = json.dumps({'event': 'mission_complete'})
                drones[0].gt_pub.publish(sentinel)
                # Spin briefly to ensure the sentinel is delivered before we
                # tear down the executor.
                flush_start = time.time()
                while time.time() - flush_start < 1.5:
                    ex.spin_once(timeout_sec=0.1)
                break
            if elapsed > args.mission_timeout:
                print(f'\n[lite_sim] Mission timeout at t={elapsed:.1f}s')
                # Still publish sentinel so logger can flush a partial result
                sentinel = String()
                sentinel.data = json.dumps({'event': 'mission_complete'})
                drones[0].gt_pub.publish(sentinel)
                flush_start = time.time()
                while time.time() - flush_start < 1.5:
                    ex.spin_once(timeout_sec=0.1)
                break
    finally:
        for d in drones:
            d.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
