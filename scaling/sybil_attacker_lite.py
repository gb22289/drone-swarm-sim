#!/usr/bin/env python3
"""
Sybil-4 attacker variant for the lite scaling simulator.

Models a coordinated attack where the adversary spawns K phantom drone
identities ("drone_{N+1} ... drone_{N+K}") and races them against the
real swarm. Each phantom owns ~total_wp/K waypoints and injects fake
"visited" reports at `spoof_delay` cadence. The real drones see the
spoofed reports first and skip those waypoints, producing a large
coverage gap even though their own missions report 100% complete.

Used to populate the `sybil_4phantoms` rows of Table VII.
"""
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SybilAttackerLite(Node):
    def __init__(self):
        super().__init__('sybil_attacker_lite')

        self.declare_parameter('n_real_drones', 10)
        self.declare_parameter('n_phantoms', 4)
        self.declare_parameter('total_wp', 90)
        self.declare_parameter('discovery_time_s', 3.0)
        self.declare_parameter('spoof_delay_s', 0.5)

        gp = self.get_parameter
        self.n_real = gp('n_real_drones').get_parameter_value().integer_value
        self.n_phantoms = gp('n_phantoms').get_parameter_value().integer_value
        self.total_wp = gp('total_wp').get_parameter_value().integer_value
        self.discovery_time = gp('discovery_time_s').get_parameter_value().double_value
        self.spoof_delay = gp('spoof_delay_s').get_parameter_value().double_value

        # Partition waypoints across phantoms — parallel injection.
        self.phantom_assignments = []
        per_phantom = self.total_wp // self.n_phantoms
        leftover = self.total_wp % self.n_phantoms
        cursor = 0
        for i in range(self.n_phantoms):
            size = per_phantom + (1 if i < leftover else 0)
            wps = list(range(cursor, cursor + size))
            cursor += size
            self.phantom_assignments.append({
                'id': f'drone{self.n_real + 1 + i}',
                'wps': wps,
                'idx': 0,
                'last_inject_t': 0.0,
            })

        self.pub = self.create_publisher(String, '/swarm/waypoint_status', 10)
        self.start_t = time.time()
        self.tick = self.create_timer(0.05, self._step)

        total_inject_time = per_phantom * self.spoof_delay
        self.get_logger().info(
            f"Parallel Sybil armed: N_real={self.n_real}, phantoms={self.n_phantoms}, "
            f"total_wp={self.total_wp}, discovery={self.discovery_time}s, "
            f"delay={self.spoof_delay}s. Each phantom owns "
            f"~{per_phantom} WPs (~{total_inject_time:.1f}s of injection)."
        )

    def _step(self):
        if time.time() - self.start_t < self.discovery_time:
            return
        now = time.time()
        for ph in self.phantom_assignments:
            if ph['idx'] >= len(ph['wps']):
                continue
            if now - ph['last_inject_t'] < self.spoof_delay:
                continue
            wp = ph['wps'][ph['idx']]
            msg = String()
            msg.data = json.dumps({
                'drone_id': ph['id'],
                'waypoint_id': wp,
                'timestamp': now,
                # Lie: claim we visited so the real drones skip.
                'actually_visited': True,
            })
            self.pub.publish(msg)
            ph['idx'] += 1
            ph['last_inject_t'] = now


def main():
    rclpy.init()
    node = SybilAttackerLite()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
