#!/usr/bin/env python3
"""
Waypoint Navigator for GPS-denied drone swarm inspection mission.

Uses MAVROS local_position/pose for navigation (EKF frame = setpoint frame).
Publishes setpoints at 20 Hz to keep ArduCopter GUIDED mode responsive.
Mission logic runs at 2 Hz.
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import yaml
import os


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        self.declare_parameter('drone_id', 'drone1')
        self.declare_parameter('byzantine', False)
        self.declare_parameter('config_file', '')
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)

        self.drone_id = self.get_parameter('drone_id').get_parameter_value().string_value
        self.byzantine = self.get_parameter('byzantine').get_parameter_value().bool_value
        config_path = self.get_parameter('config_file').get_parameter_value().string_value
        self.spawn_x = self.get_parameter('spawn_x').get_parameter_value().double_value
        self.spawn_y = self.get_parameter('spawn_y').get_parameter_value().double_value

        if not config_path:
            from ament_index_python.packages import get_package_share_directory
            config_path = os.path.join(
                get_package_share_directory('swarm_mission'), 'config', 'waypoints.yaml')

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        self.altitude = config['mission']['altitude']
        self.tolerance = config['mission']['tolerance']
        self.hold_time = config['mission']['hold_time']
        self.timeout = config['mission']['timeout']

        self.all_waypoints = {wp['id']: wp for wp in config['waypoints']}
        self.my_waypoint_ids = config['zones'].get(self.drone_id, [])

        self.reported_visited = {}
        self.actually_visited = set()
        self.current_target = None
        self.current_pose = None
        self.mission_started = False
        self.mission_complete = False
        self.start_time = None
        self.hold_start = None

        self.target_x = None
        self.target_y = None
        self.target_z = None

        self.returning_home = False
        drone_num = int(self.drone_id.replace('drone','')) if self.drone_id.startswith('drone') else 1
        self.return_altitude = float(self.altitude) + (drone_num-1) * 3
        if self.drone_id == 'drone1':
            self.mavros_prefix = ''
        else:
            self.mavros_prefix = f'/{self.drone_id}'

        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            f'{self.mavros_prefix}/mavros/setpoint_position/local',
            10)

        self.status_pub = self.create_publisher(
            String, '/swarm/waypoint_status', 10)

        # QoS profile matching MAVROS (BEST_EFFORT)
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        self.create_subscription(
            PoseStamped,
            f'{self.mavros_prefix}/mavros/local_position/pose',
            self.pose_callback,
            qos_sensor)

        self.create_subscription(
            String, '/swarm/waypoint_status', self.status_callback, 10)

        self.setpoint_timer = self.create_timer(0.05, self.publish_setpoint_tick)
        self.mission_timer = self.create_timer(0.5, self.mission_tick)

        mode_str = "BYZANTINE" if self.byzantine else "HONEST"
        self.get_logger().info(
            f'[{self.drone_id}] Navigator started ({mode_str}) — '
            f'{len(self.my_waypoint_ids)} waypoints assigned: {self.my_waypoint_ids}\n'
            f'  Spawn offset: ({self.spawn_x}, {self.spawn_y})\n'
            f'  Using MAVROS local_position for navigation (20 Hz setpoints)')

    def pose_callback(self, msg):
        self.current_pose = msg.pose
        if not self.mission_started:
            self.mission_started = True
            self.start_time = time.time()
            p = msg.pose.position
            self.get_logger().info(
                f'[{self.drone_id}] Position received — '
                f'local({p.x:.1f}, {p.y:.1f}, {p.z:.1f}) — mission starting')

    def status_callback(self, msg):
        try:
            data = json.loads(msg.data)
            wp_id = data['waypoint_id']
            reporter = data['drone_id']
            if wp_id not in self.reported_visited:
                self.reported_visited[wp_id] = reporter
                if reporter != self.drone_id:
                    self.get_logger().info(
                        f'[{self.drone_id}] {reporter} reports wp {wp_id} visited — skipping')
        except (json.JSONDecodeError, KeyError):
            pass

    def world_to_local(self, world_x, world_y):
        return world_x - self.spawn_x, world_y - self.spawn_y

    def publish_setpoint_tick(self):
        if self.target_x is None:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.target_x
        msg.pose.position.y = self.target_y
        msg.pose.position.z = self.target_z
        msg.pose.orientation.w = 1.0
        self.setpoint_pub.publish(msg)

    def mission_tick(self):
        if self.mission_complete or not self.mission_started:
            return

        if self.returning_home:
            self.target_x = 0.0
            self.target_y = 0.0
            self.target_z = float(self.return_altitude)
            if self.current_pose is not None:
                dist = self.distance_to(0.0, 0.0, self.return_altitude)
                if not hasattr(self, '_last_rth_log') or time.time() - self._last_rth_log > 5.0:
                    p = self.current_pose.position
                    self.get_logger().info(
                        f'[{self.drone_id}] Returning home — '
                        f'pos({p.x:.1f},{p.y:.1f},{p.z:.1f}) dist={dist:.1f}m')
                    self._last_rth_log = time.time()
                if dist < self.tolerance:
                    self.get_logger().info(
                        f'[{self.drone_id}] Arrived at home position — ready for next run')
                    self.mission_complete = True
                    self.target_x = 0.0
                    self.target_y = 0.0
                    self.target_z = float(self.return_altitude)
            return

        elapsed = time.time() - self.start_time
        if elapsed > self.timeout:
            self.get_logger().warn(
                f'[{self.drone_id}] Mission TIMEOUT after {elapsed:.0f}s')
            self.finish_mission()
            return

        if self.current_target is None:
            self.pick_next_waypoint()

        if self.current_target is None:
            self.get_logger().info(
                f'[{self.drone_id}] All assigned waypoints covered')
            self.finish_mission()
            return

        wp = self.all_waypoints[self.current_target]

        if self.byzantine and self.current_target not in self.actually_visited:
            self.report_waypoint_visited(self.current_target)
            self.get_logger().warn(
                f'[{self.drone_id}] BYZANTINE — falsely reporting wp '
                f'{self.current_target} ({wp["label"]})')
            self.current_target = None
            return

        local_x, local_y = self.world_to_local(wp['x'], wp['y'])
        self.target_x = float(local_x)
        self.target_y = float(local_y)
        self.target_z = float(self.altitude)

        if self.current_pose is not None:
            dist = self.distance_to(local_x, local_y, self.altitude)
            if not hasattr(self, '_last_dist_log') or time.time() - self._last_dist_log > 5.0:
                p = self.current_pose.position
                self.get_logger().info(
                    f'[{self.drone_id}] Flying to wp {self.current_target} — '
                    f'pos({p.x:.1f},{p.y:.1f},{p.z:.1f}) '
                    f'target({local_x:.1f},{local_y:.1f},{self.altitude}) '
                    f'dist={dist:.1f}m')
                self._last_dist_log = time.time()

            if dist < self.tolerance:
                if self.hold_start is None:
                    self.hold_start = time.time()
                    self.get_logger().info(
                        f'[{self.drone_id}] Reached wp {self.current_target} ({wp["label"]}) '
                        f'— holding {self.hold_time}s (dist={dist:.2f}m)')
                if time.time() - self.hold_start >= self.hold_time:
                    self.actually_visited.add(self.current_target)
                    self.report_waypoint_visited(self.current_target)
                    self.get_logger().info(
                        f'[{self.drone_id}] Waypoint {self.current_target} ({wp["label"]}) CONFIRMED')
                    self.hold_start = None
                    self.current_target = None
            else:
                self.hold_start = None

    def pick_next_waypoint(self):
        for wp_id in self.my_waypoint_ids:
            if wp_id not in self.reported_visited:
                self.current_target = wp_id
                wp = self.all_waypoints[wp_id]
                local_x, local_y = self.world_to_local(wp['x'], wp['y'])
                self.get_logger().info(
                    f'[{self.drone_id}] Targeting wp {wp_id} ({wp["label"]}) '
                    f'world({wp["x"]},{wp["y"]}) → local({local_x:.1f},{local_y:.1f})')
                return
        self.current_target = None

    def report_waypoint_visited(self, wp_id):
        data = {
            'drone_id': self.drone_id,
            'waypoint_id': wp_id,
            'timestamp': time.time(),
            'actually_visited': wp_id in self.actually_visited,
        }
        msg = String()
        msg.data = json.dumps(data)
        self.status_pub.publish(msg)

    def distance_to(self, x, y, z):
        if self.current_pose is None:
            return float('inf')
        dx = self.current_pose.position.x - x
        dy = self.current_pose.position.y - y
        dz = self.current_pose.position.z - z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def finish_mission(self):
        elapsed = time.time() - self.start_time if self.start_time else 0
        total = len(self.my_waypoint_ids)
        visited = len(self.actually_visited)
        partner = sum(1 for w in self.my_waypoint_ids
                      if w in self.reported_visited and self.reported_visited[w] != self.drone_id)

        self.get_logger().info('=' * 60)
        self.get_logger().info(f'[{self.drone_id}] MISSION COMPLETE')
        self.get_logger().info(f'  Mode: {"BYZANTINE" if self.byzantine else "HONEST"}')
        self.get_logger().info(f'  Time: {elapsed:.1f}s')
        self.get_logger().info(f'  Assigned: {total} | Visited: {visited} | Partner: {partner}')
        self.get_logger().info(f'  Visited list: {sorted(self.actually_visited)}')
        self.get_logger().info('=' * 60)

        summary = {
            'drone_id': self.drone_id,
            'byzantine': self.byzantine,
            'elapsed_seconds': elapsed,
            'assigned': self.my_waypoint_ids,
            'actually_visited': sorted(self.actually_visited),
            'reported_visited': {str(k): v for k, v in self.reported_visited.items()},
        }
        pub = self.create_publisher(String, '/swarm/mission_summary', 10)
        msg = String()
        msg.data = json.dumps(summary)
        for _ in range(5):
            pub.publish(msg)
            time.sleep(0.1)

        if not self.returning_home:
            self.returning_home = True
            self.get_logger().info(
                f'[{self.drone_id}] Returning to home position (0, 0, {self.altitude})...')


def main():
    rclpy.init()
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.finish_mission()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
