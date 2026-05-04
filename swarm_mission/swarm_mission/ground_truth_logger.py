#!/usr/bin/env python3
"""Ground Truth Logger — uses MAVROS local_position, converts to world frame."""
import json, math, time, csv, os, rclpy, yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

SPAWN_POSITIONS = {'drone1': (-6.0, 0.0), 'drone2': (-3.0, 0.0)}

class GroundTruthLogger(Node):
    def __init__(self):
        super().__init__('ground_truth_logger')
        self.declare_parameter('output_file', '~/ros2_ws/mission_results.csv')
        self.declare_parameter('config_file', '')
        output_path = self.get_parameter('output_file').get_parameter_value().string_value
        self.output_file = os.path.expanduser(output_path)
        config_path = self.get_parameter('config_file').get_parameter_value().string_value
        if not config_path:
            from ament_index_python.packages import get_package_share_directory
            config_path = os.path.join(get_package_share_directory('swarm_mission'), 'config', 'waypoints.yaml')
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        self.all_waypoints = {wp['id']: wp for wp in config['waypoints']}
        self.tolerance = config['mission']['tolerance']
        self.altitude = config['mission']['altitude']
        self.drone_poses = {}
        self.reported_visits = []
        self.ground_truth_visits = {}
        self.summaries_received = []
        self.start_time = time.time()
        self.odom_counts = {'drone1': 0, 'drone2': 0}
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
            lambda msg: self.pose_callback(msg, 'drone1'), qos)
        self.create_subscription(PoseStamped, '/drone2/mavros/local_position/pose',
            lambda msg: self.pose_callback(msg, 'drone2'), qos)
        self.create_subscription(String, '/swarm/waypoint_status', self.status_callback, 10)
        self.create_subscription(String, '/swarm/mission_summary', self.summary_callback, 10)
        self.timer = self.create_timer(0.5, self.check_proximity)
        self.stats_timer = self.create_timer(10.0, self.print_stats)
        self.get_logger().info(f'Ground Truth Logger started — output: {self.output_file}\n  Using MAVROS local_position (BEST_EFFORT QoS)')

    def local_to_world(self, drone_id, lx, ly):
        sx, sy = SPAWN_POSITIONS.get(drone_id, (0.0, 0.0))
        return lx + sx, ly + sy

    def pose_callback(self, msg, drone_id):
        p = msg.pose.position
        self.drone_poses[drone_id] = (p.x, p.y, p.z)
        self.odom_counts[drone_id] += 1

    def status_callback(self, msg):
        try:
            data = json.loads(msg.data)
            data['log_time'] = time.time() - self.start_time
            self.reported_visits.append(data)
            wp_id = data['waypoint_id']
            drone_id = data['drone_id']
            actually = data.get('actually_visited', 'unknown')
            wp = self.all_waypoints.get(wp_id, {})
            self.get_logger().info(f'REPORT: {drone_id} says wp {wp_id} ({wp.get("label","?")}) visited [actual={actually}]')
        except (json.JSONDecodeError, KeyError):
            pass

    def summary_callback(self, msg):
        try:
            data = json.loads(msg.data)
            drone_id = data.get('drone_id', '?')
            if any(s.get('drone_id') == drone_id for s in self.summaries_received):
                return
            self.summaries_received.append(data)
            self.get_logger().info(f'Received mission summary from {drone_id}')
            if len(self.summaries_received) >= 2:
                self.produce_final_report()
        except (json.JSONDecodeError, KeyError):
            pass

    def check_proximity(self):
        for drone_id, (lx, ly, lz) in self.drone_poses.items():
            wx, wy = self.local_to_world(drone_id, lx, ly)
            for wp_id, wp in self.all_waypoints.items():
                key = (drone_id, wp_id)
                if key in self.ground_truth_visits:
                    continue
                dist = math.sqrt((wx - wp['x'])**2 + (wy - wp['y'])**2 + (lz - self.altitude)**2)
                if dist < self.tolerance:
                    self.ground_truth_visits[key] = time.time() - self.start_time
                    self.get_logger().info(f'GROUND TRUTH: {drone_id} at wp {wp_id} ({wp["label"]}) dist={dist:.2f}m (local:{lx:.1f},{ly:.1f} world:{wx:.1f},{wy:.1f})')

    def print_stats(self):
        d1 = self.odom_counts.get('drone1', 0)
        d2 = self.odom_counts.get('drone2', 0)
        gt = len(self.ground_truth_visits)
        self.get_logger().info(f'Stats: drone1 poses={d1}, drone2 poses={d2}, ground_truth={gt}, reports={len(self.reported_visits)}')

    def produce_final_report(self):
        self.get_logger().info('=' * 70)
        self.get_logger().info('FINAL GROUND TRUTH REPORT')
        self.get_logger().info('=' * 70)
        reported_set = {v['waypoint_id'] for v in self.reported_visits}
        actual_set = {wp_id for (_, wp_id) in self.ground_truth_visits}
        total = len(self.all_waypoints)
        rc = len(reported_set)
        ac = len(actual_set)
        false_reports = reported_set - actual_set
        self.get_logger().info(f'Total waypoints:     {total}')
        self.get_logger().info(f'Reported visited:    {rc}/{total} ({100*rc/total:.0f}%)')
        self.get_logger().info(f'Actually visited:    {ac}/{total} ({100*ac/total:.0f}%)')
        self.get_logger().info(f'Coverage gap:        {rc - ac} waypoints')
        self.get_logger().info(f'False reports:       {sorted(false_reports)}')
        self.get_logger().info('')
        self.get_logger().info(f'{"WP":>3} {"Label":<22} {"Reported":>10} {"Actual":>10} {"Status":>12}')
        self.get_logger().info('-' * 62)
        rows = []
        for wp_id in sorted(self.all_waypoints.keys()):
            wp = self.all_waypoints[wp_id]
            rpt = wp_id in reported_set
            act = wp_id in actual_set
            if rpt and act: status = 'OK'
            elif rpt and not act: status = 'FALSE CLAIM'
            elif not rpt and act: status = 'UNREPORTED'
            else: status = 'MISSED'
            self.get_logger().info(f'{wp_id:>3} {wp["label"]:<22} {"YES" if rpt else "NO":>10} {"YES" if act else "NO":>10} {status:>12}')
            rows.append({'waypoint_id': wp_id, 'label': wp['label'], 'x': wp['x'], 'y': wp['y'], 'reported': rpt, 'actually_visited': act, 'status': status})
        self.get_logger().info('=' * 70)
        any_byz = any(s.get('byzantine', False) for s in self.summaries_received)
        self.write_csv(rows, 'byzantine' if any_byz else 'honest', rc, ac, total)

    def write_csv(self, rows, scenario, reported, actual, total):
        exists = os.path.exists(self.output_file)
        with open(self.output_file, 'a', newline='') as f:
            w = csv.writer(f)
            if not exists:
                w.writerow(['timestamp','scenario','total_waypoints','reported_visited','actually_visited','coverage_gap','reported_pct','actual_pct','waypoint_id','label','reported','actually_visited_gt','status'])
            ts = time.strftime('%Y-%m-%d %H:%M:%S')
            for r in rows:
                w.writerow([ts, scenario, total, reported, actual, reported-actual, f'{100*reported/total:.1f}', f'{100*actual/total:.1f}', r['waypoint_id'], r['label'], r['reported'], r['actually_visited'], r['status']])
        self.get_logger().info(f'Results written to {self.output_file}')

def main():
    rclpy.init()
    node = GroundTruthLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.produce_final_report()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
