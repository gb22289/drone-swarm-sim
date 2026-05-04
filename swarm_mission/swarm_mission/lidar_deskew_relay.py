#!/usr/bin/env python3
"""
LiDAR Deskew Relay — adds synthetic per-point timestamps to Gazebo VLP-16 scans.

The Gazebo ros_gz_bridge publishes PointCloud2 without a 'time' field.
LIO-SAM needs this field for deskewing (correcting motion distortion during
the LiDAR sweep). Without it, the map becomes streaky over time.

This node sits between the bridge and LIO-SAM:
  Bridge → /droneX/lidar/points_raw → [this node adds 'time'] → /droneX/lidar/points

Setup:
  1. Change bridge config to publish to /droneX/lidar/points_raw
  2. LIO-SAM still reads from /droneX/lidar/points (unchanged)
  3. Run this node for each drone

Usage:
  ros2 run swarm_mission lidar_deskew_relay --ros-args -p drone_ns:=drone1
  ros2 run swarm_mission lidar_deskew_relay --ros-args -p drone_ns:=drone2
"""
import numpy as np
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


class LidarDeskewRelay(Node):
    def __init__(self):
        super().__init__('lidar_deskew_relay')

        self.declare_parameter('drone_ns', 'drone1')
        self.declare_parameter('scan_duration', 0.1)  # VLP-16 at 10 Hz = 0.1s per sweep

        drone_ns = self.get_parameter('drone_ns').value
        self.scan_duration = self.get_parameter('scan_duration').value

        input_topic = f'/{drone_ns}/lidar/points_raw'
        output_topic = f'/{drone_ns}/lidar/points'

        # Match the bridge QoS (RELIABLE)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=5)

        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5)

        self.pub = self.create_publisher(PointCloud2, output_topic, qos)
        self.sub = self.create_subscription(
            PointCloud2, input_topic, self.callback, qos_sub)

        self.count = 0
        self.get_logger().info(
            f'Deskew relay: {input_topic} → {output_topic} '
            f'(adding time field, {self.scan_duration:.3f}s sweep)')

    def callback(self, msg: PointCloud2):
        n_points = msg.width * msg.height
        if n_points == 0:
            self.pub.publish(msg)
            return

        old_step = msg.point_step
        old_data = np.frombuffer(msg.data, dtype=np.uint8)

        # Find ring field for proper time assignment
        field_map = {f.name: f for f in msg.fields}
        has_ring = 'ring' in field_map

        # New point_step: old + 4 bytes for time (float32)
        # Align to 4 bytes
        time_offset = old_step
        new_step = old_step + 4

        # Build new fields list (copy old + add time)
        new_fields = list(msg.fields) + [
            PointField(
                name='time',
                offset=time_offset,
                datatype=PointField.FLOAT32,
                count=1)]

        # Reshape old data
        old_array = old_data.reshape(n_points, old_step)

        # Create new buffer with space for time field
        new_data = np.zeros((n_points, new_step), dtype=np.uint8)
        new_data[:, :old_step] = old_array

        # Compute synthetic time offsets
        if has_ring:
            # Extract ring values for proper per-ring timing
            ring_field = field_map['ring']
            ring_off = ring_field.offset
            # ring is uint16 (datatype=4)
            rings = np.zeros(n_points, dtype=np.uint16)
            for i in range(n_points):
                rings[i] = struct.unpack_from('<H', old_array[i], ring_off)[0]

            # Sort time by ring: each ring fires at a slightly different time
            # VLP-16 has 16 rings, each ring's points span the full azimuth
            # Approximate: points within same ring get time based on their
            # index within that ring (azimuth order)
            times = np.zeros(n_points, dtype=np.float32)
            for ring_id in range(16):
                ring_mask = rings == ring_id
                ring_count = ring_mask.sum()
                if ring_count > 0:
                    # Distribute time evenly across azimuth within this ring
                    times[ring_mask] = np.linspace(
                        0, self.scan_duration, ring_count, dtype=np.float32)
        else:
            # No ring info — just distribute linearly across all points
            times = np.linspace(
                0, self.scan_duration, n_points, dtype=np.float32)

        # Pack time values into new data
        time_bytes = times.tobytes()
        for i in range(n_points):
            new_data[i, time_offset:time_offset + 4] = np.frombuffer(
                time_bytes[i * 4:(i + 1) * 4], dtype=np.uint8)

        # Build output message
        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = n_points
        out.fields = new_fields
        out.is_bigendian = False
        out.point_step = new_step
        out.row_step = new_step * n_points
        out.data = new_data.tobytes()
        out.is_dense = msg.is_dense

        self.pub.publish(out)
        self.count += 1

        if self.count % 20 == 1:
            self.get_logger().info(
                f'Relayed scan #{self.count}: {n_points} pts, '
                f'added time field [0, {self.scan_duration:.3f}]s')


def main(args=None):
    rclpy.init(args=args)
    node = LidarDeskewRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f'Shutting down — {node.count} scans relayed')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
