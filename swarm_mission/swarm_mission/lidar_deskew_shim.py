#!/usr/bin/env python3
"""
lidar_deskew_shim.py

Subscribes to a PointCloud2 LiDAR stream that lacks per-point timestamps,
synthesizes a `time` field per point from azimuth angle, and republishes on a
separate topic. This enables LIO-SAM's deskew function to operate correctly
when using Gazebo's gpu_lidar sensor (which does not emit per-point timing
information).

Handles point clouds with mixed-datatype fields (e.g., float32 xyz + uint16
ring + float32 intensity) by using structured numpy dtypes rather than
flat arrays.

The synthesized time field represents "seconds elapsed since scan start",
distributed linearly across azimuth to mimic the firing order of a real
rotating LiDAR (e.g., Velodyne VLP-16). Default scan_period 0.1s matches
the 10 Hz expectation baked into LIO-SAM's reference config.

Topic wiring:
  Gazebo -> ros_gz_bridge -> /drone1/lidar/points  (raw, no timestamps)
                          -> this shim
                          -> /drone1/lidar/points_timed  (with time field)
                          -> LIO-SAM

Author: Patrik Mohai, MEng dissertation "Breaking Trust in the Swarm"
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
)
from sensor_msgs.msg import PointCloud2, PointField


_TYPE_SIZES = {
    PointField.INT8: 1, PointField.UINT8: 1,
    PointField.INT16: 2, PointField.UINT16: 2,
    PointField.INT32: 4, PointField.UINT32: 4,
    PointField.FLOAT32: 4, PointField.FLOAT64: 8,
}

_TYPE_NP = {
    PointField.INT8: np.int8, PointField.UINT8: np.uint8,
    PointField.INT16: np.int16, PointField.UINT16: np.uint16,
    PointField.INT32: np.int32, PointField.UINT32: np.uint32,
    PointField.FLOAT32: np.float32, PointField.FLOAT64: np.float64,
}


def _structured_dtype_from_fields(fields, point_step):
    """Build a numpy structured dtype matching the PointCloud2 layout,
    including padding bytes so total itemsize == point_step."""
    spec = []
    cursor = 0
    pad_idx = 0
    for f in sorted(fields, key=lambda x: x.offset):
        if f.offset > cursor:
            spec.append((f'_pad{pad_idx}', 'u1', f.offset - cursor))
            pad_idx += 1
        np_dt = _TYPE_NP[f.datatype]
        if f.count == 1:
            spec.append((f.name, np_dt))
        else:
            spec.append((f.name, np_dt, f.count))
        cursor = f.offset + _TYPE_SIZES[f.datatype] * f.count
    if point_step > cursor:
        spec.append((f'_pad{pad_idx}', 'u1', point_step - cursor))
    return np.dtype(spec)


class LidarDeskewShim(Node):
    def __init__(self):
        super().__init__('lidar_deskew_shim')

        self.declare_parameter('input_topic', '/drone1/lidar/points')
        self.declare_parameter('output_topic', '/drone1/lidar/points_timed')
        self.declare_parameter('scan_period', 0.1)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.scan_period = float(self.get_parameter('scan_period').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.pub = self.create_publisher(PointCloud2, self.output_topic, qos)
        self.sub = self.create_subscription(
            PointCloud2, self.input_topic, self.cb, qos,
        )

        self.frames = 0
        self.logged_schema = False
        self.get_logger().info(
            f"Deskew shim active: {self.input_topic} -> {self.output_topic} "
            f"(scan_period={self.scan_period}s)"
        )

    def cb(self, msg: PointCloud2):
        n = msg.width * msg.height
        if n == 0:
            return

        # Parse cloud using a structured dtype (handles mixed field datatypes)
        try:
            dt = _structured_dtype_from_fields(msg.fields, msg.point_step)
            arr = np.frombuffer(bytes(msg.data), dtype=dt)
        except Exception as e:
            self.get_logger().error(f"Failed to build/apply dtype: {e}")
            return

        # One-shot schema log so you can confirm parsing worked
        if not self.logged_schema:
            self.logged_schema = True
            self.get_logger().info(
                f"Parsed schema: point_step={msg.point_step}, "
                f"fields=[{', '.join(f.name for f in msg.fields)}], "
                f"first_point_raw={arr[0]}"
            )

        try:
            x = arr['x'].astype(np.float32)
            y = arr['y'].astype(np.float32)
        except ValueError as e:
            self.get_logger().error(f"Missing x or y in cloud: {e}")
            return

        # Synthesize time from azimuth, mimicking rotating-LiDAR firing order
        azimuth = np.arctan2(y, x)
        min_az = float(np.min(azimuth))
        az_shifted = np.mod((azimuth - min_az) + 2.0 * np.pi, 2.0 * np.pi)
        times = (az_shifted / (2.0 * np.pi) * self.scan_period).astype(np.float32)

        # Build the extended data buffer: original point_step bytes, then 4 bytes of time
        old_step = msg.point_step
        new_step = old_step + 4
        orig = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, old_step)
        time_bytes = times.view(np.uint8).reshape(n, 4)
        new_data = np.ascontiguousarray(
            np.concatenate([orig, time_bytes], axis=1)
        )

        # Build extended field schema — keep original offsets, append time
        new_fields = list(msg.fields) + [PointField(
            name='time', offset=old_step,
            datatype=PointField.FLOAT32, count=1,
        )]

        # Assemble and publish
        new_msg = PointCloud2()
        new_msg.header = msg.header
        new_msg.height = msg.height
        new_msg.width = msg.width
        new_msg.fields = new_fields
        new_msg.is_bigendian = msg.is_bigendian
        new_msg.point_step = new_step
        new_msg.row_step = new_step * msg.width
        new_msg.data = new_data.tobytes()
        new_msg.is_dense = msg.is_dense
        self.pub.publish(new_msg)

        self.frames += 1
        if self.frames == 1 or self.frames % 20 == 0:
            self.get_logger().info(
                f"frame {self.frames}: {n} pts, "
                f"time range [{times.min():.4f}, {times.max():.4f}]s"
            )


def main(args=None):
    rclpy.init(args=args)
    node = LidarDeskewShim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()