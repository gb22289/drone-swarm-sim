#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class LioMavrosBridge(Node):
    def __init__(self):
        super().__init__('lio_mavros_bridge')
        self.declare_parameter('drone_ns', 'drone1')
        drone_ns = self.get_parameter('drone_ns').get_parameter_value().string_value

        self.last_stamp_ns = 0
        self.latest_pose = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        odom_topic = f'/{drone_ns}/lio_sam/mapping/odometry_incremental'
        pose_topic = f'/{drone_ns}/mavros/vision_pose/pose'

        self.sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, qos)
        self.pub = self.create_publisher(
            PoseStamped, pose_topic, 10)

        # Republish at 20 Hz so ArduCopter always sees fresh vision data
        self.timer = self.create_timer(0.05, self.republish_pose)

        self.get_logger().info(f'Bridge started: {odom_topic} -> {pose_topic} (20 Hz republish)')

    def odom_callback(self, msg):
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if stamp_ns <= self.last_stamp_ns:
            return
        self.last_stamp_ns = stamp_ns
        self.latest_pose = msg.pose.pose

    def republish_pose(self):
        if self.latest_pose is None:
            return
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose = self.latest_pose
        self.pub.publish(pose_msg)


def main():
    rclpy.init()
    node = LioMavrosBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
