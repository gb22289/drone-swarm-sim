#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class LioMavrosBridge(Node):
    def __init__(self):
        super().__init__('lio_mavros_bridge')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        self.sub = self.create_subscription(
            Odometry,
            '/lio_sam/mapping/odometry_incremental',
            self.odom_callback,
            qos)
        self.pub = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            10)
        self.get_logger().info('LIO-SAM → MAVROS bridge started')

    def odom_callback(self, msg):
        self.get_logger().info('Got odometry!')
        pose_msg = PoseStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = 'map'
        pose_msg.pose = msg.pose.pose
        self.pub.publish(pose_msg)

def main():
    rclpy.init()
    node = LioMavrosBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
