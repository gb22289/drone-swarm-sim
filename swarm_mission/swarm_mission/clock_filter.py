#!/usr/bin/env python3
"""
Clock monotonicity filter.

Subscribes to /clock, ensures timestamps are strictly monotonically
increasing, and republishes on /clock_filtered. Drops any message
where time moves backwards.

Use by remapping: --ros-args -r /clock:=/clock_raw
Then this node reads /clock_raw and publishes /clock.

Or simpler: run Gazebo publishing to /clock_raw, and this node
bridges to /clock.

Usage:
  ros2 run swarm_mission clock_filter
"""
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


class ClockFilter(Node):
    def __init__(self):
        super().__init__('clock_filter')
        self.last_sec = 0
        self.last_nsec = 0
        self.dropped = 0
        self.forwarded = 0

        self.declare_parameter('input_topic', '/clock_raw')
        self.declare_parameter('output_topic', '/clock')
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        # Use a large queue to avoid dropping valid messages at 760Hz
        self.sub = self.create_subscription(
            Clock, input_topic, self.callback, 100)
        self.pub = self.create_publisher(Clock, output_topic, 100)

        self.get_logger().info(
            f'Clock filter started: {input_topic} → {output_topic} (monotonic)')

    def callback(self, msg):
        sec = msg.clock.sec
        nsec = msg.clock.nanosec

        # Check monotonicity
        if sec > self.last_sec or (sec == self.last_sec and nsec > self.last_nsec):
            self.pub.publish(msg)
            self.last_sec = sec
            self.last_nsec = nsec
            self.forwarded += 1
        else:
            self.dropped += 1
            if self.dropped % 100 == 1:
                self.get_logger().warn(
                    f'Dropped {self.dropped} backwards clock messages '
                    f'(forwarded {self.forwarded})')


def main(args=None):
    rclpy.init(args=args)
    node = ClockFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            f'Clock filter: forwarded {node.forwarded}, dropped {node.dropped}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
