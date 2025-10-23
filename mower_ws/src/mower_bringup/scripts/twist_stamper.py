#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistStamper(Node):
    def __init__(self):
        super().__init__('twist_stamper')
        self.in_topic = self.declare_parameter('in_topic', '/cmd_vel').get_parameter_value().string_value
        self.out_topic = self.declare_parameter('out_topic', '/cmd_vel').get_parameter_value().string_value
        self.frame_id = self.declare_parameter('frame_id', 'base_link').get_parameter_value().string_value
        self.sub = self.create_subscription(Twist, self.in_topic, self.cb, 10)
        self.pub = self.create_publisher(TwistStamped, self.out_topic, 10)
        self.get_logger().info(f"TwistStamper: {self.in_topic} -> {self.out_topic} (frame_id={self.frame_id})")

    def cb(self, msg: Twist):
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = self.frame_id
        ts.twist = msg
        self.pub.publish(ts)


def main():
    rclpy.init()
    node = TwistStamper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
