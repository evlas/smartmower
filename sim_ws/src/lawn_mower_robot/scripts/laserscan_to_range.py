#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range


class LaserScanToRange(Node):
    def __init__(self):
        super().__init__('laserscan_to_range')
        # Parameters
        self.declare_parameter('input_scan_topic', '/sonar/center/scan')
        self.declare_parameter('output_range_topic', '/sonar/center')
        self.declare_parameter('frame_id_override', '')
        self.declare_parameter('radiation_type', Range.ULTRASOUND)

        input_topic = self.get_parameter('input_scan_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_range_topic').get_parameter_value().string_value
        self.frame_id_override = self.get_parameter('frame_id_override').get_parameter_value().string_value
        self.radiation_type = self.get_parameter('radiation_type').get_parameter_value().integer_value

        self.sub = self.create_subscription(LaserScan, input_topic, self.scan_cb, 10)
        self.pub = self.create_publisher(Range, output_topic, 10)

        self.get_logger().info(f"LaserScan->Range bridge: '{input_topic}' -> '{output_topic}'")

    def scan_cb(self, msg: LaserScan):
        rng = Range()
        rng.header.stamp = msg.header.stamp
        rng.header.frame_id = self.frame_id_override if self.frame_id_override else msg.header.frame_id

        # Set metadata from LaserScan
        rng.radiation_type = self.radiation_type
        # Field of view: use scan span if available, fallback to a narrow beam
        fov = float(msg.angle_max - msg.angle_min)
        if not math.isfinite(fov) or fov <= 0.0:
            fov = 0.001
        rng.field_of_view = fov
        rng.min_range = msg.range_min
        rng.max_range = msg.range_max

        # Choose a single range value: take the minimum finite value
        rng_value = self._min_finite(msg.ranges)
        if rng_value is None:
            # If nothing valid, set to +inf to indicate no return
            rng_value = float('inf')
        # Clamp to valid bounds
        rng.range = max(rng.min_range, min(rng_value, rng.max_range))

        self.pub.publish(rng)

    @staticmethod
    def _min_finite(values) -> Optional[float]:
        m = None
        for v in values:
            if v is None:
                continue
            if not math.isfinite(v):
                continue
            if m is None or v < m:
                m = v
        return m


def main():
    rclpy.init()
    node = LaserScanToRange()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
