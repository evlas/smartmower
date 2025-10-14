#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus

class PoseToNavSat(Node):
    def __init__(self):
        super().__init__('pose_to_navsat')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('navsat_topic', 'gps/fix')
        self.declare_parameter('origin_lat_deg', 41.66375)
        self.declare_parameter('origin_lon_deg', 13.2818889)
        self.declare_parameter('origin_elev_m', 0.0)

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.navsat_topic = self.get_parameter('navsat_topic').get_parameter_value().string_value
        self.lat0 = self.get_parameter('origin_lat_deg').get_parameter_value().double_value
        self.lon0 = self.get_parameter('origin_lon_deg').get_parameter_value().double_value
        self.h0 = self.get_parameter('origin_elev_m').get_parameter_value().double_value

        self.R_earth = 6378137.0  # meters
        self.lat0_rad = math.radians(self.lat0)

        qos = rclpy.qos.QoSProfile(depth=10)
        self.sub = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, qos)
        self.pub = self.create_publisher(NavSatFix, self.navsat_topic, qos)
        self.get_logger().info(f"pose_to_navsat: origin=({self.lat0}, {self.lon0}, {self.h0}), odom='{self.odom_topic}', out='{self.navsat_topic}'")

    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x  # assume x ~ East (m)
        y = msg.pose.pose.position.y  # assume y ~ North (m)
        z = msg.pose.pose.position.z

        # Convert local EN (x,y) to lat/lon (approx, small area)
        dlat = (y / self.R_earth) * (180.0 / math.pi)
        dlon = (x / (self.R_earth * math.cos(self.lat0_rad))) * (180.0 / math.pi)

        lat = self.lat0 + dlat
        lon = self.lon0 + dlon
        alt = self.h0 + z

        out = NavSatFix()
        out.header = msg.header
        out.header.frame_id = 'gps_frame'  # publish as GPS frame
        out.status.status = int(NavSatStatus.STATUS_FIX)
        out.status.service = int(NavSatStatus.SERVICE_GPS)
        out.latitude = lat
        out.longitude = lon
        out.altitude = alt
        out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.pub.publish(out)


def main():
    rclpy.init()
    node = PoseToNavSat()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
