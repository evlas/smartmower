#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class StopVelocityPublisher(Node):
    """
    Nodo che pubblica continuamente velocità zero sul topic /mower/cmd_vel/stop
    per garantire che quando questo input è selezionato da twist_mux,
    il robot riceva effettivamente velocità zero invece di nessun messaggio
    """

    def __init__(self):
        super().__init__('stop_velocity_publisher')

        # Publisher per velocità zero
        self.stop_pub = self.create_publisher(TwistStamped, '/mower/cmd_vel/stop', 10)

        # Timer per pubblicazione continua (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_stop_velocity)

        # Crea messaggio TwistStamped con velocità zero
        self.frame_id = 'base_link'

        self.get_logger().info('StopVelocityPublisher initialized - publishing zero velocity on /mower/cmd_vel/stop')

    def publish_stop_velocity(self):
        """Pubblica velocità zero"""
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = self.frame_id
        # twist zero di default
        self.stop_pub.publish(ts)

def main(args=None):
    rclpy.init(args=args)
    node = StopVelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
