#!/usr/bin/env python3

import rclcpp
from rclcpp.node import Node
from geometry_msgs.msg import Twist

class StopVelocityPublisher(Node):
    """
    Nodo che pubblica continuamente velocità zero sul topic /mower/cmd_vel/stop
    per garantire che quando questo input è selezionato da twist_mux,
    il robot riceva effettivamente velocità zero invece di nessun messaggio
    """

    def __init__(self):
        super().__init__('stop_velocity_publisher')

        # Publisher per velocità zero
        self.stop_pub = self.create_publisher(
            Twist,
            '/mower/cmd_vel/stop',
            10
        )

        # Timer per pubblicazione continua (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_stop_velocity)

        # Crea messaggio velocità zero
        self.stop_twist = Twist()
        # Velocità lineare e angolare già zero per default

        self.get_logger().info('StopVelocityPublisher initialized - publishing zero velocity on /mower/cmd_vel/stop')

    def publish_stop_velocity(self):
        """Pubblica velocità zero"""
        self.stop_pub.publish(self.stop_twist)

def main(args=None):
    rclcpp.init(args=args)
    node = StopVelocityPublisher()
    rclcpp.spin(node)
    rclcpp.shutdown()

if __name__ == '__main__':
    main()
