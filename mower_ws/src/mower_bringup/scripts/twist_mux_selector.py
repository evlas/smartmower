#!/usr/bin/env python3

import rclcpp
from rclcpp.node import Node
from std_msgs.msg import String, Int32
import threading

class TwistMuxSelector(Node):
    """
    Nodo che seleziona quale input twist_mux utilizzare basato sullo stato della macchina a stati
    """

    def __init__(self):
        super().__init__('twist_mux_selector')

        # Mapping stati -> priorità twist_mux
        self.state_priorities = {
            'IDLE': 0,      # Gestito da stop_cmd_vel
            'UNDOCKING': 20,
            'MOWING': 30,
            'DOCKING': 40,
            'CHARGING': 0,  # Gestito da stop_cmd_vel
            'EMERGENCY_STOP': 0,  # Gestito da stop_cmd_vel
            'MANUAL_CONTROL': 100,   # Alta priorità per controllo umano
            'ERROR': 0,     # Gestito da stop_cmd_vel
            'PAUSED': 0     # Gestito da stop_cmd_vel
        }

        # Subscriber allo stato della macchina a stati
        self.state_sub = self.create_subscription(
            String,
            '/mower/state',
            self.state_callback,
            10
        )

        # Publisher per selezione twist_mux
        self.select_pub = self.create_publisher(
            String,
            '/twist_mux/select',
            10
        )

        # Publisher per priorità twist_mux
        self.priority_pub = self.create_publisher(
            Int32,
            '/twist_mux/priority',
            10
        )

        self.current_state = 'IDLE'
        self.get_logger().info('TwistMuxSelector initialized')

    def state_callback(self, msg):
        """Callback per cambio stato della macchina a stati"""
        new_state = msg.data.strip()

        if new_state != self.current_state:
            self.current_state = new_state
            self.update_mux_selection()

    def update_mux_selection(self):
        """Aggiorna la selezione twist_mux basata sullo stato corrente"""
        # Stati in cui il robot deve rimanere completamente fermo
        stop_states = {'IDLE', 'EMERGENCY_STOP', 'CHARGING', 'ERROR', 'PAUSED'}

        if self.current_state in stop_states:
            # Per stati di stop, pubblica selezione per input STOP
            select_msg = String()
            select_msg.data = "stop_cmd_vel"
            self.select_pub.publish(select_msg)

            # Crea messaggio di priorità (0 per stato di stop)
            priority_msg = Int32()
            priority_msg.data = 0
            self.priority_pub.publish(priority_msg)

            self.get_logger().info(f'State {self.current_state}: Robot stopped - selecting stop_cmd_vel')
            return

        priority = self.state_priorities.get(self.current_state, 0)

        # Crea messaggio di selezione (nome dell'input twist_mux)
        select_msg = String()
        select_msg.data = f"{self.current_state.lower()}_cmd_vel"
        self.select_pub.publish(select_msg)

        # Crea messaggio di priorità
        priority_msg = Int32()
        priority_msg.data = priority
        self.priority_pub.publish(priority_msg)

        self.get_logger().info(f'State changed to {self.current_state}, '
                              f'selecting {select_msg.data} with priority {priority}')

def main(args=None):
    rclcpp.init(args=args)
    node = TwistMuxSelector()
    rclcpp.spin(node)
    rclcpp.shutdown()

if __name__ == '__main__':
    main()
