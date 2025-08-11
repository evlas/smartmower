#!/usr/bin/env python3
"""
MQTT Parameter Client for Smart Mower Robot
Command-line tool to interact with robot parameters via MQTT
"""

import json
import argparse
import paho.mqtt.client as mqtt
import time
import sys
from typing import Dict, Any

class MQTTParameterClient:
    """Client for interacting with robot parameters via MQTT"""
    
    def __init__(self, broker_host: str = "localhost", broker_port: int = 1883, device_id: str = "smartmower_001"):
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.device_id = device_id
        self.client = None
        self.response_received = False
        self.response_data = None
        
    def connect(self):
        """Connect to MQTT broker"""
        try:
            self.client = mqtt.Client()
            self.client.on_connect = self._on_connect
            self.client.on_message = self._on_message
            
            self.client.connect(self.broker_host, self.broker_port, 60)
            self.client.loop_start()
            
            # Wait for connection
            time.sleep(1)
            print(f"Connected to MQTT broker at {self.broker_host}:{self.broker_port}")
            return True
            
        except Exception as e:
            print(f"Error connecting to MQTT broker: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from MQTT broker"""
        if self.client:
            self.client.loop_stop()
            self.client.disconnect()
    
    def _on_connect(self, client, userdata, flags, rc):
        """MQTT connection callback"""
        if rc == 0:
            # Subscribe to status topics
            client.subscribe(f"{self.device_id}/config/status/+/+/+")
            client.subscribe(f"{self.device_id}/config/status/+/+")
            client.subscribe(f"{self.device_id}/commands/status/+")
            client.subscribe(f"{self.device_id}/config/dump")
        else:
            print(f"Failed to connect: {rc}")
    
    def _on_message(self, client, userdata, msg):
        """MQTT message callback"""
        try:
            topic_parts = msg.topic.split('/')
            
            if "status" in topic_parts:
                print(f"Status update: {msg.topic}")
                try:
                    data = json.loads(msg.payload.decode())
                    print(json.dumps(data, indent=2))
                except:
                    print(f"Response: {msg.payload.decode()}")
                    
            elif msg.topic.endswith("/config/dump"):
                print("Current configuration:")
                try:
                    data = json.loads(msg.payload.decode())
                    print(json.dumps(data, indent=2))
                except:
                    print(msg.payload.decode())
                    
                self.response_received = True
                self.response_data = msg.payload.decode()
                
        except Exception as e:
            print(f"Error processing message: {e}")
    
    def set_parameter(self, parameter_path: str, value: Any):
        """Set a parameter value"""
        if not self.client:
            print("Not connected to MQTT broker")
            return False
        
        try:
            topic = f"{self.device_id}/config/{parameter_path}"
            payload = json.dumps(value)
            
            print(f"Setting {parameter_path} = {value}")
            self.client.publish(topic, payload)
            
            # Wait for confirmation
            time.sleep(2)
            return True
            
        except Exception as e:
            print(f"Error setting parameter: {e}")
            return False
    
    def get_config(self):
        """Get current configuration"""
        if not self.client:
            print("Not connected to MQTT broker")
            return None
        
        try:
            self.response_received = False
            self.client.publish(f"{self.device_id}/commands/get_config", "")
            
            # Wait for response
            timeout = 5
            start_time = time.time()
            while not self.response_received and (time.time() - start_time) < timeout:
                time.sleep(0.1)
            
            if self.response_received:
                return self.response_data
            else:
                print("Timeout waiting for configuration")
                return None
                
        except Exception as e:
            print(f"Error getting configuration: {e}")
            return None
    
    def save_config(self):
        """Save current configuration"""
        if not self.client:
            print("Not connected to MQTT broker")
            return False
        
        try:
            print("Saving configuration...")
            self.client.publish(f"{self.device_id}/commands/save_config", "")
            time.sleep(1)
            return True
            
        except Exception as e:
            print(f"Error saving configuration: {e}")
            return False
    
    def load_defaults(self):
        """Load default configuration"""
        if not self.client:
            print("Not connected to MQTT broker")
            return False
        
        try:
            print("Loading default configuration...")
            self.client.publish(f"{self.device_id}/commands/load_defaults", "")
            time.sleep(1)
            return True
            
        except Exception as e:
            print(f"Error loading defaults: {e}")
            return False
    
    def validate_config(self):
        """Validate current configuration"""
        if not self.client:
            print("Not connected to MQTT broker")
            return False
        
        try:
            print("Validating configuration...")
            self.client.publish(f"{self.device_id}/commands/validate_config", "")
            time.sleep(2)
            return True
            
        except Exception as e:
            print(f"Error validating configuration: {e}")
            return False

def main():
    """Main function with command-line interface"""
    parser = argparse.ArgumentParser(description="MQTT Parameter Client for Smart Mower")
    parser.add_argument("--broker", default="localhost", help="MQTT broker host")
    parser.add_argument("--port", type=int, default=1883, help="MQTT broker port")
    parser.add_argument("--device", default="smartmower_001", help="Device ID")
    
    subparsers = parser.add_subparsers(dest="command", help="Available commands")
    
    # Set parameter command
    set_parser = subparsers.add_parser("set", help="Set parameter value")
    set_parser.add_argument("parameter", help="Parameter path (e.g., tuning/pid/linear_kp)")
    set_parser.add_argument("value", help="Parameter value")
    set_parser.add_argument("--type", choices=["str", "int", "float", "bool", "json"], 
                           default="json", help="Value type")
    
    # Get config command
    subparsers.add_parser("get", help="Get current configuration")
    
    # Save config command
    subparsers.add_parser("save", help="Save current configuration")
    
    # Load defaults command
    subparsers.add_parser("defaults", help="Load default configuration")
    
    # Validate config command
    subparsers.add_parser("validate", help="Validate current configuration")
    
    # Interactive mode
    subparsers.add_parser("interactive", help="Interactive parameter tuning mode")
    
    args = parser.parse_args()
    
    if not args.command:
        parser.print_help()
        return
    
    # Create client and connect
    client = MQTTParameterClient(args.broker, args.port, args.device)
    if not client.connect():
        return
    
    try:
        if args.command == "set":
            # Parse value based on type
            if args.type == "str":
                value = args.value
            elif args.type == "int":
                value = int(args.value)
            elif args.type == "float":
                value = float(args.value)
            elif args.type == "bool":
                value = args.value.lower() in ("true", "1", "yes", "on")
            else:  # json
                try:
                    value = json.loads(args.value)
                except:
                    value = args.value
            
            client.set_parameter(args.parameter, value)
            
        elif args.command == "get":
            client.get_config()
            
        elif args.command == "save":
            client.save_config()
            
        elif args.command == "defaults":
            client.load_defaults()
            
        elif args.command == "validate":
            client.validate_config()
            
        elif args.command == "interactive":
            interactive_mode(client)
        
        # Wait a bit for responses
        time.sleep(2)
        
    finally:
        client.disconnect()

def interactive_mode(client):
    """Interactive parameter tuning mode"""
    print("\n=== Interactive Parameter Tuning Mode ===")
    print("Commands:")
    print("  set <path> <value>  - Set parameter")
    print("  get                 - Get current config")
    print("  save                - Save configuration")
    print("  validate            - Validate configuration")
    print("  battery             - Show battery info")
    print("  pid                 - Show PID parameters")
    print("  help                - Show this help")
    print("  quit                - Exit")
    print()
    
    while True:
        try:
            command = input("parameter> ").strip().split()
            if not command:
                continue
                
            if command[0] == "quit":
                break
                
            elif command[0] == "help":
                print("Available commands: set, get, save, validate, battery, pid, help, quit")
                
            elif command[0] == "set" and len(command) >= 3:
                path = command[1]
                value = " ".join(command[2:])
                try:
                    # Try to parse as JSON
                    parsed_value = json.loads(value)
                except:
                    parsed_value = value
                client.set_parameter(path, parsed_value)
                
            elif command[0] == "get":
                client.get_config()
                
            elif command[0] == "save":
                client.save_config()
                
            elif command[0] == "validate":
                client.validate_config()
                
            elif command[0] == "battery":
                print("Battery parameter examples:")
                print("  set battery/type \"LiPo\"")
                print("  set battery/cell_count 6")
                print("  set battery/capacity 5000")
                
            elif command[0] == "pid":
                print("PID parameter examples:")
                print("  set tuning/pid/linear_kp 1.5")
                print("  set tuning/pid/linear_ki 0.2")
                print("  set tuning/pid/angular_kp 2.0")
                
            else:
                print("Unknown command. Type 'help' for available commands.")
                
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error: {e}")
    
    print("Exiting interactive mode...")

if __name__ == "__main__":
    main()
