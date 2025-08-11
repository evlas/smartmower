#!/usr/bin/env python3
"""
Parameter Manager for Smart Mower Robot
Handles loading, saving, validation and MQTT management of robot parameters
"""

import json
import os
import logging
from datetime import datetime
from typing import Dict, Any, Optional, Callable
from dataclasses import dataclass
import paho.mqtt.client as mqtt
import threading
import time

@dataclass
class BatteryConfig:
    """Battery configuration with voltage calculations"""
    type: str  # LiPo, LiFePO4, Li-ion
    cell_count: int  # 3S, 4S, 5S, 6S, 7S, 8S
    capacity: int  # mAh
    c_rating: int
    
    # Voltage per cell by battery type
    VOLTAGE_SPECS = {
        "LiPo": {"max": 4.2, "nominal": 3.7, "min": 3.0, "storage": 3.8},
        "LiFePO4": {"max": 3.6, "nominal": 3.2, "min": 2.5, "storage": 3.3},
        "Li-ion": {"max": 4.1, "nominal": 3.6, "min": 2.8, "storage": 3.7}
    }
    
    def calculate_pack_voltages(self) -> Dict[str, float]:
        """Calculate pack voltages based on cell type and count"""
        if self.type not in self.VOLTAGE_SPECS:
            raise ValueError(f"Unsupported battery type: {self.type}")
        
        specs = self.VOLTAGE_SPECS[self.type]
        return {
            "cell_max": specs["max"],
            "cell_nominal": specs["nominal"], 
            "cell_min": specs["min"],
            "cell_storage": specs["storage"],
            "pack_full": specs["max"] * self.cell_count,
            "pack_nominal": specs["nominal"] * self.cell_count,
            "pack_low": specs["min"] * self.cell_count + 0.2,  # Safety margin
            "pack_critical": specs["min"] * self.cell_count
        }

class ParameterValidator:
    """Validates parameter values and ranges"""
    
    @staticmethod
    def validate_range(value: float, min_val: float, max_val: float, name: str) -> bool:
        """Validate value is within range"""
        if not (min_val <= value <= max_val):
            logging.error(f"Parameter {name} value {value} out of range [{min_val}, {max_val}]")
            return False
        return True
    
    @staticmethod
    def validate_pid_params(params: Dict[str, float]) -> bool:
        """Validate PID parameters"""
        required = ["kp", "ki", "kd"]
        for param in required:
            if param not in params:
                logging.error(f"Missing PID parameter: {param}")
                return False
            if not ParameterValidator.validate_range(params[param], 0.0, 100.0, f"PID {param}"):
                return False
        return True
    
    @staticmethod
    def validate_battery_config(config: Dict[str, Any]) -> bool:
        """Validate battery configuration"""
        required_fields = ["type", "cell_count", "capacity"]
        for field in required_fields:
            if field not in config:
                logging.error(f"Missing battery field: {field}")
                return False
        
        # Validate battery type
        if config["type"] not in BatteryConfig.VOLTAGE_SPECS:
            logging.error(f"Invalid battery type: {config['type']}")
            return False
        
        # Validate cell count
        if not (3 <= config["cell_count"] <= 8):
            logging.error(f"Invalid cell count: {config['cell_count']}")
            return False
        
        return True

class ParameterManager:
    """Main parameter management class"""
    
    def __init__(self, config_file: str = "robot_config.json", mqtt_broker: str = "localhost", mqtt_port: int = 1883):
        self.config_file = config_file
        self.config_data = {}
        self.mqtt_client = None
        self.mqtt_broker = mqtt_broker
        self.mqtt_port = mqtt_port
        self.device_id = "smartmower_001"
        self.parameter_callbacks = {}
        self.lock = threading.Lock()
        
        # Setup logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        # Load configuration
        self.load_config()
        
        # Setup MQTT
        self.setup_mqtt()
    
    def load_config(self) -> bool:
        """Load configuration from file"""
        try:
            if os.path.exists(self.config_file):
                with open(self.config_file, 'r') as f:
                    self.config_data = json.load(f)
                self.logger.info(f"Configuration loaded from {self.config_file}")
                
                # Update battery voltage calculations
                self._update_battery_voltages()
                return True
            else:
                self.logger.error(f"Configuration file {self.config_file} not found")
                return False
        except Exception as e:
            self.logger.error(f"Error loading configuration: {e}")
            return False
    
    def save_config(self) -> bool:
        """Save configuration to file"""
        try:
            with self.lock:
                # Update metadata
                if "metadata" not in self.config_data:
                    self.config_data["metadata"] = {}
                self.config_data["metadata"]["last_modified"] = datetime.now().isoformat()
                
                # Create backup
                if os.path.exists(self.config_file):
                    backup_file = f"{self.config_file}.backup"
                    os.rename(self.config_file, backup_file)
                
                # Save new config
                with open(self.config_file, 'w') as f:
                    json.dump(self.config_data, f, indent=2)
                
                self.logger.info(f"Configuration saved to {self.config_file}")
                return True
        except Exception as e:
            self.logger.error(f"Error saving configuration: {e}")
            return False
    
    def _update_battery_voltages(self):
        """Update battery voltage thresholds based on type and cell count"""
        if "battery" in self.config_data:
            battery_config = self.config_data["battery"]
            if ParameterValidator.validate_battery_config(battery_config):
                battery = BatteryConfig(
                    type=battery_config["type"],
                    cell_count=battery_config["cell_count"],
                    capacity=battery_config.get("capacity", 5000),
                    c_rating=battery_config.get("c_rating", 25)
                )
                
                # Update voltage thresholds
                voltages = battery.calculate_pack_voltages()
                self.config_data["battery"]["voltage_thresholds"] = voltages
                
                # Update charging parameters
                if "charging" not in self.config_data["battery"]:
                    self.config_data["battery"]["charging"] = {}
                
                self.config_data["battery"]["charging"]["voltage_max"] = voltages["pack_full"]
                
                self.logger.info(f"Updated battery voltages for {battery.type} {battery.cell_count}S")
    
    def setup_mqtt(self):
        """Setup MQTT client and subscriptions"""
        try:
            self.mqtt_client = mqtt.Client()
            self.mqtt_client.on_connect = self._on_mqtt_connect
            self.mqtt_client.on_message = self._on_mqtt_message
            
            # Get device ID and MQTT config from config
            if "system" in self.config_data and "communication" in self.config_data["system"]:
                comm_config = self.config_data["system"]["communication"]
                self.device_id = comm_config.get("device_id", self.device_id)
                self.mqtt_broker = comm_config.get("mqtt_broker_host", self.mqtt_broker)
                self.mqtt_port = comm_config.get("mqtt_broker_port", self.mqtt_port)
                
                # Set MQTT authentication if credentials are provided
                mqtt_username = comm_config.get("mqtt_username")
                mqtt_password = comm_config.get("mqtt_password")
                if mqtt_username:
                    self.mqtt_client.username_pw_set(mqtt_username, mqtt_password)
                    self.logger.info(f"MQTT authentication set for user: {mqtt_username}")
            
            # Connect to MQTT broker
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            
            self.logger.info(f"MQTT client connected to {self.mqtt_broker}:{self.mqtt_port}")
            
        except Exception as e:
            self.logger.error(f"Error setting up MQTT: {e}")
    
    def _on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT connection callback"""
        if rc == 0:
            self.logger.info("Connected to MQTT broker")
            # Subscribe to parameter topics
            topics = [
                f"{self.device_id}/config/+/+/+",
                f"{self.device_id}/config/+/+",
                f"{self.device_id}/commands/+",
            ]
            for topic in topics:
                client.subscribe(topic)
                self.logger.info(f"Subscribed to {topic}")
            
            # Publish initial status and configuration data
            self.publish_status()
            self.publish_config_data()
        else:
            self.logger.error(f"Failed to connect to MQTT broker: {rc}")
    
    def _on_mqtt_message(self, client, userdata, msg):
        """MQTT message callback"""
        try:
            topic_parts = msg.topic.split('/')
            if len(topic_parts) < 3:
                return
            
            device_id = topic_parts[0]
            if device_id != self.device_id:
                return
            
            message_type = topic_parts[1]  # config or commands
            
            if message_type == "config":
                self._handle_config_message(topic_parts[2:], msg.payload.decode())
            elif message_type == "commands":
                self._handle_command_message(topic_parts[2], msg.payload.decode())
                
        except Exception as e:
            self.logger.error(f"Error processing MQTT message: {e}")
    
    def _handle_config_message(self, path_parts, value_str):
        """Handle configuration parameter updates"""
        try:
            # Convert value to appropriate type
            value = self._parse_value(value_str)
            
            # Update configuration
            with self.lock:
                current = self.config_data
                for part in path_parts[:-1]:
                    if part not in current:
                        current[part] = {}
                    current = current[part]
                
                old_value = current.get(path_parts[-1])
                current[path_parts[-1]] = value
                
                # Validate the change
                if self._validate_parameter_change(path_parts, value):
                    # Special handling for battery parameters
                    if path_parts[0] == "battery":
                        self._update_battery_voltages()
                    
                    # Save configuration
                    self.save_config()
                    
                    # Notify callbacks
                    param_path = "/".join(path_parts)
                    if param_path in self.parameter_callbacks:
                        self.parameter_callbacks[param_path](value, old_value)
                    
                    # Publish confirmation
                    confirm_topic = f"{self.device_id}/config/status/{'/'.join(path_parts)}"
                    self.mqtt_client.publish(confirm_topic, json.dumps({
                        "status": "updated",
                        "old_value": old_value,
                        "new_value": value,
                        "timestamp": datetime.now().isoformat()
                    }))
                    
                    self.logger.info(f"Parameter {param_path} updated: {old_value} -> {value}")
                else:
                    # Revert change
                    current[path_parts[-1]] = old_value
                    self.logger.error(f"Invalid parameter change rejected: {'/'.join(path_parts)} = {value}")
                    
        except Exception as e:
            self.logger.error(f"Error handling config message: {e}")
    
    def _handle_command_message(self, command, payload):
        """Handle command messages"""
        try:
            if command == "save_config":
                self.save_config()
                self.mqtt_client.publish(f"{self.device_id}/commands/status/save_config", "completed")
                
            elif command == "load_defaults":
                self._load_default_config()
                self.mqtt_client.publish(f"{self.device_id}/commands/status/load_defaults", "completed")
                
            elif command == "get_config":
                config_topic = f"{self.device_id}/config/dump"
                self.mqtt_client.publish(config_topic, json.dumps(self.config_data, indent=2))
                
            elif command == "validate_config":
                is_valid = self._validate_full_config()
                self.mqtt_client.publish(f"{self.device_id}/commands/status/validate_config", 
                                       "valid" if is_valid else "invalid")
                
        except Exception as e:
            self.logger.error(f"Error handling command {command}: {e}")
    
    def _parse_value(self, value_str: str):
        """Parse string value to appropriate type"""
        # Try to parse as JSON first (handles numbers, booleans, strings, arrays, objects)
        try:
            return json.loads(value_str)
        except:
            # Return as string if JSON parsing fails
            return value_str
    
    def _validate_parameter_change(self, path_parts, value) -> bool:
        """Validate a parameter change"""
        param_path = "/".join(path_parts)
        
        # PID validation
        if "pid" in path_parts:
            if isinstance(value, (int, float)):
                return ParameterValidator.validate_range(value, 0.0, 100.0, param_path)
        
        # Speed validation
        elif "speeds" in path_parts:
            if isinstance(value, (int, float)):
                return ParameterValidator.validate_range(value, 0.0, 10.0, param_path)
        
        # Battery validation
        elif path_parts[0] == "battery":
            if path_parts[1] == "type":
                return value in BatteryConfig.VOLTAGE_SPECS
            elif path_parts[1] == "cell_count":
                return 3 <= value <= 8
            elif path_parts[1] == "capacity":
                return 1000 <= value <= 20000
        
        # Default: accept the change
        return True
    
    def _validate_full_config(self) -> bool:
        """Validate entire configuration"""
        try:
            # Validate battery config
            if "battery" in self.config_data:
                if not ParameterValidator.validate_battery_config(self.config_data["battery"]):
                    return False
            
            # Validate PID configs
            if "tuning" in self.config_data and "pid" in self.config_data["tuning"]:
                pid_config = self.config_data["tuning"]["pid"]
                linear_pid = {k.replace("linear_", ""): v for k, v in pid_config.items() if k.startswith("linear_")}
                angular_pid = {k.replace("angular_", ""): v for k, v in pid_config.items() if k.startswith("angular_")}
                
                if not ParameterValidator.validate_pid_params(linear_pid):
                    return False
                if not ParameterValidator.validate_pid_params(angular_pid):
                    return False
            
            return True
            
        except Exception as e:
            self.logger.error(f"Error validating config: {e}")
            return False
    
    def _load_default_config(self):
        """Load default configuration"""
        # This would load from a default config file or reset to hardcoded defaults
        self.logger.info("Loading default configuration")
        # Implementation depends on your default config strategy
    
    def get_parameter(self, path: str, default=None):
        """Get parameter value by path (e.g., 'tuning/pid/linear_kp')"""
        try:
            parts = path.split('/')
            current = self.config_data
            for part in parts:
                current = current[part]
            return current
        except (KeyError, TypeError):
            return default
    
    def set_parameter(self, path: str, value, publish_mqtt: bool = True):
        """Set parameter value by path"""
        try:
            parts = path.split('/')
            with self.lock:
                current = self.config_data
                for part in parts[:-1]:
                    if part not in current:
                        current[part] = {}
                    current = current[part]
                
                old_value = current.get(parts[-1])
                current[parts[-1]] = value
                
                # Update battery voltages if needed
                if parts[0] == "battery":
                    self._update_battery_voltages()
                
                # Publish parameter change notification
                if publish_mqtt and self.mqtt_client:
                    self.publish_parameter_change(path, value, old_value)
                
                # Call registered callbacks
                if path in self.parameter_callbacks:
                    try:
                        self.parameter_callbacks[path](value, old_value)
                    except Exception as e:
                        self.logger.error(f"Error calling callback for {path}: {e}")
                
                self.logger.info(f"Parameter {path} set: {old_value} -> {value}")
                return True
                
        except Exception as e:
            self.logger.error(f"Error setting parameter {path}: {e}")
            return False
    
    def register_callback(self, parameter_path: str, callback: Callable):
        """Register callback for parameter changes"""
        self.parameter_callbacks[parameter_path] = callback
        self.logger.info(f"Registered callback for {parameter_path}")
    
    def get_battery_info(self) -> Dict[str, Any]:
        """Get complete battery information"""
        if "battery" not in self.config_data:
            return {}
        
        battery_config = self.config_data["battery"]
        return {
            "type": battery_config.get("type"),
            "cell_count": battery_config.get("cell_count"),
            "capacity": battery_config.get("capacity"),
            "voltages": battery_config.get("voltage_thresholds", {}),
            "charging": battery_config.get("charging", {}),
            "protections": battery_config.get("protections", {})
        }
    
    def publish_status(self):
        """Publish service status to MQTT"""
        if not self.mqtt_client:
            return
        
        try:
            status_data = {
                "type": "config_status",
                "timestamp": int(time.time()),
                "system": {
                    "running": True,
                    "config_file": self.config_file,
                    "device_id": self.device_id,
                    "mqtt_broker": f"{self.mqtt_broker}:{self.mqtt_port}"
                },
                "parameters": {
                    "total_callbacks": len(self.parameter_callbacks),
                    "monitored_params": list(self.parameter_callbacks.keys())
                }
            }
            
            topic = "smartmower/config/status"
            self.mqtt_client.publish(topic, json.dumps(status_data), qos=1, retain=True)
            self.logger.info(f"Published status to {topic}")
            
        except Exception as e:
            self.logger.error(f"Error publishing status: {e}")
    
    def publish_config_data(self):
        """Publish current configuration data to MQTT"""
        if not self.mqtt_client:
            return
        
        try:
            # Publish essential configuration sections
            config_sections = {
                "tuning": self.config_data.get("tuning", {}),
                "battery": self.config_data.get("battery", {}),
                "hardware": self.config_data.get("hardware", {}),
                "system": {
                    "communication": self.config_data.get("system", {}).get("communication", {}),
                    "safety": self.config_data.get("system", {}).get("safety", {})
                }
            }
            
            data_payload = {
                "type": "config_data",
                "timestamp": int(time.time()),
                "device_id": self.device_id,
                "config": config_sections
            }
            
            topic = "smartmower/config/data"
            self.mqtt_client.publish(topic, json.dumps(data_payload), qos=1, retain=True)
            self.logger.info(f"Published config data to {topic}")
            
        except Exception as e:
            self.logger.error(f"Error publishing config data: {e}")
    
    def publish_parameter_change(self, parameter_path: str, new_value, old_value):
        """Publish parameter change notification to MQTT"""
        if not self.mqtt_client:
            return
        
        try:
            change_data = {
                "type": "parameter_change",
                "timestamp": int(time.time()),
                "parameter": parameter_path,
                "old_value": old_value,
                "new_value": new_value,
                "device_id": self.device_id
            }
            
            topic = f"smartmower/config/changes/{parameter_path.replace('/', '_')}"
            self.mqtt_client.publish(topic, json.dumps(change_data), qos=1)
            self.logger.info(f"Published parameter change to {topic}")
            
        except Exception as e:
            self.logger.error(f"Error publishing parameter change: {e}")
    
    def shutdown(self):
        """Shutdown parameter manager"""
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        self.logger.info("Parameter manager shutdown")

# Example usage and testing
if __name__ == "__main__":
    # Create parameter manager
    pm = ParameterManager("robot_config.json")
    
    # Example: Register callback for PID changes
    def pid_changed(new_value, old_value):
        print(f"PID parameter changed: {old_value} -> {new_value}")
    
    pm.register_callback("tuning/pid/linear_kp", pid_changed)
    
    # Example: Get/Set parameters
    print("Current linear Kp:", pm.get_parameter("tuning/pid/linear_kp"))
    pm.set_parameter("tuning/pid/linear_kp", 1.5)
    
    # Example: Get battery info
    battery_info = pm.get_battery_info()
    print("Battery info:", json.dumps(battery_info, indent=2))
    
    # Keep running to handle MQTT messages
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pm.shutdown()
