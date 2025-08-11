#!/usr/bin/env python3
"""
Script to add comprehensive field descriptions to robot_config.json
"""

import json
import re

def add_field_comments(config_file):
    """Add inline comments to all fields in the configuration file"""
    
    # Field descriptions mapping
    descriptions = {
        # Metadata
        "config_version": "Configuration file format version",
        "robot_id": "Unique identifier for this robot instance", 
        "last_modified": "ISO timestamp of last configuration update",
        "description": "Human-readable description of this config file",
        
        # Battery system
        "type": "Battery chemistry type",
        "cells": "Number of cells in series configuration",
        "nominal_voltage_per_cell": "Nominal voltage per cell in volts",
        "max_voltage_per_cell": "Maximum safe voltage per cell in volts",
        "min_voltage_per_cell": "Minimum safe voltage per cell in volts",
        "capacity_ah": "Battery capacity in amp-hours",
        "nominal_pack_voltage": "Total pack voltage at nominal charge",
        "max_pack_voltage": "Total pack voltage when fully charged",
        "min_pack_voltage": "Total pack voltage at minimum safe level",
        "full_voltage_per_cell": "Voltage per cell considered fully charged",
        "trickle_current_ma": "Trickle charge current in milliamps",
        "stable_time_seconds": "Time to wait at trickle current before considering charged",
        "current_threshold_ma": "Current threshold below which charging is considered complete",
        "voltage_curve": "Voltage per cell at different charge percentages",
        
        # Hardware
        "length_m": "Robot length in meters",
        "width_m": "Robot width in meters", 
        "height_m": "Robot height in meters",
        "weight_kg": "Robot weight in kilograms",
        "ground_clearance_m": "Ground clearance in meters",
        "wheel_diameter_m": "Drive wheel diameter in meters",
        "wheel_base_m": "Distance between front and rear axles",
        "track_width_m": "Distance between left and right wheels",
        "blade_diameter_m": "Cutting blade diameter in meters",
        "deck_width_m": "Physical cutting deck width in meters",
        "blade_offset_x_m": "Blade offset from robot center (X-axis)",
        "blade_offset_y_m": "Blade offset from robot center (Y-axis)",
        "max_cutting_height_mm": "Maximum cutting height in millimeters",
        "min_cutting_height_mm": "Minimum cutting height in millimeters",
        
        # Tuning
        "kp": "Proportional gain",
        "ki": "Integral gain", 
        "kd": "Derivative gain",
        "max_linear_speed_mps": "Maximum linear speed in meters per second",
        "max_angular_speed_rps": "Maximum angular speed in radians per second",
        "cutting_speed_mps": "Speed while cutting grass in m/s",
        "pattern": "Currently selected cutting pattern",
        "available_patterns": "List of available cutting patterns",
        "cutting_width_m": "Effective cutting width in meters",
        "cutting_height_mm": "Grass cutting height in millimeters",
        "blade_speed_rpm": "Blade rotation speed in RPM",
        
        # System
        "mqtt_broker_host": "MQTT broker hostname or IP address",
        "mqtt_broker_port": "MQTT broker port number",
        "device_id": "Unique device identifier for MQTT",
        "base_topic": "Base MQTT topic for messages",
        "qos": "MQTT Quality of Service level",
        "retain": "Whether to retain MQTT messages",
        "level": "Logging level (debug, info, warn, error)",
        "file": "Log file path",
        "data_dir": "Data directory path",
        
        # Module specific
        "uart_device": "UART device path for communication",
        "baudrate": "Communication baud rate",
        "battery_profile": "Reference to battery profile in battery_system.profiles",
        "device": "Device path",
        "protocol": "Communication protocol",
        "update_rate_hz": "Update rate in Hz"
    }
    
    with open(config_file, 'r') as f:
        content = f.read()
    
    # Add comments to fields
    for field, desc in descriptions.items():
        # Pattern to match field definitions
        pattern = f'"{field}": ([^,}}]+)'
        replacement = f'"{field}": \\1, "//": "{desc}"'
        
        # Only add comment if not already present
        if f'"//": "{desc}"' not in content:
            content = re.sub(pattern, replacement, content)
    
    # Clean up any double commas
    content = re.sub(r',\s*,', ',', content)
    
    with open(config_file, 'w') as f:
        f.write(content)
    
    print(f"Added field descriptions to {config_file}")

if __name__ == "__main__":
    add_field_comments("/home/vito/smartmower/src/config/robot_config.json")
