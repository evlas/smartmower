#!/usr/bin/env python3
"""
Configuration utilities for Smart Mower Vision System
Provides unified access to robot configuration for test scripts and utilities.
"""

import json
import os
from typing import Dict, Any, Optional

DEFAULT_CONFIG_PATH = "/opt/smartmower/etc/config/robot_config.json"

def load_robot_config(config_path: str = DEFAULT_CONFIG_PATH) -> Dict[str, Any]:
    """
    Load the unified robot configuration file.
    
    Args:
        config_path: Path to the robot_config.json file
        
    Returns:
        Dictionary containing the configuration data
        
    Raises:
        FileNotFoundError: If config file doesn't exist
        json.JSONDecodeError: If config file is invalid JSON
    """
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Configuration file not found: {config_path}")
    
    with open(config_path, 'r') as f:
        return json.load(f)

def get_mqtt_config(config_path: str = DEFAULT_CONFIG_PATH) -> Dict[str, Any]:
    """
    Get MQTT configuration from the unified config file.
    
    Args:
        config_path: Path to the robot_config.json file
        
    Returns:
        Dictionary containing MQTT configuration:
        {
            'broker_host': str,
            'broker_port': int,
            'username': str,
            'password': str,
            'device_id': str,
            'telemetry_interval': int
        }
        
    Raises:
        KeyError: If required MQTT configuration is missing
        FileNotFoundError: If config file doesn't exist
        json.JSONDecodeError: If config file is invalid JSON
    """
    config = load_robot_config(config_path)
    
    # Extract MQTT configuration from system.communication section
    try:
        system = config['system']
        communication = system['communication']
        
        mqtt_config = {
            'broker_host': communication['mqtt_broker_host'],
            'broker_port': communication['mqtt_broker_port'],
            'username': communication['mqtt_username'],
            'password': communication['mqtt_password'],
            'device_id': communication.get('device_id', 'smartmower_001'),
            'telemetry_interval': communication.get('telemetry_interval', 5)
        }
        
        return mqtt_config
        
    except KeyError as e:
        raise KeyError(f"Missing MQTT configuration in {config_path}: {e}")

def get_mqtt_credentials(config_path: str = DEFAULT_CONFIG_PATH) -> tuple[str, str, str, int]:
    """
    Get MQTT credentials from the unified config file.
    
    Args:
        config_path: Path to the robot_config.json file
        
    Returns:
        Tuple of (broker_host, username, password, port)
        
    Raises:
        KeyError: If required MQTT configuration is missing
        FileNotFoundError: If config file doesn't exist
        json.JSONDecodeError: If config file is invalid JSON
    """
    mqtt_config = get_mqtt_config(config_path)
    return (
        mqtt_config['broker_host'],
        mqtt_config['username'],
        mqtt_config['password'],
        mqtt_config['broker_port']
    )

# Fallback values for when config file is not available (development/testing)
FALLBACK_MQTT_CONFIG = {
    'broker_host': 'localhost',
    'broker_port': 1883,
    'username': 'mower',
    'password': 'smart',
    'device_id': 'smartmower_test',
    'telemetry_interval': 5
}

def get_mqtt_config_safe(config_path: str = DEFAULT_CONFIG_PATH) -> Dict[str, Any]:
    """
    Get MQTT configuration with fallback to default values.
    This is useful for test scripts that should work even without the config file.
    
    Args:
        config_path: Path to the robot_config.json file
        
    Returns:
        Dictionary containing MQTT configuration (from file or fallback)
    """
    try:
        return get_mqtt_config(config_path)
    except (FileNotFoundError, KeyError, json.JSONDecodeError) as e:
        print(f"Warning: Could not load MQTT config from {config_path}: {e}")
        print("Using fallback MQTT configuration for testing")
        return FALLBACK_MQTT_CONFIG.copy()

def get_mqtt_credentials_safe(config_path: str = DEFAULT_CONFIG_PATH) -> tuple[str, str, str, int]:
    """
    Get MQTT credentials with fallback to default values.
    This is useful for test scripts that should work even without the config file.
    
    Args:
        config_path: Path to the robot_config.json file
        
    Returns:
        Tuple of (broker_host, username, password, port)
    """
    mqtt_config = get_mqtt_config_safe(config_path)
    return (
        mqtt_config['broker_host'],
        mqtt_config['username'],
        mqtt_config['password'],
        mqtt_config['broker_port']
    )

if __name__ == "__main__":
    # Test the configuration loading
    try:
        mqtt_config = get_mqtt_config()
        print("MQTT Configuration loaded successfully:")
        print(f"  Broker: {mqtt_config['broker_host']}:{mqtt_config['broker_port']}")
        print(f"  Username: {mqtt_config['username']}")
        print(f"  Device ID: {mqtt_config['device_id']}")
    except Exception as e:
        print(f"Error loading configuration: {e}")
        print("Using fallback configuration:")
        mqtt_config = get_mqtt_config_safe()
        print(f"  Broker: {mqtt_config['broker_host']}:{mqtt_config['broker_port']}")
        print(f"  Username: {mqtt_config['username']}")
