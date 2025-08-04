#!/usr/bin/env python3
"""
Test script for the Vision Camera RPi module.
This script tests the camera capture and MQTT publishing functionality.
"""
import os
import sys
import json
import time
import paho.mqtt.client as mqtt
import cv2
import numpy as np
from datetime import datetime

# Configuration
CONFIG_FILE = "/opt/smartmower/etc/config/robot_config.json"
TEST_IMAGE_PATH = "/tmp/test_camera.jpg"

# Load configuration
def load_config():
    """Load configuration from the unified config file."""
    try:
        with open(CONFIG_FILE, 'r') as f:
            config = json.load(f)
        
        # Get MQTT settings
        mqtt_config = config.get('system', {}).get('communication', {})
        
        return {
            'mqtt': {
                'host': mqtt_config.get('mqtt_broker_host', 'localhost'),
                'port': mqtt_config.get('mqtt_broker_port', 1883),
                'username': mqtt_config.get('mqtt_username', 'mower'),
                'password': mqtt_config.get('mqtt_password', 'smart'),
                'topic': 'vision/camera/test'
            },
            'camera': {
                'device': '/dev/video0',  # Default camera device
                'width': 1280,
                'height': 720,
                'fps': 30
            }
        }
    except Exception as e:
        print(f"Error loading config: {e}")
        sys.exit(1)

def test_camera():
    """Test camera capture functionality."""
    config = load_config()
    cam_config = config['camera']
    
    print(f"Testing camera at {cam_config['device']}...")
    print(f"Resolution: {cam_config['width']}x{cam_config['height']} @ {cam_config['fps']} FPS")
    
    # Try to open the camera
    cap = cv2.VideoCapture(cam_config['device'], cv2.CAP_V4L2)
    
    if not cap.isOpened():
        print(f"Failed to open camera at {cam_config['device']}")
        return False
    
    # Set camera properties
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_config['width'])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_config['height'])
    cap.set(cv2.CAP_PROP_FPS, cam_config['fps'])
    
    # Capture a test frame
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame")
        cap.release()
        return False
    
    # Save the test image
    cv2.imwrite(TEST_IMAGE_PATH, frame)
    print(f"Test image saved to {TEST_IMAGE_PATH}")
    
    # Release the camera
    cap.release()
    return True

def test_mqtt():
    """Test MQTT connectivity."""
    config = load_config()
    mqtt_config = config['mqtt']
    
    print(f"Testing MQTT connection to {mqtt_config['host']}:{mqtt_config['port']}...")
    
    # Create MQTT client
    client = mqtt.Client()
    
    # Set credentials if provided
    if mqtt_config.get('username'):
        client.username_pw_set(mqtt_config['username'], mqtt_config.get('password', ''))
    
    connected = False
    
    def on_connect(client, userdata, flags, rc):
        nonlocal connected
        if rc == 0:
            print("Connected to MQTT broker")
            connected = True
        else:
            print(f"Failed to connect to MQTT broker: {mqtt.connack_string(rc)}")
    
    def on_disconnect(client, userdata, rc):
        print(f"Disconnected from MQTT broker: {mqtt.connack_string(rc) if rc != 0 else 'Normal'}")
    
    # Set callbacks
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    
    try:
        # Connect to broker with timeout
        client.connect_async(mqtt_config['host'], mqtt_config['port'], 60)
        client.loop_start()
        
        # Wait for connection
        for _ in range(10):  # 5 second timeout
            if connected:
                break
            time.sleep(0.5)
        
        if not connected:
            print("Timed out waiting for MQTT connection")
            return False
        
        # Publish a test message
        test_msg = {
            'timestamp': datetime.utcnow().isoformat(),
            'status': 'test',
            'message': 'MQTT test message from vision camera test script'
        }
        
        result = client.publish(
            mqtt_config['topic'],
            json.dumps(test_msg),
            qos=1,
            retain=False
        )
        
        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            print(f"Failed to publish test message: {result.rc}")
            return False
        
        print(f"Published test message to topic: {mqtt_config['topic']}")
        return True
        
    except Exception as e:
        print(f"MQTT test failed: {e}")
        return False
    finally:
        client.loop_stop()
        client.disconnect()

def main():
    """Main test function."""
    print("=== Vision Camera RPi Test ===")
    
    # Test camera
    print("\n[1/2] Testing camera...")
    camera_ok = test_camera()
    print(f"Camera test: {'PASSED' if camera_ok else 'FAILED'}")
    
    # Test MQTT
    print("\n[2/2] Testing MQTT...")
    mqtt_ok = test_mqtt()
    print(f"MQTT test: {'PASSED' if mqtt_ok else 'FAILED'}")
    
    # Print summary
    print("\n=== Test Summary ===")
    print(f"Camera: {'PASSED' if camera_ok else 'FAILED'}")
    print(f"MQTT: {'PASSED' if mqtt_ok else 'FAILED'}")
    
    if camera_ok and mqtt_ok:
        print("\n✅ All tests passed!")
        return 0
    else:
        print("\n❌ Some tests failed!")
        return 1

if __name__ == "__main__":
    sys.exit(main())
