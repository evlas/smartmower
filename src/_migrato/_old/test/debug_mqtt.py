#!/usr/bin/env python3

import paho.mqtt.client as mqtt
import json
import time
import sys
import os

# Add parent directory to path to import config_utils
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))
from config_utils import get_mqtt_credentials_safe

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    if rc == 0:
        client.subscribe("smartmower/vision/camera")

def on_message(client, userdata, msg):
    print(f"Received message on topic {msg.topic}")
    print(f"Message length: {len(msg.payload)} bytes")
    
    try:
        # Try to parse as JSON
        data = json.loads(msg.payload.decode('utf-8'))
        print("JSON keys:", list(data.keys()))
        
        if 'timestamp' in data:
            print(f"Timestamp: {data['timestamp']}")
        
        # Check for different possible image field names
        for key in ['image', 'raw', 'data', 'base64']:
            if key in data:
                print(f"Found image data in field '{key}', length: {len(data[key])}")
                break
        else:
            print("No image data found in expected fields")
            
    except json.JSONDecodeError as e:
        print(f"Failed to parse JSON: {e}")
        print(f"First 200 chars: {msg.payload[:200]}")
    except Exception as e:
        print(f"Error processing message: {e}")

# Load MQTT credentials from unified configuration
broker_host, username, password, port = get_mqtt_credentials_safe()
print(f"Using MQTT config: {broker_host}:{port} (user: {username})")

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.username_pw_set(username, password)

try:
    client.connect(broker_host, port, 60)
    client.loop_start()
    print("Listening for messages... Press Ctrl+C to stop")
    time.sleep(10)  # Listen for 10 seconds
except KeyboardInterrupt:
    print("\nStopping...")
finally:
    client.disconnect()
    client.loop_stop()
