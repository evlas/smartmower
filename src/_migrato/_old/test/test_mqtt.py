#!/usr/bin/env python3

import paho.mqtt.client as mqtt
import time
import sys
import os

# Add parent directory to path to import config_utils
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))
from config_utils import get_mqtt_credentials_safe

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    if rc == 0:
        print("Connection successful!")
        client.subscribe("smartmower/vision/camera")
    else:
        print(f"Connection failed with code {rc}")
        if rc == 1:
            print("Connection refused - incorrect protocol version")
        elif rc == 2:
            print("Connection refused - invalid client identifier")
        elif rc == 3:
            print("Connection refused - server unavailable")
        elif rc == 4:
            print("Connection refused - bad username or password")
        elif rc == 5:
            print("Connection refused - not authorised")

def on_message(client, userdata, msg):
    print(f"Received message on topic {msg.topic}")

def on_disconnect(client, userdata, rc):
    print(f"Disconnected with result code {rc}")

# Test 1: Connection without credentials
print("=== Test 1: Connection without credentials ===")
client1 = mqtt.Client()
client1.on_connect = on_connect
client1.on_message = on_message
client1.on_disconnect = on_disconnect

try:
    client1.connect("localhost", 1883, 60)
    client1.loop_start()
    time.sleep(2)
    client1.disconnect()
    client1.loop_stop()
except Exception as e:
    print(f"Error: {e}")

print("\n=== Test 2: Connection with credentials ===")
client2 = mqtt.Client()
client2.on_connect = on_connect
client2.on_message = on_message
client2.on_disconnect = on_disconnect
# Load MQTT credentials from unified configuration
broker_host, username, password, port = get_mqtt_credentials_safe()
print(f"Using MQTT config: {broker_host}:{port} (user: {username})")

client2.username_pw_set(username, password)

try:
    client2.connect(broker_host, port, 60)
    client2.loop_start()
    time.sleep(2)
    client2.disconnect()
    client2.loop_stop()
except Exception as e:
    print(f"Error: {e}")
