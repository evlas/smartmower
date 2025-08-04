#!/usr/bin/env python3

import paho.mqtt.client as mqtt
import json
import time
import sys
import os

# Add parent directory to path to import config_utils
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'vision'))
from config_utils import get_mqtt_credentials_safe

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    if rc == 0:
        print("Connection successful!")
        client.subscribe("smartmower/fusion/data")
        client.subscribe("smartmower/fusion/status")
    else:
        print(f"Connection failed with code {rc}")

def on_message(client, userdata, msg):
    try:
        topic = msg.topic
        payload = msg.payload.decode('utf-8')
        data = json.loads(payload)
        
        print(f"\n=== MQTT Message from {topic} ===")
        
        if topic == "smartmower/fusion/data":
            print("🔄 FUSION DATA:")
            print(f"  Type: {data.get('type', 'N/A')}")
            print(f"  Timestamp: {data.get('timestamp', 'N/A')}")
            
            # Position
            if 'position' in data:
                pos = data['position']
                print(f"  📍 Position:")
                print(f"    X: {pos.get('x', 0):.3f} m")
                print(f"    Y: {pos.get('y', 0):.3f} m") 
                print(f"    Z: {pos.get('z', 0):.3f} m")
            
            # Velocity
            if 'velocity' in data:
                vel = data['velocity']
                print(f"  🏃 Velocity:")
                print(f"    VX: {vel.get('vx', 0):.3f} m/s")
                print(f"    VY: {vel.get('vy', 0):.3f} m/s")
                print(f"    VZ: {vel.get('vz', 0):.3f} m/s")
                print(f"    Speed: {vel.get('speed', 0):.3f} m/s")
            
            # Orientation
            if 'orientation' in data:
                orient = data['orientation']
                print(f"  🧭 Orientation:")
                print(f"    Roll:  {orient.get('roll', 0):.3f} rad ({orient.get('roll', 0)*57.2958:.1f}°)")
                print(f"    Pitch: {orient.get('pitch', 0):.3f} rad ({orient.get('pitch', 0)*57.2958:.1f}°)")
                print(f"    Yaw:   {orient.get('yaw', 0):.3f} rad ({orient.get('yaw', 0)*57.2958:.1f}°)")
            
            # Uncertainty
            if 'uncertainty' in data:
                unc = data['uncertainty']
                print(f"  📊 Uncertainty (σ):")
                print(f"    Pos X: {unc.get('position_x', 0):.3f} m")
                print(f"    Pos Y: {unc.get('position_y', 0):.3f} m")
                print(f"    Vel X: {unc.get('velocity_x', 0):.3f} m/s")
                print(f"    Vel Y: {unc.get('velocity_y', 0):.3f} m/s")
                print(f"    Yaw:   {unc.get('yaw', 0):.3f} rad")
                
        elif topic == "smartmower/fusion/status":
            print("📊 FUSION STATUS:")
            print(f"  Type: {data.get('type', 'N/A')}")
            print(f"  Timestamp: {data.get('timestamp', 'N/A')}")
            if 'system' in data:
                sys_info = data['system']
                print(f"  System:")
                print(f"    Running: {sys_info.get('running', False)}")
                print(f"    Initialized: {sys_info.get('initialized', False)}")
        
        print("=" * 50)
        
    except Exception as e:
        print(f"Error processing message: {e}")
        print(f"Raw payload: {msg.payload}")

def main():
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
        
        print("🔍 Listening for fusion sensor messages...")
        print("📡 Subscribed to:")
        print("  - smartmower/fusion/data")
        print("  - smartmower/fusion/status")
        print("\nPress Ctrl+C to stop")
        
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n🛑 Stopping...")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        client.loop_stop()
        client.disconnect()

if __name__ == "__main__":
    main()
