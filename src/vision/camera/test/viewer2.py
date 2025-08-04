#!/usr/bin/env python3
"""
Simple MQTT Image Viewer for SmartMower Vision Camera
Subscribes to MQTT topic and displays images with timestamp overlay
"""

import json
import base64
import cv2
import numpy as np
import paho.mqtt.client as mqtt
import threading
import time
from datetime import datetime
import argparse
import sys

class ImageViewer:
    def __init__(self, broker_host="localhost", broker_port=1883, 
                 topic="smartmower/vision/camera", username=None, password=None):
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.topic = topic
        self.username = username
        self.password = password
        
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect
        
        self.latest_image = None
        self.latest_timestamp = None
        self.image_lock = threading.Lock()
        self.running = True
        
        # Statistics
        self.frame_count = 0
        self.start_time = time.time()
        
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print(f"Connected to MQTT broker at {self.broker_host}:{self.broker_port}")
            client.subscribe(self.topic)
            print(f"Subscribed to topic: {self.topic}")
        else:
            print(f"Failed to connect to MQTT broker. Return code: {rc}")
            
    def on_disconnect(self, client, userdata, rc):
        print("Disconnected from MQTT broker")
        
    def on_message(self, client, userdata, msg):
        try:
            print(f"Received message on topic {msg.topic}")
            print(f"Message length: {len(msg.payload)} bytes")
            
            # Parse JSON message
            message = json.loads(msg.payload.decode())
            timestamp = message.get('timestamp', '')
            raw_data = message.get('image', '')
            
            if not raw_data:
                print("No image data in message")
                return
                
            print(f"Raw data length: {len(raw_data)} characters")
            print(f"Timestamp: {timestamp}")
            
            # Decode base64 image
            try:
                image_bytes = base64.b64decode(raw_data)
                print(f"Decoded image size: {len(image_bytes)} bytes")
            except Exception as e:
                print(f"Base64 decode error: {e}")
                return
            
            # Convert to OpenCV image
            nparr = np.frombuffer(image_bytes, np.uint8)
            image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if image is None:
                print("Failed to decode image with OpenCV")
                return
                
            print(f"Image dimensions: {image.shape}")
            
            # Update latest image with thread safety
            with self.image_lock:
                self.latest_image = image.copy()
                self.latest_timestamp = timestamp
                self.frame_count += 1
                print(f"Frame updated. Total frames: {self.frame_count}")
                
        except json.JSONDecodeError as e:
            print(f"JSON decode error: {e}")
            print(f"Message content: {msg.payload[:100]}...")  # Print first 100 chars
        except Exception as e:
            print(f"Error processing message: {e}")
            import traceback
            traceback.print_exc()
            
    def add_timestamp_overlay(self, image, timestamp):
        """Add timestamp overlay to image"""
        if not timestamp:
            return image
            
        # Create a copy to avoid modifying original
        img_with_overlay = image.copy()
        
        # Parse timestamp and format it nicely
        try:
            dt = datetime.fromisoformat(timestamp.replace('Z', '+00:00'))
            display_time = dt.strftime("%Y-%m-%d %H:%M:%S")
        except:
            display_time = timestamp
            
        # Text properties
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        color = (0, 255, 0)  # Green
        thickness = 2
        
        # Get text size
        (text_width, text_height), baseline = cv2.getTextSize(
            display_time, font, font_scale, thickness)
        
        # Position at top-left with some padding
        x, y = 10, 30
        
        # Add semi-transparent background
        overlay = img_with_overlay.copy()
        cv2.rectangle(overlay, (x-5, y-text_height-5), 
                     (x+text_width+5, y+baseline+5), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, img_with_overlay, 0.3, 0, img_with_overlay)
        
        # Add text
        cv2.putText(img_with_overlay, display_time, (x, y), 
                   font, font_scale, color, thickness)
        
        return img_with_overlay
        
    def add_stats_overlay(self, image):
        """Add frame rate and statistics overlay"""
        if self.frame_count == 0:
            return image
            
        # Calculate FPS
        elapsed_time = time.time() - self.start_time
        fps = self.frame_count / elapsed_time if elapsed_time > 0 else 0
        
        # Stats text
        stats_text = f"FPS: {fps:.1f} | Frames: {self.frame_count}"
        
        # Text properties
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        color = (255, 255, 0)  # Yellow
        thickness = 1
        
        # Get text size
        (text_width, text_height), baseline = cv2.getTextSize(
            stats_text, font, font_scale, thickness)
        
        # Position at bottom-right
        h, w = image.shape[:2]
        x = w - text_width - 10
        y = h - 10
        
        # Add semi-transparent background
        overlay = image.copy()
        cv2.rectangle(overlay, (x-5, y-text_height-5), 
                     (x+text_width+5, y+baseline+5), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, image, 0.3, 0, image)
        
        # Add text
        cv2.putText(image, stats_text, (x, y), 
                   font, font_scale, color, thickness)
        
        return image
        
    def display_loop(self):
        """Main display loop"""
        print("Starting image display. Press 'q' to quit, 's' to save current frame.")
        
        while self.running:
            with self.image_lock:
                if self.latest_image is not None:
                    # Create display image with overlays
                    display_image = self.latest_image.copy()
                    display_image = self.add_timestamp_overlay(display_image, self.latest_timestamp)
                    display_image = self.add_stats_overlay(display_image)
                    
                    # Show image
                    cv2.imshow('SmartMower Vision Camera', display_image)
                    
            # Handle key presses
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q'):
                self.running = False
                break
            elif key == ord('s'):
                # Save current frame
                with self.image_lock:
                    if self.latest_image is not None:
                        filename = f"frame_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
                        cv2.imwrite(filename, self.latest_image)
                        print(f"Frame saved as {filename}")
                        
        cv2.destroyAllWindows()
        
    def start(self):
        """Start the viewer"""
        try:
            # Set up MQTT authentication only if both username and password are provided
            if self.username and self.password:
                print(f"Using authentication with username: {self.username}")
                self.client.username_pw_set(self.username, self.password)
            else:
                print("Connecting without authentication")
                
            # Connect to MQTT broker
            self.client.connect(self.broker_host, self.broker_port, 60)
            
            # Start MQTT loop in background
            self.client.loop_start()
            
            # Start display loop
            self.display_loop()
            
        except KeyboardInterrupt:
            print("\nShutting down...")
        except Exception as e:
            print(f"Error: {e}")
        finally:
            self.stop()
            
    def stop(self):
        """Stop the viewer"""
        self.running = False
        self.client.loop_stop()
        self.client.disconnect()
        cv2.destroyAllWindows()

def main():
    parser = argparse.ArgumentParser(description='MQTT Image Viewer for SmartMower Vision Camera')
    parser.add_argument('--broker', default='localhost', help='MQTT broker host (default: localhost)')
    parser.add_argument('--port', type=int, default=1883, help='MQTT broker port (default: 1883)')
    parser.add_argument('--topic', default='smartmower/vision/camera', help='MQTT topic (default: smartmower/vision/camera)')
    parser.add_argument('--username', help='MQTT username (optional)')
    parser.add_argument('--password', help='MQTT password (optional)')
    
    args = parser.parse_args()
    
    print("SmartMower Vision Camera Viewer")
    print("=" * 40)
    print(f"Broker: {args.broker}:{args.port}")
    print(f"Topic: {args.topic}")
    if args.username:
        print(f"Username: {args.username}")
        print("Authentication: Enabled")
    else:
        print("Authentication: Disabled")
    print("=" * 40)
    
    viewer = ImageViewer(
        broker_host=args.broker,
        broker_port=args.port,
        topic=args.topic,
        username=args.username,
        password=args.password
    )
    
    viewer.start()

if __name__ == "__main__":
    main()

