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
                 topic="smartmower/vision/camera", username="mower", password="smart"):
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.topic = topic
        self.username = username
        self.password = password
        
        # Debug flags
        self.debug = True
        self.last_debug_time = time.time()
        
        # MQTT client setup
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect
        
        # Image handling
        self.latest_image = None
        self.latest_timestamp = None
        self.image_lock = threading.Lock()
        self.running = True
        
        # Statistics
        self.frame_count = 0
        self.start_time = time.time()
        self.last_frame_time = time.time()
        
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
            current_time = time.time()
            if self.debug and current_time - self.last_debug_time > 1.0:  # Limit debug output to once per second
                print(f"\n[DEBUG] Received message on topic {msg.topic}")
                print(f"[DEBUG] Message length: {len(msg.payload)} bytes")
                self.last_debug_time = current_time
            
            try:
                # Parse JSON message
                message = json.loads(msg.payload.decode())
                
                # Extract data from new message format
                timestamp = message.get('timestamp', '')
                image_info = message.get('image_info', {})
                raw_data = message.get('image_data', '')
                
                if not raw_data:
                    print("No image data in message")
                    return
                    
                print(f"Raw data length: {len(raw_data)} characters")
                print(f"Timestamp: {timestamp}")
                print(f"Image info: {json.dumps(image_info, indent=2)}")
                
                # Decode base64 image
                try:
                    image_bytes = base64.b64decode(raw_data)
                    print(f"Decoded image size: {len(image_bytes)} bytes")
                except Exception as e:
                    print(f"Base64 decode error: {e}")
                    return
                    
            except json.JSONDecodeError as e:
                print(f"JSON decode error: {e}")
                print(f"Message content (first 200 chars): {msg.payload.decode('utf-8', errors='replace')[:200]}")
                return
            except Exception as e:
                print(f"Error processing message: {e}")
                import traceback
                traceback.print_exc()
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
        
        # Create initial window with test pattern
        test_img = np.zeros((200, 400, 3), dtype=np.uint8)
        cv2.putText(test_img, "Waiting for MQTT connection...", (20, 40), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(test_img, "Press 'q' to quit", (20, 80), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 200), 1)
        cv2.imshow('SmartMower Vision Camera', test_img)
        cv2.waitKey(100)  # Give it time to display
        
        last_status_print = 0
        while self.running:
            current_time = time.time()
            
            # Print status periodically
            if current_time - last_status_print > 2.0:  # Print status every 2 seconds
                with self.image_lock:
                    img_status = "has image" if self.latest_image is not None else "no image"
                    fps = self.frame_count / (current_time - self.start_time) if (current_time - self.start_time) > 0 else 0
                    print(f"\rStatus: {img_status} | FPS: {fps:.1f} | Frames: {self.frame_count} | Waiting for 'q' to quit...", end='')
                    last_status_print = current_time
            
            with self.image_lock:
                if self.latest_image is not None:
                    try:
                        # Create display image with overlays
                        display_image = self.latest_image.copy()
                        display_image = self.add_timestamp_overlay(display_image, self.latest_timestamp)
                        display_image = self.add_stats_overlay(display_image)
                        
                        # Show image
                        cv2.imshow('SmartMower Vision Camera', display_image)
                        self.last_frame_time = time.time()
                        
                    except Exception as e:
                        print(f"\nError displaying image: {e}")
                        # Show error image
                        error_img = np.zeros((200, 500, 3), dtype=np.uint8)
                        cv2.putText(error_img, f"Error: {str(e)}", (10, 40), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
                        cv2.imshow('SmartMower Vision Camera - ERROR', error_img)
                else:
                    # Show waiting image if no frames received yet
                    wait_img = np.zeros((200, 500, 3), dtype=np.uint8)
                    wait_time = int(time.time() - self.start_time)
                    cv2.putText(wait_img, "Waiting for video frames...", (20, 40), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 200), 1)
                    cv2.putText(wait_img, f"Time waiting: {wait_time}s", (20, 80), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 0), 1)
                    cv2.imshow('SmartMower Vision Camera', wait_img)
            
            # Handle key presses with shorter timeout for more responsive UI
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q'):
                print("\nQuit requested by user")
                self.running = False
                break
            elif key == ord('s') and self.latest_image is not None:
                # Save current frame
                with self.image_lock:
                    if self.latest_image is not None:
                        filename = f"frame_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
                        try:
                            cv2.imwrite(filename, self.latest_image)
                            print(f"\nFrame saved as {filename}")
                        except Exception as e:
                            print(f"\nError saving frame: {e}")
                        
        cv2.destroyAllWindows()
        
    def start(self):
        """Start the viewer"""
        try:
            print(f"Connecting to MQTT broker at {self.broker_host}:{self.broker_port}...")
            
            # Set up MQTT authentication if provided
            if self.username and self.password:
                print("Using MQTT authentication")
                self.client.username_pw_set(self.username, self.password)
            
            # Set last will and testament
            self.client.will_set(self.topic + "/status", payload="offline", qos=0, retain=True)
            
            # Connect to MQTT broker with timeout
            try:
                self.client.connect(self.broker_host, self.broker_port, 60)
                print("Connected to MQTT broker")
            except Exception as e:
                print(f"\nERROR: Could not connect to MQTT broker at {self.broker_host}:{self.broker_port}")
                print(f"Error details: {e}")
                print("Please check:")
                print("1. Is the MQTT broker running?")
                print("2. Is the broker address correct?")
                print("3. Are the username/password correct?")
                print("4. Is there a firewall blocking the connection?")
                self.stop()
                return
            
            # Start MQTT loop in background
            self.client.loop_start()
            print(f"Subscribed to topic: {self.topic}")
            print("\nWaiting for video frames... (Press 'q' to quit, 's' to save frame)")
            
            # Start display loop
            self.display_loop()
            
        except KeyboardInterrupt:
            print("\nShutdown requested by user...")
        except Exception as e:
            print(f"\nERROR: {e}")
            import traceback
            traceback.print_exc()
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
    parser.add_argument('--topic', default='smartmower/vision/camera', help='MQTT topic (default: /smartmower/vision/camera)')
    parser.add_argument('--username', help='MQTT username')
    parser.add_argument('--password', help='MQTT password')
    
    args = parser.parse_args()
    
    print("SmartMower Vision Camera Viewer")
    print("=" * 40)
    print(f"Broker: {args.broker}:{args.port}")
    print(f"Topic: {args.topic}")
    if args.username:
        print(f"Username: {args.username}")
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
