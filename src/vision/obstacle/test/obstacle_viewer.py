#!/usr/bin/env python3
"""
Obstacle Viewer for SfM Obstacle Detection System
Visualizza gli ostacoli rilevati dal sistema Structure from Motion

Autore: SmartMower Vision System
Data: 2025-07-23
"""

import json
import time
import argparse
from datetime import datetime
import numpy as np
import cv2
import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import threading
import math
import sys
import os

# Add parent directory to path to import config_utils
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))
from config_utils import get_mqtt_credentials_safe

# MQTT Configuration
broker_host, username, password, port = get_mqtt_credentials_safe()
print(f"Using MQTT config: {broker_host}:{port} (user: {username})")

class ObstacleViewer:
    def __init__(self, broker_host=broker_host, broker_port=port, 
                 obstacle_topic="smartmower/vision/obstacles", 
                 username=username, password=password):
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.obstacle_topic = obstacle_topic
        self.username = username
        self.password = password
        
        # Data storage
        self.obstacles = []
        self.obstacle_history = deque(maxlen=100)  # Keep last 100 obstacle sets
        self.data_lock = threading.Lock()
        
        # MQTT client
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect
        
        # Statistics
        self.total_messages = 0
        self.last_message_time = None
        self.fps = 0
        self.robot_velocity = 0.0  # m/s
        self.robot_velocity_history = deque(maxlen=60)  # Keep last 60 velocity readings
        self.last_velocity_update = 0
        
        # Visualization settings
        self.max_distance = 5.0
        self.grid_size = 0.5  # meters
        self.robot_size = 0.3  # meters
        
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print(f"Connected to MQTT broker at {self.broker_host}:{self.broker_port}")
            client.subscribe(self.obstacle_topic)
            print(f"Subscribed to topic: {self.obstacle_topic}")
        else:
            print(f"Failed to connect to MQTT broker. Return code: {rc}")
    
    def on_message(self, client, userdata, msg):
        try:
            # Parse JSON message
            data = json.loads(msg.payload.decode('utf-8'))
            
            with self.data_lock:
                self.obstacles = data.get('obstacles', [])
                self.obstacle_history.append({
                    'timestamp': data.get('timestamp', ''),
                    'obstacles': self.obstacles.copy(),
                    'total_points': data.get('total_points', 0)
                })
                
                # Update velocity if available
                if 'robot_velocity' in data:
                    self.robot_velocity = float(data['robot_velocity'])
                    self.robot_velocity_history.append((time.time(), self.robot_velocity))
                
                # Update statistics
                self.total_messages += 1
                current_time = time.time()
                if self.last_message_time:
                    time_diff = current_time - self.last_message_time
                    if time_diff > 0:
                        self.fps = 1.0 / time_diff
                self.last_message_time = current_time
            
            print(f"Received {len(self.obstacles)} obstacles, "
                  f"Total points: {data.get('total_points', 0)}, "
                  f"FPS: {self.fps:.1f}")
            
        except json.JSONDecodeError as e:
            print(f"Failed to parse JSON: {e}")
        except Exception as e:
            print(f"Error processing message: {e}")
    
    def on_disconnect(self, client, userdata, rc):
        print("Disconnected from MQTT broker")
    
    def start_mqtt(self):
        """Start MQTT client in background thread"""
        if self.username and self.password:
            self.client.username_pw_set(self.username, self.password)
        
        self.client.connect(self.broker_host, self.broker_port, 60)
        self.client.loop_start()
    
    def stop_mqtt(self):
        """Stop MQTT client"""
        self.client.loop_stop()
        self.client.disconnect()
    
    def create_top_down_view(self, width=800, height=600):
        """Create a top-down view of obstacles"""
        # Create blank image
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Calculate scale (pixels per meter)
        scale = min(width, height) / (2 * self.max_distance)
        center_x = width // 2
        center_y = height - 50  # Robot at bottom center
        
        # Draw grid
        for i in range(int(-self.max_distance), int(self.max_distance) + 1):
            if i == 0:
                continue
            # Vertical lines
            x = int(center_x + i * self.grid_size * scale)
            if 0 <= x < width:
                cv2.line(img, (x, 0), (x, height), (30, 30, 30), 1)
            
            # Horizontal lines
            y = int(center_y - i * self.grid_size * scale)
            if 0 <= y < height:
                cv2.line(img, (0, y), (width, y), (30, 30, 30), 1)
        
        # Draw coordinate axes
        cv2.line(img, (center_x, 0), (center_x, height), (50, 50, 50), 2)  # Y axis
        cv2.line(img, (0, center_y), (width, center_y), (50, 50, 50), 2)   # X axis
        
        # Draw robot
        robot_radius = int(self.robot_size * scale)
        cv2.circle(img, (center_x, center_y), robot_radius, (100, 100, 255), -1)
        cv2.putText(img, "ROBOT", (center_x - 25, center_y + 5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # Draw obstacles
        with self.data_lock:
            for obstacle in self.obstacles:
                distance = obstacle.get('distance', 0)
                if distance <= 0 or distance > self.max_distance:
                    continue
                
                # Convert image coordinates to world coordinates
                # Assume camera is pointing forward, x is left-right, y is forward
                img_x = obstacle.get('x', 320)  # Default to center
                img_y = obstacle.get('y', 240)
                
                # Simple projection (this would need proper camera calibration)
                world_x = (img_x - 320) * distance * 0.001  # Rough conversion
                world_y = distance
                
                # Convert to image coordinates
                pixel_x = int(center_x + world_x * scale)
                pixel_y = int(center_y - world_y * scale)
                
                if 0 <= pixel_x < width and 0 <= pixel_y < height:
                    # Color based on distance
                    if distance < 1.0:
                        color = (0, 0, 255)  # Red - very close
                    elif distance < 2.0:
                        color = (0, 165, 255)  # Orange - close
                    elif distance < 3.0:
                        color = (0, 255, 255)  # Yellow - medium
                    else:
                        color = (0, 255, 0)  # Green - far
                    
                    # Draw obstacle
                    cv2.circle(img, (pixel_x, pixel_y), 5, color, -1)
                    
                    # Draw distance label
                    dist_text = f"{distance:.1f}m"
                    cv2.putText(img, dist_text, (pixel_x + 8, pixel_y - 8),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.3, color, 1)
                    
                    # Draw velocity vector if available
                    vel_x = obstacle.get('velocity_x', 0)
                    vel_y = obstacle.get('velocity_y', 0)
                    if abs(vel_x) > 0.1 or abs(vel_y) > 0.1:
                        end_x = int(pixel_x + vel_x * 2)
                        end_y = int(pixel_y + vel_y * 2)
                        cv2.arrowedLine(img, (pixel_x, pixel_y), (end_x, end_y), color, 1)
        
        # Draw statistics
        stats_y = 20
        stats_spacing = 20
        
        # Basic stats
        cv2.putText(img, f"Obstacles: {len(self.obstacles)}", (10, stats_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(img, f"Messages: {self.total_messages}", (10, stats_y + stats_spacing),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(img, f"FPS: {self.fps:.1f}", (10, stats_y + stats_spacing*2),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Velocity display
        velocity_color = (0, 200, 255)  # Orange
        velocity_text = f"Velocity: {self.robot_velocity:.2f} m/s"
        cv2.putText(img, velocity_text, (10, stats_y + stats_spacing*3),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, velocity_color, 1)
        
        # Draw velocity history graph
        self.draw_velocity_graph(img, width - 200, 20, 180, 60)
        
        # Draw legend
        legend_x = width - 150
        legend_y = 20
        cv2.putText(img, "Distance Legend:", (legend_x, legend_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.circle(img, (legend_x + 10, legend_y + 20), 3, (0, 0, 255), -1)
        cv2.putText(img, "< 1.0m", (legend_x + 20, legend_y + 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
        cv2.circle(img, (legend_x + 10, legend_y + 40), 3, (0, 165, 255), -1)
        cv2.putText(img, "< 2.0m", (legend_x + 20, legend_y + 45),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
        cv2.circle(img, (legend_x + 10, legend_y + 60), 3, (0, 255, 255), -1)
        cv2.putText(img, "< 3.0m", (legend_x + 20, legend_y + 65),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
        cv2.circle(img, (legend_x + 10, legend_y + 80), 3, (0, 255, 0), -1)
        cv2.putText(img, "> 3.0m", (legend_x + 20, legend_y + 85),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
        
        return img
    
    def draw_velocity_graph(self, img, x, y, width, height):
        """Draw a graph of velocity over time"""
        if len(self.robot_velocity_history) < 2:
            return
            
        # Create graph background
        graph = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Draw grid
        for i in range(0, width, 20):
            cv2.line(graph, (i, 0), (i, height), (40, 40, 40), 1)
        for i in range(0, height, 10):
            cv2.line(graph, (0, i), (width, i), (40, 40, 40), 1)
        
        # Get time range (last 10 seconds)
        current_time = time.time()
        min_time = current_time - 10.0
        
        # Find min/max velocity in the time range
        velocities = [v for t, v in self.robot_velocity_history if t >= min_time]
        if not velocities:
            return
            
        min_v = min(0, min(velocities) * 1.1)  # Add 10% margin
        max_v = max(1.0, max(velocities) * 1.1)  # At least 1 m/s range
        v_range = max_v - min_v
        
        if v_range == 0:  # Avoid division by zero
            v_range = 1.0
        
        # Draw zero line
        zero_y = int(height * (1 - (0 - min_v) / v_range))
        cv2.line(graph, (0, zero_y), (width-1, zero_y), (100, 100, 100), 1)
        
        # Draw velocity line
        points = []
        for t, v in self.robot_velocity_history:
            if t < min_time:
                continue
            x_pos = int(width * (1 - (current_time - t) / 10.0))
            y_pos = int(height * (1 - (v - min_v) / v_range))
            points.append((x_pos, y_pos))
        
        if len(points) > 1:
            # Draw line
            for i in range(1, len(points)):
                cv2.line(graph, points[i-1], points[i], (0, 200, 255), 2)
        
        # Draw current value
        cv2.putText(graph, f"{self.robot_velocity:.1f} m/s", 
                   (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 255), 1)
        
        # Copy graph to the main image
        img[y:y+height, x:x+width] = graph
        
        # Draw border
        cv2.rectangle(img, (x, y), (x+width-1, y+height-1), (100, 100, 100), 1)
    
    def create_distance_histogram(self):
        """Create histogram of obstacle distances"""
        distances = []
        with self.data_lock:
            for obstacle in self.obstacles:
                dist = obstacle.get('distance', 0)
                if 0 < dist <= self.max_distance:
                    distances.append(dist)
        
        if not distances:
            return np.zeros((200, 400, 3), dtype=np.uint8)
        
        # Create histogram
        hist, bins = np.histogram(distances, bins=20, range=(0, self.max_distance))
        
        # Create image
        img = np.zeros((200, 400, 3), dtype=np.uint8)
        
        if len(hist) > 0:
            max_count = max(hist) if max(hist) > 0 else 1
            bin_width = 400 // len(hist)
            
            for i, count in enumerate(hist):
                if count > 0:
                    height = int((count / max_count) * 150)
                    x1 = i * bin_width
                    x2 = (i + 1) * bin_width
                    y1 = 180 - height
                    y2 = 180
                    
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), -1)
                    cv2.rectangle(img, (x1, y1), (x2, y2), (255, 255, 255), 1)
        
        # Add labels
        cv2.putText(img, "Distance Histogram", (10, 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(img, "0m", (5, 195), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
        cv2.putText(img, f"{self.max_distance}m", (370, 195),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
        
        return img
    
    def run_visualization(self):
        """Run the main visualization loop"""
        print("Starting obstacle visualization...")
        print("Press 'q' to quit, 's' to save screenshot")
        
        while True:
            # Create visualizations
            top_view = self.create_top_down_view()
            histogram = self.create_distance_histogram()
            
            # Combine views
            combined = np.zeros((600, 1200, 3), dtype=np.uint8)
            combined[:600, :800] = top_view
            combined[200:400, 800:1200] = histogram
            
            # Show combined view
            cv2.imshow('SfM Obstacle Detection', combined)
            
            key = cv2.waitKey(50) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                filename = f"obstacle_view_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
                cv2.imwrite(filename, combined)
                print(f"Screenshot saved as {filename}")
        
        cv2.destroyAllWindows()
    
    def print_statistics(self):
        """Print current statistics"""
        with self.data_lock:
            print(f"\n=== SfM Obstacle Detection Statistics ===")
            print(f"Total messages received: {self.total_messages}")
            print(f"Current obstacles: {len(self.obstacles)}")
            print(f"Processing FPS: {self.fps:.1f}")
            
            if self.obstacles:
                distances = [obs.get('distance', 0) for obs in self.obstacles]
                print(f"Distance range: {min(distances):.2f}m - {max(distances):.2f}m")
                print(f"Average distance: {np.mean(distances):.2f}m")
                
                close_obstacles = [d for d in distances if d < 1.0]
                if close_obstacles:
                    print(f"Close obstacles (<1m): {len(close_obstacles)}")

def main():
    parser = argparse.ArgumentParser(description='SfM Obstacle Detection Viewer')
    parser.add_argument('--broker', default='localhost', help='MQTT broker host')
    parser.add_argument('--port', type=int, default=1883, help='MQTT broker port')
    parser.add_argument('--topic', default='smartmower/vision/obstacles', help='MQTT topic')
    parser.add_argument('--username', default='mower', help='MQTT username')
    parser.add_argument('--password', default='smart', help='MQTT password')
    parser.add_argument('--stats-only', action='store_true', help='Print statistics only')
    
    args = parser.parse_args()
    
    print("SfM Obstacle Detection Viewer")
    print("=" * 40)
    print(f"Broker: {args.broker}:{args.port}")
    print(f"Topic: {args.topic}")
    print(f"Username: {args.username}")
    print("=" * 40)
    
    viewer = ObstacleViewer(
        broker_host=args.broker,
        broker_port=args.port,
        obstacle_topic=args.topic,
        username=args.username,
        password=args.password
    )
    
    try:
        viewer.start_mqtt()
        
        if args.stats_only:
            # Just print statistics periodically
            while True:
                time.sleep(5)
                viewer.print_statistics()
        else:
            # Run full visualization
            time.sleep(2)  # Wait for initial connection
            viewer.run_visualization()
            
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        viewer.stop_mqtt()

if __name__ == "__main__":
    main()
