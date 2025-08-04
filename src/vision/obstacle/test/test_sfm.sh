#!/bin/bash

# Test script for SfM Obstacle Detection System
# SmartMower Vision System

echo "=== SfM Obstacle Detection System Test ==="
echo

# Check if executable exists
if [ ! -f "./sfm_obstacle_detector" ]; then
    echo "❌ sfm_obstacle_detector not found. Run 'make' first."
    exit 1
fi

echo "✅ Executable found"

# Check MQTT broker
echo "🔍 Checking MQTT broker connection..."
if mosquitto_pub -h localhost -t "test" -m "test" -u mower -P smart >/dev/null 2>&1; then
    echo "✅ MQTT broker accessible"
else
    echo "❌ MQTT broker not accessible. Check if mosquitto is running and credentials are correct."
    exit 1
fi

# Check if camera is publishing
echo "🔍 Checking camera feed..."
timeout 5 mosquitto_sub -h localhost -t "smartmower/vision/camera" -u mower -P smart -C 1 >/dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "✅ Camera feed available"
else
    echo "⚠️  Camera feed not available. The SfM detector will wait for camera data."
fi

# Create test directories
mkdir -p debug_output
mkdir -p logs

echo
echo "🚀 Starting SfM Obstacle Detector..."
echo "   - Configuration: config.json"
echo "   - Debug frames will be saved in current directory"
echo "   - Obstacle data published to: smartmower/vision/obstacles"
echo "   - Press Ctrl+C to stop"
echo

# Start the detector
./sfm_obstacle_detector
