# Vision Camera for SmartMower

This application captures images from V4L2-compatible cameras (including USB cameras and Raspberry Pi cameras) and publishes them to an MQTT broker using the unified configuration system.

## Features

- Capture images from any V4L2-compatible camera
- Publish frames via MQTT with JPEG compression
- Unified configuration using `/opt/smartmower/etc/config/robot_config.json`
- MQTT authentication with username/password
- Configurable resolution and frame rate
- OpenCV-based image capture and processing
- Automatic reconnection to MQTT broker
- Graceful shutdown handling
- Systemd service with security hardening

## Dependencies

- libmosquitto-dev (MQTT client library)
- libopencv-dev (OpenCV for image processing)
- libjsoncpp-dev (JSON parsing)
- build-essential (compiler and build tools)
- libstdc++-12-dev (C++ standard library)

For V4L2 camera support:
- v4l-utils (Video4Linux utilities)
- libv4l-dev (V4L2 development files)

## Installation

### On Debian/Ubuntu:

```bash
# Install dependencies
sudo apt update
sudo apt install -y libmosquitto-dev libcjson-dev libopencv-dev build-essential

# For Raspberry Pi Camera v3 support (optional)
sudo apt install -y libcamera-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good
```

## Configuration

The vision camera module uses the unified configuration system. The following settings should be present in `/opt/smartmower/etc/config/robot_config.json`:

```json
{
  "system": {
    "communication": {
      "mqtt_broker_host": "localhost",
      "mqtt_broker_port": 1883,
      "mqtt_username": "mower",
      "mqtt_password": "smart"
    }
  },
  "vision": {
    "camera": {
      "device": "/dev/video0",
      "width": 1280,
      "height": 720,
      "fps": 30,
      "mqtt_topic": "vision/camera/frame"
    }
  }
}
```

### Camera Configuration Options

- `device`: Path to the V4L2 device (e.g., `/dev/video0`)
- `width`: Frame width in pixels
- `height`: Frame height in pixels
- `fps`: Frames per second
- `mqtt_topic`: MQTT topic for publishing frames

## Building and Installation

### Build Dependencies

```bash
# Install build dependencies
sudo apt update
sudo apt install -y build-essential libopencv-dev libmosquitto-dev libjsoncpp-dev v4l-utils
```

### Building

```bash
# Clone the repository
cd ~/smartmower
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### Installation

```bash
# Install the application
sudo make install

# Set up systemd service
sudo systemctl daemon-reload
sudo systemctl enable vision-camera-rpi.service
sudo systemctl start vision-camera-rpi.service
```

### Testing

A test script is provided to verify camera and MQTT functionality:

```bash
# Install test dependencies
pip3 install paho-mqtt opencv-python

# Run the test script
cd src/vision/camera
python3 test_camera.py
```

## Troubleshooting

### Camera Access Issues

If you encounter permission issues with the camera device:

```bash
# Add your user to the video group
sudo usermod -a -G video $USER

# Or set appropriate permissions
sudo chmod 666 /dev/video0
```

### Debugging

To enable debug output:

```bash
# Set debug environment variables
export GST_DEBUG=2
export GST_DEBUG_NO_COLOR=1

# Run with debug output
/opt/smartmower/bin/vision_camera_rpi
```

## Systemd Service

The systemd service file is installed to `/etc/systemd/system/vision-camera-rpi.service`. You can control the service using:

```bash
# Check status
sudo systemctl status vision-camera-rpi.service

# View logs
journalctl -u vision-camera-rpi.service -f

# Restart the service
sudo systemctl restart vision-camera-rpi.service
```

**Viewer Controls:**
- Press `q` to quit
- Press `s` to save current frame as JPEG

**Viewer Features:**
- Real-time image display with timestamp overlay
- FPS counter and frame statistics
- Automatic image decoding from Base64
- Frame saving functionality

### Install to system (optional):
```bash
sudo make install
```

## MQTT Message Format

The application sends MQTT messages in the following JSON format:

```json
{
    "timestamp": "2023-12-07T10:30:45",
    "raw": "base64_encoded_jpeg_image_data"
}
```

## Building for Raspberry Pi

The Makefile will automatically detect if it's running on a Raspberry Pi and enable the appropriate camera support. Just run:

```bash
make
```

## Clean Up

To clean the build files:
```bash
make clean
```

## License

This project is licensed under the MIT License - see the LICENSE file for details.
