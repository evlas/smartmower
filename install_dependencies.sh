#!/bin/bash

# Update package lists
echo "Updating package lists..."
sudo apt-get update

# Install build essentials and basic tools
echo "Installing build essentials and basic tools..."
sudo apt-get install -y build-essential cmake git pkg-config

# Install Mosquitto MQTT library
echo "Installing Mosquitto MQTT library..."
sudo apt-get install -y libmosquitto-dev mosquitto mosquitto-clients

# Install JSON libraries
echo "Installing JSON libraries..."
sudo apt-get install -y libjson-c-dev libjansson-dev

# Install Eigen3 for linear algebra
echo "Installing Eigen3 library..."
sudo apt-get install -y libeigen3-dev

# Install OpenCV and computer vision libraries
echo "Installing OpenCV and computer vision libraries..."
sudo apt-get install -y \
    libopencv-dev \
    libopencv-core-dev \
    libopencv-highgui-dev \
    libopencv-imgproc-dev \
    libopencv-video-dev \
    libopencv-calib3d-dev \
    libopencv-features2d-dev \
    libopencv-objdetect-dev \
    libopencv-ximgproc-dev \
    libopencv-videoio-dev \
    libopencv-imgcodecs-dev

# Install CJSON library
echo "Installing CJSON library..."
sudo apt-get install -y libcjson-dev

# Install threading and math libraries
echo "Installing threading and math libraries..."
sudo apt-get install -y libpthread-stubs0-dev

# Install systemd service files (if needed)
echo "Installing systemd service files..."
sudo apt-get install -y systemd

# Install additional development tools
echo "Installing additional development tools..."
sudo apt-get install -y \
    libtbb-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    libgtk-3-dev \
    libatlas-base-dev \
    gfortran

echo "\nAll dependencies have been installed successfully!"
echo "You can now build the project by running 'make' in the project root directory."
