# Makefile for SfM Obstacle Detector
# SmartMower Vision System

# Compiler and flags
CXX = g++
CXXFLAGS = -Wall -Wextra -O2 -std=c++17 -pthread -Iinclude -I/usr/include/jsoncpp -I/usr/include/opencv4
PKG_CONFIG = pkg-config

# Directory structure
SRCDIR = src
OBJDIR = obj
BINDIR = bin
SYSTEMDDIR = systemd

# OpenCV flags
OPENCV_CFLAGS = $(shell $(PKG_CONFIG) --cflags opencv4 2>/dev/null || $(PKG_CONFIG) --cflags opencv)
OPENCV_LIBS = $(shell $(PKG_CONFIG) --libs opencv4 2>/dev/null || $(PKG_CONFIG) --libs opencv)

# Additional libraries
LIBS = -lmosquitto -lcjson -ljsoncpp -lpthread -lm $(OPENCV_LIBS)

# Configuration (unified)
CONFIG = robot_config.json

# Installation directories
PREFIX ?= /opt/smartmower
INSTALL_BINDIR = $(PREFIX)/bin
SYSCONFDIR = $(PREFIX)/etc/config

# Target executable
TARGET = $(BINDIR)/sfm_obstacle_detector

# Source files
SOURCES = $(SRCDIR)/sfm_obstacle_detector_fixed.cpp
OBJECTS = $(patsubst $(SRCDIR)/%.cpp,$(OBJDIR)/%.o,$(SOURCES))

# Default target
all: $(TARGET)

# Create directories
$(OBJDIR) $(BINDIR):
	mkdir -p $@

# Build the executable
$(TARGET): $(OBJECTS) | $(BINDIR)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)

# Compile source files
$(OBJDIR)/%.o: $(SRCDIR)/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) -c -o $@ $<

# Clean build files
clean:
	rm -rf $(OBJDIR) $(BINDIR)

# Install target
install: $(TARGET)
	# Install binary
	install -d $(DESTDIR)$(INSTALL_BINDIR)
	install -m 755 $(TARGET) $(DESTDIR)$(INSTALL_BINDIR)/
	# Create a symlink from sfm_obstacle_detector to obstacle_detector
	ln -sf $(notdir $(TARGET)) $(DESTDIR)$(INSTALL_BINDIR)/obstacle_detector
	# Create necessary directories
	install -d $(DESTDIR)/opt/smartmower/log
	install -d $(DESTDIR)/opt/smartmower/data/vision
	# Set ownership and permissions
	chown -R smartmower:smartmower $(DESTDIR)/opt/smartmower/log $(DESTDIR)/opt/smartmower/data 2>/dev/null || true
	chmod 755 $(DESTDIR)/opt/smartmower/log $(DESTDIR)/opt/smartmower/data $(DESTDIR)/opt/smartmower/data/vision 2>/dev/null || true
	# Install systemd service file
	install -d $(DESTDIR)/etc/systemd/system
	install -m 644 systemd/obstacle_detector.service $(DESTDIR)/etc/systemd/system/

# Uninstall target
uninstall:
	rm -f $(DESTDIR)$(BINDIR)/$(TARGET)
	rm -f $(DESTDIR)$(BINDIR)/obstacle_detector
	rm -f $(DESTDIR)/etc/systemd/system/smartmower-obstacle.service

# Test target - runs the detector with verbose output
test: $(TARGET)
	./$(TARGET)

# Debug target - compile with debug symbols
debug: CXXFLAGS += -g -DDEBUG
debug: $(TARGET)

# Check dependencies
check-deps:
	@echo "Checking dependencies..."
	@$(PKG_CONFIG) --exists opencv4 || $(PKG_CONFIG) --exists opencv || (echo "OpenCV not found" && exit 1)
	@$(PKG_CONFIG) --exists libmosquitto || (echo "libmosquitto not found" && exit 1)
	@echo "All dependencies found"

# Print configuration
info:
	@echo "SfM Obstacle Detector Build Configuration"
	@echo "========================================"
	@echo "Compiler: $(CXX)"
	@echo "Flags: $(CXXFLAGS)"
	@echo "OpenCV CFLAGS: $(OPENCV_CFLAGS)"
	@echo "OpenCV LIBS: $(OPENCV_LIBS)"
	@echo "Additional LIBS: -lmosquitto -lcjson -lpthread -lm"
	@echo "Target: $(TARGET)"

.PHONY: all clean install uninstall test debug check-deps info
