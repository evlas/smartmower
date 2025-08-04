# Makefile for Grass Detection Module
# SmartMower Vision System

# Compiler and flags
CXX = g++
CXXFLAGS = -Wall -Wextra -O2 -std=c++14 -pthread -Iinclude
PKG_CONFIG = pkg-config

# Directory structure
SRCDIR = src
INCDIR = include
OBJDIR = obj
BINDIR = bin
SYSTEMDDIR = systemd

# OpenCV flags
OPENCV_CFLAGS = $(shell $(PKG_CONFIG) --cflags opencv4 2>/dev/null || $(PKG_CONFIG) --cflags opencv)
OPENCV_LIBS = $(shell $(PKG_CONFIG) --libs opencv4 2>/dev/null || $(PKG_CONFIG) --libs opencv)

# Additional libraries
LIBS = -lmosquitto -lcjson -lpthread -lm $(OPENCV_LIBS)

# Configuration (unified)
CONFIG = robot_config.json

# Installation directories
PREFIX ?= /opt/smartmower
INSTALL_BINDIR = $(PREFIX)/bin
SYSCONFDIR = $(PREFIX)/etc/config

# Target executable
TARGET = $(BINDIR)/grass_detector

# Source files
SOURCES = $(wildcard $(SRCDIR)/*.cpp)
OBJECTS = $(SOURCES:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)

# Default target
all: $(TARGET)

# Create directories
$(OBJDIR) $(BINDIR):
	mkdir -p $@

# Build the executable
$(TARGET): $(OBJDIR)/grass_detector.o | $(BINDIR)
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) -o $@ $< $(LIBS)

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
	# Create necessary directories
	install -d $(DESTDIR)/opt/smartmower/log
	install -d $(DESTDIR)/opt/smartmower/data/vision
	# Set ownership and permissions
	chown -R smartmower:smartmower $(DESTDIR)/opt/smartmower/log $(DESTDIR)/opt/smartmower/data 2>/dev/null || true
	chmod 755 $(DESTDIR)/opt/smartmower/log $(DESTDIR)/opt/smartmower/data $(DESTDIR)/opt/smartmower/data/vision 2>/dev/null || true
	# Install systemd service file
	install -d $(DESTDIR)/etc/systemd/system
	install -m 644 systemd/grass_detector.service $(DESTDIR)/etc/systemd/system/

# Uninstall target
uninstall:
	rm -f $(DESTDIR)$(INSTALL_BINDIR)/grass_detector
	rm -f $(DESTDIR)/etc/systemd/system/smartmower-grass.service

.PHONY: all clean install uninstall
