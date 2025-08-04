# Compiler
CXX = g++
CXXFLAGS = -std=c++11 -Wall -O2 -Iinclude

# Dependencies
OPENCV_LIBS = `pkg-config --cflags --libs opencv4`
MOSQUITTO_LIBS = -lmosquitto -lcjson -lpthread

# Configuration (unified)
CONFIG = robot_config.json

# Installation directories
PREFIX ?= /opt/smartmower
BINDIR = $(PREFIX)/bin
SYSCONFDIR = $(PREFIX)/etc/config

# Source files
SRC = src/perimeter_detector.cpp
TARGET = perimeter_detector

# Build rules
all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(OPENCV_LIBS) $(MOSQUITTO_LIBS)

clean:
	rm -f $(TARGET) *.o

# Install target
install: $(TARGET)
	# Install binary
	install -d $(DESTDIR)$(BINDIR)
	install -m 755 $(TARGET) $(DESTDIR)$(BINDIR)/
	# Create necessary directories
	install -d $(DESTDIR)/opt/smartmower/log
	install -d $(DESTDIR)/opt/smartmower/data/vision
	# Set ownership and permissions
	chown -R smartmower:smartmower $(DESTDIR)/opt/smartmower/log $(DESTDIR)/opt/smartmower/data 2>/dev/null || true
	chmod 755 $(DESTDIR)/opt/smartmower/log $(DESTDIR)/opt/smartmower/data $(DESTDIR)/opt/smartmower/data/vision 2>/dev/null || true
	# Install systemd service file
	install -d $(DESTDIR)/etc/systemd/system
	install -m 644 systemd/perimeter_detector.service $(DESTDIR)/etc/systemd/system/

# Uninstall target
uninstall:
	rm -f $(DESTDIR)$(BINDIR)/$(TARGET)
	rm -f $(DESTDIR)/etc/systemd/system/smartmower-perimeter.service

.PHONY: all clean install uninstall
