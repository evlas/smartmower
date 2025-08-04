# Compiler and flags
CXX = g++
CXXFLAGS = -Wall -Wextra -O2 -std=c++17 -D_GNU_SOURCE -Iinclude

# Directory structure
SRCDIR = src
INCDIR = include
OBJDIR = obj
BINDIR = bin
SYSTEMDDIR = systemd

# Get OpenCV flags using pkg-config
PKG_CONFIG = pkg-config
OPENCV_CFLAGS = $(shell $(PKG_CONFIG) --cflags opencv4 2>/dev/null || $(PKG_CONFIG) --cflags opencv)
OPENCV_LIBS = $(shell $(PKG_CONFIG) --libs opencv4 2>/dev/null || $(PKG_CONFIG) --libs opencv)

# Get JsonCpp flags
JSONCPP_CFLAGS = $(shell $(PKG_CONFIG) --cflags jsoncpp 2>/dev/null || echo "-I/usr/include/jsoncpp")
JSONCPP_LIBS = $(shell $(PKG_CONFIG) --libs jsoncpp 2>/dev/null || echo "-ljsoncpp")

# Additional libraries and flags
CXXFLAGS += $(JSONCPP_CFLAGS) -I/usr/include/jsoncpp
LDFLAGS = $(OPENCV_LIBS) $(JSONCPP_LIBS) -lmosquitto -lpthread -lstdc++fs

# Configuration (unified)
CONFIG = /opt/smartmower/etc/config/robot_config.json

# Installation directories
PREFIX ?= /opt/smartmower
INSTALL_BINDIR = $(PREFIX)/bin
SYSCONFDIR = $(PREFIX)/etc

# Targets
TARGETS = $(BINDIR)/vision_camera_rpi

# Source files
SOURCES = $(wildcard $(SRCDIR)/*.cpp)
OBJECTS = $(SOURCES:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)

# Default target
all: $(TARGETS)

# Create directories
$(OBJDIR) $(BINDIR):
	mkdir -p $@

# Build the executables
$(BINDIR)/vision_camera: $(OBJDIR)/vision_camera.o | $(BINDIR)
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS)

$(BINDIR)/vision_camera_rpi: $(OBJDIR)/vision_camera_rpi.o | $(BINDIR)
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS)

# Compile source files
$(OBJDIR)/%.o: $(SRCDIR)/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) $(JSONCPP_CFLAGS) -c -o $@ $<

# Clean build files
clean:
	rm -rf $(OBJDIR) $(BINDIR)

# Install targets
install: $(TARGETS)
	# Install binaries
	install -d $(DESTDIR)$(INSTALL_BINDIR)
	for target in $(TARGETS); do \
		install -m 755 $$target $(DESTDIR)$(INSTALL_BINDIR)/; \
	done
	# Create directories with proper permissions
	install -d $(DESTDIR)/opt/smartmower/log
	install -d $(DESTDIR)/opt/smartmower/data/vision
	install -d $(DESTDIR)/opt/smartmower/etc/config
	# Set ownership and permissions
	chown -R smartmower:smartmower $(DESTDIR)/opt/smartmower/log $(DESTDIR)/opt/smartmower/data/vision 2>/dev/null || true
	chmod -R 755 $(DESTDIR)/opt/smartmower/log $(DESTDIR)/opt/smartmower/data/vision 2>/dev/null || true
	# Install systemd service files
	install -d $(DESTDIR)/etc/systemd/system
	install -m 644 $(SYSTEMDDIR)/*.service $(DESTDIR)/etc/systemd/system/

systemd-install:
	# Install systemd service files
	cp $(SYSTEMDDIR)/*.service /etc/systemd/system/
	systemctl daemon-reload
	systemctl enable vision-camera.service

systemd-uninstall:
	# Remove systemd service files
	systemctl stop vision-camera.service
	systemctl disable vision-camera.service
	rm -f /etc/systemd/system/vision-camera.service
	systemctl daemon-reload

uninstall:
	for target in $(TARGETS); do \
		rm -f $(DESTDIR)$(INSTALL_BINDIR)/$$target; \
	done

.PHONY: all clean install uninstall
