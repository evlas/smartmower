# Compiler and flags
CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O2
INCLUDE_DIRS := -Iinclude -I/usr/include/json-c -I/usr/include/eigen3
LDFLAGS := -lmosquitto -ljson-c -lpthread

# Local directories
SRCDIR := src
INCDIR := include
OBJDIR := obj
LOCALBINDIR := bin

# Source files
SRCS := $(wildcard $(SRCDIR)/*.cpp)
OBJS := $(patsubst $(SRCDIR)/%.cpp,$(OBJDIR)/%.o,$(SRCS))

# Output binary
TARGET := $(LOCALBINDIR)/slam_node

# Default target
all: directories $(TARGET)

directories:
	@mkdir -p $(OBJDIR) $(LOCALBINDIR)

# Build target
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $(INCLUDE_DIRS) -o $@ $^ $(LDFLAGS)

# Pattern rule for object files
$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDE_DIRS) -c -o $@ $<

# Dependencies
$(OBJDIR)/slam_node.o: $(SRCDIR)/slam_node.cpp include/slam_types.h include/mapping.h include/odometry.h include/gps.h
$(OBJDIR)/mapping.o: $(SRCDIR)/mapping.cpp include/mapping.h include/slam_types.h
$(OBJDIR)/odometry.o: $(SRCDIR)/odometry.cpp include/odometry.h include/slam_types.h
$(OBJDIR)/gps.o: $(SRCDIR)/gps.cpp include/gps.h include/slam_types.h
$(OBJDIR)/slam_utils.o: $(SRCDIR)/slam_utils.cpp include/slam_functions.h include/slam_types.h

# Clean build artifacts
clean:
	rm -rf $(OBJDIR) $(LOCALBINDIR)

# Install the binary and systemd service
install: $(TARGET)
	# Create directories
	install -d $(DESTDIR)/opt/smartmower/bin
	install -d $(DESTDIR)/opt/smartmower/log
	install -d $(DESTDIR)/opt/smartmower/data/slam/maps
	install -d $(DESTDIR)/etc/systemd/system
	
	# Install binary
	install -m 755 $(TARGET) $(DESTDIR)/opt/smartmower/bin/slam_node
	
	# Install systemd service
	install -m 644 systemd/slam.service $(DESTDIR)/etc/systemd/system/
	
	# Set ownership and permissions for directories
	chown -R smartmower:smartmower $(DESTDIR)/opt/smartmower/log
	chown -R smartmower:smartmower $(DESTDIR)/opt/smartmower/data
	chmod 755 $(DESTDIR)/opt/smartmower/log
	chmod 755 $(DESTDIR)/opt/smartmower/data
	chmod 755 $(DESTDIR)/opt/smartmower/data/slam
	chmod 755 $(DESTDIR)/opt/smartmower/data/slam/maps
	
	@echo "SLAM module installed successfully!"
	@echo "Enable service with: sudo systemctl enable smartmower-slam"
	@echo "Start service with: sudo systemctl start smartmower-slam"

.PHONY: all clean install
