# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O3 -Iinclude -I/usr/local/include -I/usr/include/eigen3 -I/usr/include/json-c -DJSON_C_VERSION=0x0d0e0a
LDFLAGS = -lmosquitto -ljson-c -lpthread -lm

# Add debug symbols in debug mode
ifdef DEBUG
    CXXFLAGS += -g -DDEBUG
endif

# Local directories
SRCDIR = src
INCDIR = include
OBJDIR = obj
LOCALBINDIR = bin

# Targets
TARGET = $(LOCALBINDIR)/fusion_sensor
SRC = $(SRCDIR)/fusion_sensor.cpp
OBJ = $(OBJDIR)/fusion_sensor.o

# Installation directories
PREFIX ?= /opt/smartmower
INSTALLBINDIR = $(PREFIX)/bin
SYSCONFDIR = $(PREFIX)/etc/config

# Default target
all: directories $(TARGET)

directories:
	@mkdir -p $(OBJDIR) $(LOCALBINDIR)

# Build target
$(TARGET): $(OBJ)
	$(CXX) -o $@ $^ $(LDFLAGS)

# Compile source files
$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# Clean build files
clean:
	rm -rf $(OBJDIR) $(LOCALBINDIR)

# Install target
install: $(TARGET)
	# Install binary
	install -d $(DESTDIR)$(INSTALLBINDIR)
	install -m 755 $(TARGET) $(DESTDIR)$(INSTALLBINDIR)/fusion_sensor
	# Install unified config if it doesn't exist
	if [ ! -f $(DESTDIR)$(SYSCONFDIR)/robot_config.json ]; then \
		install -d $(DESTDIR)$(SYSCONFDIR); \
		install -m 644 ../config/robot_config.json $(DESTDIR)$(SYSCONFDIR)/robot_config.json; \
	fi
	# Create log and data directories
	install -d $(DESTDIR)/opt/smartmower/log
	install -d $(DESTDIR)/var/lib/smartmower/fusion
	# Install systemd service file
	install -d $(DESTDIR)/etc/systemd/system
	install -m 644 systemd/fusion_sensor.service $(DESTDIR)/etc/systemd/system/

# Uninstall target
uninstall:
	rm -f $(DESTDIR)$(INSTALLBINDIR)/fusion_sensor
	# Note: Don't remove unified config as it's shared
