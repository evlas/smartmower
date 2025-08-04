# Root Makefile for SmartMower Project
# This Makefile coordinates building and installing all components

# Installation directories
PREFIX ?= /opt/smartmower
BINDIR ?= $(PREFIX)/bin
LIBDIR ?= $(PREFIX)/lib
INCLUDEDIR ?= $(PREFIX)/include
ETCDIR ?= $(PREFIX)/etc

# List of all component directories with Makefiles
COMPONENTS := \
	src/fusion \
	src/gps \
	src/pico \
	src/vision/camera \
	src/vision/grass \
	src/vision/obstacle \
	src/vision/perimeter \
	src/state_machine \
	src/slam \
	src/path_planning \
	src/lcd_interface

# Default target
all: build

# Build all components
build:
	@echo "Building all components..."
	@for dir in $(COMPONENTS); do \
		if [ -f "$$dir/Makefile" ]; then \
			echo "Building $$dir..."; \
			$(MAKE) -C $$dir || exit 1; \
		else \
			echo "Warning: No Makefile in $$dir"; \
		fi; \
	done

# Install all components
install: build
	@echo "Installing to $(PREFIX)..."
	# Create all necessary directories
	@mkdir -p $(BINDIR) $(LIBDIR) $(INCLUDEDIR) $(ETCDIR)
	@mkdir -p $(PREFIX)/etc/config
	@mkdir -p $(PREFIX)/log
	@mkdir -p $(PREFIX)/data
	@mkdir -p $(PREFIX)/data/slam/maps
	@mkdir -p $(PREFIX)/data/vision
	@mkdir -p $(PREFIX)/data/path_planning
	@for dir in $(COMPONENTS); do \
		if [ -f "$$dir/Makefile" ]; then \
			echo "Installing $$dir..."; \
			$(MAKE) -C $$dir install PREFIX=$(PREFIX) || exit 1; \
		fi; \
	done
	# Set ownership and permissions for all directories
	@echo "Setting permissions and ownership..."
	@chown -R smartmower:smartmower $(PREFIX)/log
	@chown -R smartmower:smartmower $(PREFIX)/data
	@chown -R smartmower:smartmower $(PREFIX)/etc/config
	@chmod 755 $(PREFIX)/log
	@chmod 755 $(PREFIX)/data
	@chmod 755 $(PREFIX)/etc/config
	@chmod -R 755 $(PREFIX)/data/slam
	@chmod -R 755 $(PREFIX)/data/vision
	@chmod -R 755 $(PREFIX)/data/path_planning
	@echo "Installation complete in $(PREFIX)"

# Clean all components
clean:
	@echo "Cleaning all components..."
	@for dir in $(COMPONENTS); do \
		if [ -f "$$dir/Makefile" ]; then \
			echo "Cleaning $$dir..."; \
			$(MAKE) -C $$dir clean || true; \
		fi; \
	done

# Uninstall all components
uninstall:
	@echo "Uninstalling from $(PREFIX)..."
	@for dir in $(COMPONENTS); do \
		if [ -f "$$dir/Makefile" ]; then \
			echo "Uninstalling $$dir..."; \
			$(MAKE) -C $$dir uninstall PREFIX=$(PREFIX) || true; \
		fi; \
	done
	@rm -rf $(PREFIX)
	@echo "Uninstallation complete"

.PHONY: all build install clean uninstall
