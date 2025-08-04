# Smart Mower System - Main Makefile
# Professional installation to /opt/smartmower/

# Installation directories
PREFIX ?= /opt/smartmower
BINDIR = $(PREFIX)/bin
ETCDIR = $(PREFIX)/etc
LOGDIR = $(PREFIX)/log
DATADIR = $(PREFIX)/data
SYSTEMDDIR = /etc/systemd/system

# User and group for services
SMARTMOWER_USER ?= smartmower
SMARTMOWER_GROUP ?= smartmower

# All modules to build and install
MODULES = pico gps fusion path_planning slam state_machine
VISION_MODULES = vision/camera vision/grass vision/obstacle vision/perimeter

.PHONY: all build install install-dirs install-config install-systemd install-web clean help

all: build

# Build all modules
build:
	@echo "=== Building Smart Mower System ==="
	@for module in $(MODULES); do \
		echo "Building $$module..."; \
		$(MAKE) -C $$module || exit 1; \
	done
	@for module in $(VISION_MODULES); do \
		echo "Building $$module..."; \
		$(MAKE) -C $$module || exit 1; \
	done
	@echo "=== Build Complete ==="

# Create installation directories
install-dirs:
	@echo "=== Creating Installation Directories ==="
	sudo mkdir -p $(BINDIR)
	sudo mkdir -p $(ETCDIR)
	sudo mkdir -p $(LOGDIR)
	sudo mkdir -p $(DATADIR)
	sudo mkdir -p $(DATADIR)/pico
	sudo mkdir -p $(DATADIR)/gps
	sudo mkdir -p $(DATADIR)/fusion
	sudo mkdir -p $(DATADIR)/path_planning
	sudo mkdir -p $(DATADIR)/slam
	sudo mkdir -p $(DATADIR)/state_machine
	sudo mkdir -p $(DATADIR)/vision_camera
	sudo mkdir -p $(DATADIR)/vision_grass
	sudo mkdir -p $(DATADIR)/vision_obstacle
	sudo mkdir -p $(DATADIR)/vision_perimeter
	@echo "=== Directories Created ==="

# Install configuration files
install-config: install-dirs
	@echo "=== Installing Configuration Files ==="
	sudo cp config/robot_config.json $(ETCDIR)/
	sudo chmod 644 $(ETCDIR)/robot_config.json
	@echo "=== Configuration Installed ==="

# Install systemd services
install-systemd:
	@echo "=== Installing Systemd Services ==="
	sudo cp config/systemd/*.service $(SYSTEMDDIR)/
	sudo systemctl daemon-reload
	@echo "=== Systemd Services Installed ==="

# Create system user and group
create-user:
	@echo "=== Creating Smart Mower System User ==="
	sudo groupadd -r $(SMARTMOWER_GROUP) 2>/dev/null || true
	sudo useradd -r -g $(SMARTMOWER_GROUP) -d $(PREFIX) -s /bin/false $(SMARTMOWER_USER) 2>/dev/null || true
	@echo "=== System User Created ==="

# Set proper ownership and permissions
set-permissions: create-user
	@echo "=== Setting Permissions ==="
	sudo chown -R $(SMARTMOWER_USER):$(SMARTMOWER_GROUP) $(PREFIX)
	sudo chmod -R 755 $(BINDIR)
	sudo chmod -R 644 $(ETCDIR)
	sudo chmod -R 755 $(LOGDIR)
	sudo chmod -R 755 $(DATADIR)
	@echo "=== Permissions Set ==="

# Install web interface
install-web:
	@echo "=== Installing Web Interface ==="
	# Install Python dependencies
	pip3 install -r web/requirements.txt
	# Install web application
	sudo cp web/app.py $(BINDIR)/web_app.py
	sudo mkdir -p $(BINDIR)/templates
	sudo cp -r web/templates/* $(BINDIR)/templates/
	sudo chmod +x $(BINDIR)/web_app.py
	@echo "=== Web Interface Installed ==="

# Full installation
install: build install-dirs install-config install-web set-permissions
	@echo "=== Installing Smart Mower System ==="
	@for module in $(MODULES); do \
		echo "Installing $$module..."; \
		$(MAKE) -C $$module install || exit 1; \
	done
	@for module in $(VISION_MODULES); do \
		echo "Installing $$module..."; \
		$(MAKE) -C $$module install || exit 1; \
	done
	$(MAKE) install-systemd
	@echo "=== Installation Complete ==="
	@echo ""
	@echo "Smart Mower System installed to $(PREFIX)"
	@echo "Configuration: $(ETCDIR)/robot_config.json"
	@echo "Logs: $(LOGDIR)/"
	@echo "Data: $(DATADIR)/"
	@echo ""
	@echo "To enable and start services:"
	@echo "  sudo systemctl enable smartmower-pico smartmower-gps smartmower-fusion"
	@echo "  sudo systemctl enable smartmower-path-planning smartmower-slam smartmower-state-machine"
	@echo "  sudo systemctl enable smartmower-vision-camera smartmower-vision-grass"
	@echo "  sudo systemctl enable smartmower-vision-obstacle smartmower-vision-perimeter"
	@echo "  sudo systemctl enable smartmower-web"
	@echo "  sudo systemctl start smartmower-state-machine smartmower-web"
	@echo ""
	@echo "Web interface will be available at: http://localhost:8080"

# Clean all modules
clean:
	@echo "=== Cleaning Smart Mower System ==="
	@for module in $(MODULES); do \
		echo "Cleaning $$module..."; \
		$(MAKE) -C $$module clean || true; \
	done
	@for module in $(VISION_MODULES); do \
		echo "Cleaning $$module..."; \
		$(MAKE) -C $$module clean || true; \
	done
	@echo "=== Clean Complete ==="

# Uninstall
uninstall:
	@echo "=== Uninstalling Smart Mower System ==="
	sudo systemctl stop smartmower-* 2>/dev/null || true
	sudo systemctl disable smartmower-* 2>/dev/null || true
	sudo rm -f $(SYSTEMDDIR)/smartmower-*.service
	sudo systemctl daemon-reload
	sudo rm -rf $(PREFIX)
	@echo "=== Uninstall Complete ==="

# Help
help:
	@echo "Smart Mower System - Available targets:"
	@echo "  build          - Build all modules"
	@echo "  install        - Full installation to $(PREFIX)"
	@echo "  install-dirs   - Create installation directories"
	@echo "  install-config - Install configuration files"
	@echo "  install-systemd- Install systemd services"
	@echo "  create-user    - Create system user and group"
	@echo "  set-permissions- Set proper ownership and permissions"
	@echo "  clean          - Clean all build artifacts"
	@echo "  uninstall      - Remove installation"
	@echo "  help           - Show this help"
