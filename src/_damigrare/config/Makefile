# Smart Mower Configuration Module Makefile
# Manages installation of robot_config.json and parameter services

# Installation directories
PREFIX ?= /opt/smartmower
CONFDIR = $(PREFIX)/etc/config
BINDIR = $(PREFIX)/bin
SYSDIR = /etc/systemd/system
LOGDIR = $(PREFIX)/log

# Source files
CONFIG_FILE = robot_config.json
PYTHON_SERVICES = parameter_service.py mqtt_parameter_client.py parameter_manager.py
SYSTEMD_SERVICES = systemd/parameters.service
REQUIREMENTS = requirements.txt

# User for services
SERVICE_USER = smartmower

all:
	@echo "Configuration module - no compilation needed"
	@echo "Available targets: install, uninstall, clean"

# Create smartmower user if it doesn't exist
create-user:
	@if ! id "$(SERVICE_USER)" &>/dev/null; then \
		echo "Creating $(SERVICE_USER) user..."; \
		sudo useradd -r -s /bin/false $(SERVICE_USER) || true; \
	else \
		echo "User $(SERVICE_USER) already exists"; \
	fi

# Install Python dependencies
install-deps:
	@echo "Installing Python dependencies..."
	@if command -v apt >/dev/null 2>&1; then \
		echo "Debian/Ubuntu detected, using apt..."; \
		sudo apt update && sudo apt install -y python3-paho-mqtt; \
	else \
		echo "Using pip..."; \
		pip3 install --user -r $(REQUIREMENTS); \
	fi

# Install configuration and services
install: create-user install-deps
	@echo "Installing Smart Mower Configuration Module..."
	
	# Create directories
	sudo mkdir -p $(DESTDIR)$(CONFDIR)
	sudo mkdir -p $(DESTDIR)$(BINDIR)
	sudo mkdir -p $(DESTDIR)$(LOGDIR)
	
	# Install robot configuration file
	sudo install -m 644 -o $(SERVICE_USER) -g $(SERVICE_USER) $(CONFIG_FILE) $(DESTDIR)$(CONFDIR)/$(CONFIG_FILE)
	
	# Install Python services
	sudo install -m 755 -o $(SERVICE_USER) -g $(SERVICE_USER) parameter_service.py $(DESTDIR)$(BINDIR)/
	sudo install -m 755 -o $(SERVICE_USER) -g $(SERVICE_USER) mqtt_parameter_client.py $(DESTDIR)$(BINDIR)/
	sudo install -m 644 -o $(SERVICE_USER) -g $(SERVICE_USER) parameter_manager.py $(DESTDIR)$(BINDIR)/
	
	# Set log directory permissions
	sudo chown $(SERVICE_USER):$(SERVICE_USER) $(DESTDIR)$(LOGDIR)
	sudo chmod 755 $(DESTDIR)$(LOGDIR)
	
	# Install systemd service
	sudo install -m 644 $(SYSTEMD_SERVICES) $(DESTDIR)$(SYSDIR)/
	sudo systemctl daemon-reload
	
	@echo "Configuration module installed successfully!"
	@echo "Enable service with: sudo systemctl enable smartmower-parameters.service"
	@echo "Start service with: sudo systemctl start smartmower-parameters.service"

# Enable and start the parameter service
enable:
	sudo systemctl enable smartmower-parameters.service
	sudo systemctl start smartmower-parameters.service
	sudo systemctl status smartmower-parameters.service --no-pager

# Stop and disable the parameter service
disable:
	-sudo systemctl stop smartmower-parameters.service
	-sudo systemctl disable smartmower-parameters.service

# Uninstall everything
uninstall: disable
	@echo "Uninstalling Smart Mower Configuration Module..."
	
	# Remove installed files
	sudo rm -f $(DESTDIR)$(CONFDIR)/$(CONFIG_FILE)
	sudo rm -f $(DESTDIR)$(BINDIR)/parameter_service.py
	sudo rm -f $(DESTDIR)$(BINDIR)/mqtt_parameter_client.py
	sudo rm -f $(DESTDIR)$(BINDIR)/parameter_manager.py
	sudo rm -f $(DESTDIR)$(SYSDIR)/smartmower-parameters.service
	
	# Reload systemd
	sudo systemctl daemon-reload
	
	@echo "Configuration module uninstalled successfully!"
	@echo "Note: User '$(SERVICE_USER)' and log directory were not removed"

# Clean local build artifacts (none for this module)
clean:
	@echo "Cleaning configuration module..."
	rm -rf __pycache__/
	find . -name "*.pyc" -delete
	find . -name "*.pyo" -delete

# Show service status
status:
	sudo systemctl status smartmower-parameters.service --no-pager

# Show service logs
logs:
	sudo journalctl -u smartmower-parameters.service -f

# Backup current configuration
backup:
	@if [ -f $(DESTDIR)$(CONFDIR)/$(CONFIG_FILE) ]; then \
		sudo cp $(DESTDIR)$(CONFDIR)/$(CONFIG_FILE) $(DESTDIR)$(CONFDIR)/$(CONFIG_FILE).backup.$(shell date +%Y%m%d_%H%M%S); \
		echo "Configuration backed up"; \
	else \
		echo "No configuration file to backup"; \
	fi

# Restore configuration from local file
restore:
	@if [ -f $(CONFIG_FILE) ]; then \
		sudo install -m 644 -o $(SERVICE_USER) -g $(SERVICE_USER) $(CONFIG_FILE) $(DESTDIR)$(CONFDIR)/$(CONFIG_FILE); \
		echo "Configuration restored from local file"; \
		sudo systemctl restart smartmower-parameters.service; \
	else \
		echo "No local configuration file to restore"; \
	fi

.PHONY: all install uninstall clean create-user install-deps enable disable status logs backup restore
