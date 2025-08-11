CC = gcc
CFLAGS = -Wall -Wextra -O2 -Iinclude
LDFLAGS = -lmosquitto -ljansson -lrt -lpthread

# Local directories
SRCDIR = src
INCDIR = include
OBJDIR = obj
LOCALBINDIR = bin

# Targets
TARGET = $(LOCALBINDIR)/gps_bridge
SRC = $(SRCDIR)/gps_bridge.c
OBJ = $(OBJDIR)/gps_bridge.o
SERVICE = systemd/gps_bridge.service

# Installation directories
PREFIX ?= /opt/smartmower
INSTALLBINDIR = $(PREFIX)/bin
SYSDIR = /etc/systemd/system

all: directories $(TARGET)

directories:
	@mkdir -p $(OBJDIR) $(LOCALBINDIR)

$(TARGET): $(OBJ)
	$(CC) -o $@ $^ $(LDFLAGS)

$(OBJDIR)/%.o: $(SRCDIR)/%.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -rf $(OBJDIR) $(LOCALBINDIR)

install: $(TARGET)
	# Create directories
	install -d $(DESTDIR)$(INSTALLBINDIR)
	install -d $(DESTDIR)/opt/smartmower/etc/config
	install -d $(DESTDIR)/opt/smartmower/log
	
	# Install binary
	install -m 755 $(TARGET) $(DESTDIR)$(INSTALLBINDIR)/gps_bridge
	
	# Install unified config file
	install -m 644 ../config/robot_config.json $(DESTDIR)/opt/smartmower/etc/config/
	
	# Set ownership and permissions for directories
	chown -R smartmower:smartmower $(DESTDIR)/opt/smartmower/log
	chown -R smartmower:smartmower $(DESTDIR)/opt/smartmower/etc/config
	chmod 755 $(DESTDIR)/opt/smartmower/log
	chmod 755 $(DESTDIR)/opt/smartmower/etc/config
	chmod 644 $(DESTDIR)/opt/smartmower/etc/config/robot_config.json
	
	# Install systemd service
	if [ -d $(DESTDIR)$(SYSDIR) ]; then \
		install -m 644 $(SERVICE) $(DESTDIR)$(SYSDIR)/; \
		systemctl daemon-reload; \
		echo "Service installed. Enable with: systemctl enable $(SERVICE)"; \
	fi

uninstall:
	rm -f $(DESTDIR)$(INSTALLBINDIR)/gps_bridge
	rm -f $(DESTDIR)$(SYSDIR)/$(SERVICE)
	systemctl daemon-reload

.PHONY: all clean install uninstall
