CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -pthread -Iinclude
LIBS = -lmosquitto -ljansson -lpthread

# Local directories
SRCDIR = src
INCDIR = include
OBJDIR = obj
LOCALBINDIR = bin

# Targets
TARGET = $(LOCALBINDIR)/pico_bridge
SOURCES = $(SRCDIR)/pico_bridge.c
OBJECTS = $(OBJDIR)/pico_bridge.o
# Configuration file (now centralized)
SERVICE = systemd/pico_bridge.service

# Installation directories
PREFIX ?= /opt/smartmower
INSTALLBINDIR = $(PREFIX)/bin
SYSDIR = /etc/systemd/system

all: directories $(TARGET)

directories:
	@mkdir -p $(OBJDIR) $(LOCALBINDIR)

$(TARGET): $(OBJECTS)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJECTS) $(LIBS)

$(OBJDIR)/%.o: $(SRCDIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(OBJDIR) $(LOCALBINDIR)

install: $(TARGET)
	# Install binary
	install -d $(DESTDIR)$(INSTALLBINDIR)
	install -m 755 $(TARGET) $(DESTDIR)$(INSTALLBINDIR)/pico_bridge
	
	# Create log directory
	install -d $(DESTDIR)/opt/smartmower/log
	
	# Install systemd service
	if [ -d $(DESTDIR)$(SYSDIR) ]; then \
		install -m 644 $(SERVICE) $(DESTDIR)$(SYSDIR)/; \
		systemctl daemon-reload; \
		echo "Service installed. Enable with: systemctl enable pico_bridge.service"; \
	fi

uninstall:
	rm -f $(DESTDIR)$(INSTALLBINDIR)/pico_bridge
	rm -f $(DESTDIR)$(SYSDIR)/pico_bridge.service
	systemctl daemon-reload

.PHONY: all clean install uninstall
