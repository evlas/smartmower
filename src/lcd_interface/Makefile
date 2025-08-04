# Makefile for LCD Interface

# Installation directories
PREFIX ?= /opt/smartmower
BINDIR ?= $(PREFIX)/bin
ETCDIR ?= $(PREFIX)/etc
SYSTEMDDIR ?= /etc/systemd/system

# Source files
SOURCES = lcd_interface.py
TARGET = lcd_interface.py
SERVICE = systemd/smartmower-lcd.service

.PHONY: all install uninstall clean

all: $(TARGET)

$(TARGET): $(SOURCES)
	@echo "Checking LCD interface..."

install: all
	@echo "Installing LCD interface..."
	@install -d $(BINDIR)
	@install -m 0755 $(TARGET) $(BINDIR)/
	@install -d $(SYSTEMDDIR)
	@install -m 0644 $(SERVICE) $(SYSTEMDDIR)/
	@echo "Enabling smartmower-lcd service..."
	@systemctl daemon-reload
	@systemctl enable smartmower-lcd
	@echo "LCD interface installed to $(BINDIR)"

uninstall:
	@echo "Uninstalling LCD interface..."
	@rm -f $(BINDIR)/$(TARGET)
	@rm -f $(SYSTEMDDIR)/$(notdir $(SERVICE))
	@systemctl disable smartmower-lcd 2>/dev/null || true
	@echo "LCD interface uninstalled"

clean:
	@echo "Cleaning LCD interface..."
	@rm -f *.pyc *.pyo
	@echo "Clean complete"
