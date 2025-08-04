# Makefile for Smart Mower State Machine
# Compiler and flags
CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -O2 -pthread -I./include
LDFLAGS = -lmosquitto -ljson-c -lpthread -lm

# Local directories
OBJDIR = obj
LOCALBINDIR = bin

# Installation directories
PREFIX ?= /opt/smartmower
INSTALLBINDIR = $(PREFIX)/bin
SYSCONFDIR = $(PREFIX)/etc

# Target executable
TARGET = $(LOCALBINDIR)/state_machine

# Source files
SRCDIR = src
STATEDIR = $(SRCDIR)/states
SOURCES = $(SRCDIR)/state_machine.c \
          $(SRCDIR)/state_machine_main.c \
          $(SRCDIR)/config.c \
          $(STATEDIR)/base_state.c \
          $(STATEDIR)/init_state.c \
          $(STATEDIR)/idle_state.c \
          $(STATEDIR)/mowing_state.c \
          $(STATEDIR)/charging_state.c \
          $(STATEDIR)/emergency_stop_state.c \
          $(STATEDIR)/undocking_state.c \
          $(STATEDIR)/docking_state.c \
          $(STATEDIR)/manual_control_state.c \
          $(STATEDIR)/error_state.c \
          $(STATEDIR)/paused_state.c

# Object files
OBJECTS = $(patsubst $(SRCDIR)/%.c,$(OBJDIR)/%.o,$(filter $(SRCDIR)/%.c,$(SOURCES))) \
          $(patsubst $(STATEDIR)/%.c,$(OBJDIR)/states/%.o,$(filter $(STATEDIR)/%.c,$(SOURCES)))

# Configuration file
CONFIG = config.json

# Default target
all: directories $(TARGET)

directories:
	@mkdir -p $(OBJDIR) $(OBJDIR)/states $(LOCALBINDIR)

# Build the executable
$(TARGET): $(OBJECTS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

# Compile source files
$(OBJDIR)/%.o: $(SRCDIR)/%.c
	$(CC) $(CFLAGS) -c -o $@ $<

$(OBJDIR)/states/%.o: $(STATEDIR)/%.c
	$(CC) $(CFLAGS) -c -o $@ $<

# Clean build files
clean:
	rm -rf $(OBJDIR) $(LOCALBINDIR)

# Install target
install: $(TARGET)
	@echo "Installing State Machine..."
	# Install binary
	install -d $(DESTDIR)$(INSTALLBINDIR)
	install -m 755 $(TARGET) $(DESTDIR)$(INSTALLBINDIR)/state_machine
	# Create log and data directories
	install -d $(DESTDIR)$(PREFIX)/log
	install -d $(DESTDIR)$(PREFIX)/data/state_machine
	install -d $(DESTDIR)/etc/systemd/system
	# Install systemd service
	install -m 644 systemd/state_machine.service $(DESTDIR)/etc/systemd/system/
	# Set ownership and permissions for directories
	chown -R smartmower:smartmower $(DESTDIR)$(PREFIX)/log
	chown -R smartmower:smartmower $(DESTDIR)$(PREFIX)/data/state_machine
	chmod 755 $(DESTDIR)$(PREFIX)/log
	chmod 755 $(DESTDIR)$(PREFIX)/data/state_machine
	@echo "State Machine installed successfully!"
	@echo "Enable service with: sudo systemctl enable state_machine"
	@echo "Start service with: sudo systemctl start state_machine"

# Uninstall target
uninstall:
	rm -f $(DESTDIR)$(BINDIR)/$(TARGET)
	rm -f $(DESTDIR)$(SYSCONFDIR)/state_machine.json
	rm -f $(DESTDIR)/lib/systemd/system/$(TARGET).service

# Debug target
debug: CFLAGS += -g -DDEBUG
debug: $(TARGET)

# Test target (compila ma non esegue)
test: $(TARGET)
	@echo "Test di compilazione completato con successo"

# Help target
help:
	@echo "Targets disponibili:"
	@echo "  all      - Compila il programma (default)"
	@echo "  clean    - Rimuove i file oggetto e l'eseguibile"
	@echo "  install  - Installa il programma nel sistema"
	@echo "  uninstall- Rimuove il programma dal sistema"
	@echo "  debug    - Compila con simboli di debug"
	@echo "  test     - Test di compilazione"
	@echo "  help     - Mostra questo messaggio"

.PHONY: all clean install uninstall debug test help
