CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -pthread -I/usr/local/include
LDFLAGS = -pthread -lmosquitto -ljansson -lm

SRCDIR = .
INCDIR = .
OBJDIR = obj
BINDIR = bin

TARGET = $(BINDIR)/pico_gps_simulator
SOURCES = $(wildcard $(SRCDIR)/*.c)
OBJECTS = $(patsubst $(SRCDIR)/%.c,$(OBJDIR)/%.o,$(SOURCES))

# Installation directories
PREFIX ?= /opt/smartmower
INSTALLBINDIR = $(PREFIX)/bin

# Default target
all: directories $(TARGET)

# Create necessary directories
directories:
	@mkdir -p $(OBJDIR) $(BINDIR)

# Link the executable
$(TARGET): $(OBJECTS)
	$(CC) -o $@ $^ $(LDFLAGS)

# Compile source files
$(OBJDIR)/%.o: $(SRCDIR)/%.c
	$(CC) $(CFLAGS) -I$(INCDIR) -c $< -o $@

# Install the binary
install: $(TARGET)
	install -d $(DESTDIR)$(INSTALLBINDIR)
	install -m 755 $(TARGET) $(DESTDIR)$(INSTALLBINDIR)/

# Clean up build artifacts
clean:
	rm -rf $(OBJDIR) $(BINDIR)

.PHONY: all directories clean install
