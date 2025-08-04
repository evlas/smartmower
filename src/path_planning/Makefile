# Makefile for Path Planning Module
# Smart Mower - 6000mq Area with Obstacle Avoidance

CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -O2 -g
INCLUDES = -I./include -I../common
LIBS = -lmosquitto -ljson-c -lpthread -lm

# Directories
SRCDIR = src
INCDIR = include
OBJDIR = obj
BINDIR = bin

# Source files
SOURCES = $(wildcard $(SRCDIR)/*.c)
OBJECTS = $(SOURCES:$(SRCDIR)/%.c=$(OBJDIR)/%.o)

# Target executable
TARGET = $(BINDIR)/path_planning_node

# Default target
all: directories $(TARGET)

# Create necessary directories
directories:
	@mkdir -p $(OBJDIR) $(BINDIR)

# Main target
$(TARGET): $(OBJECTS)
	@echo "Linking path planning node..."
	$(CC) $(OBJECTS) -o $@ $(LIBS)
	@echo "Path planning node built successfully!"

# Object files
$(OBJDIR)/%.o: $(SRCDIR)/%.c
	@echo "Compiling $<..."
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Pattern-specific targets for testing
spiral: $(TARGET)
	@echo "Testing spiral pattern generation..."
	./$(TARGET) --test-pattern spiral

linear: $(TARGET)
	@echo "Testing linear pattern generation..."
	./$(TARGET) --test-pattern linear

random: $(TARGET)
	@echo "Testing random pattern generation..."
	./$(TARGET) --test-pattern random

adaptive: $(TARGET)
	@echo "Testing adaptive ML pattern generation..."
	./$(TARGET) --test-pattern adaptive

all-patterns: $(TARGET)
	@echo "Testing all pattern algorithms..."
	./$(TARGET) --test-all-patterns

# Clean build artifacts
clean:
	@echo "Cleaning build artifacts..."
	rm -rf $(OBJDIR) $(BINDIR)

# Installation directories
PREFIX ?= /opt/smartmower
INSTALLBINDIR = $(PREFIX)/bin
SYSCONFDIR = $(PREFIX)/etc/config

# Install target
install: $(TARGET)
	@echo "Installing path planning node..."
	# Create directories
	install -d $(DESTDIR)$(INSTALLBINDIR)
	install -d $(DESTDIR)$(PREFIX)/log
	install -d $(DESTDIR)$(PREFIX)/data/path_planning
	install -d $(DESTDIR)/etc/systemd/system
	# Install binary
	install -m 755 $(TARGET) $(DESTDIR)$(INSTALLBINDIR)/path_planning_node
	# Install systemd service if it exists
	if [ -f systemd/path_planning_node.service ]; then \
		install -m 644 systemd/path_planning_node.service $(DESTDIR)/etc/systemd/system/; \
	fi
	# Set ownership and permissions for directories
	chown -R smartmower:smartmower $(DESTDIR)$(PREFIX)/log
	chown -R smartmower:smartmower $(DESTDIR)$(PREFIX)/data/path_planning
	chmod 755 $(DESTDIR)$(PREFIX)/log
	chmod 755 $(DESTDIR)$(PREFIX)/data/path_planning
	@echo "Path planning node installed successfully!"
	@echo "Uses unified config: $(SYSCONFDIR)/robot_config.json"
	@echo "Enable service with: sudo systemctl enable smartmower-path-planning"
	@echo "Start service with: sudo systemctl start smartmower-path-planning"

# Development targets
debug: CFLAGS += -DDEBUG -g3
debug: $(TARGET)

release: CFLAGS += -DNDEBUG -O3
release: clean $(TARGET)

# Test target
test: $(TARGET)
	@echo "Running path planning tests..."
	./$(TARGET) --test-mode

# Documentation
docs:
	@echo "Generating documentation..."
	doxygen Doxyfile

# Static analysis
analyze:
	@echo "Running static analysis..."
	cppcheck --enable=all --std=c99 $(SRCDIR)/

# Memory check
memcheck: $(TARGET)
	@echo "Running memory check..."
	valgrind --leak-check=full --show-leak-kinds=all ./$(TARGET) --test-mode

# Performance profiling
profile: CFLAGS += -pg
profile: $(TARGET)
	@echo "Building with profiling enabled..."

# Help target
help:
	@echo "Available targets:"
	@echo "  all      - Build the path planning node (default)"
	@echo "  clean    - Remove build artifacts"
	@echo "  install  - Install to system path"
	@echo "  debug    - Build with debug symbols"
	@echo "  release  - Build optimized release version"
	@echo "  test     - Run test suite"
	@echo "  spiral   - Test spiral pattern generation"
	@echo "  linear   - Test linear pattern generation"
	@echo "  docs     - Generate documentation"
	@echo "  analyze  - Run static code analysis"
	@echo "  memcheck - Run memory leak detection"
	@echo "  profile  - Build with profiling support"
	@echo "  help     - Show this help message"

.PHONY: all clean install debug release test spiral linear docs analyze memcheck profile help directories
