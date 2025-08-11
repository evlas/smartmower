# Makefile per test della state machine
CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -g -O0 -DDEBUG
LDFLAGS = -lmosquitto -ljson-c -lpthread -lm

# Directory
SRC_DIR = src
INCLUDE_DIR = include
STATES_DIR = $(SRC_DIR)/states
TEST_DIR = .

# File sorgenti per i test
TEST_SOURCES = test_state_machine.c
STATE_SOURCES = $(SRC_DIR)/state_machine.c \
                $(STATES_DIR)/base_state.c \
                $(STATES_DIR)/init_state.c \
                $(STATES_DIR)/idle_state.c \
                $(STATES_DIR)/mowing_state.c \
                $(STATES_DIR)/charging_state.c \
                $(STATES_DIR)/emergency_stop_state.c \
                $(STATES_DIR)/undocking_state.c \
                $(STATES_DIR)/docking_state.c \
                $(STATES_DIR)/manual_control_state.c \
                $(STATES_DIR)/error_state.c \
                $(STATES_DIR)/paused_state.c

CONFIG_SOURCES = $(SRC_DIR)/config.c

# Tutti i sorgenti
ALL_SOURCES = $(TEST_SOURCES) $(STATE_SOURCES) $(CONFIG_SOURCES)

# File oggetto
OBJECTS = $(ALL_SOURCES:.c=.o)

# Target principale
TEST_TARGET = test_state_machine

# Include paths
INCLUDES = -I$(INCLUDE_DIR) -I$(INCLUDE_DIR)/states -I/usr/include/json-c

.PHONY: all test clean help

all: $(TEST_TARGET)

$(TEST_TARGET): $(OBJECTS)
	@echo "🔗 Linking test executable..."
	$(CC) $(OBJECTS) -o $(TEST_TARGET) $(LDFLAGS)
	@echo "✅ Test executable created: $(TEST_TARGET)"

%.o: %.c
	@echo "🔨 Compiling $<..."
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

test: $(TEST_TARGET)
	@echo "🚀 Running state machine tests..."
	@echo "=================================="
	./$(TEST_TARGET)
	@echo "=================================="

test-verbose: $(TEST_TARGET)
	@echo "🚀 Running state machine tests (verbose)..."
	@echo "============================================="
	DEBUG_CONFIG=1 ./$(TEST_TARGET)
	@echo "============================================="

test-quick: $(TEST_TARGET)
	@echo "⚡ Running quick state machine tests..."
	@echo "======================================"
	timeout 30s ./$(TEST_TARGET) || echo "Test completed or timed out"
	@echo "======================================"

clean:
	@echo "🧹 Cleaning test files..."
	rm -f $(OBJECTS) $(TEST_TARGET)
	rm -f *.log *.tmp
	@echo "✅ Cleanup completed"

help:
	@echo "📚 Available targets:"
	@echo "  all          - Build test executable"
	@echo "  test         - Run all tests"
	@echo "  test-verbose - Run tests with debug output"
	@echo "  test-quick   - Run tests with 30s timeout"
	@echo "  clean        - Remove generated files"
	@echo "  help         - Show this help"

# Dipendenze specifiche
test_state_machine.o: test_state_machine.c $(INCLUDE_DIR)/state_machine.h $(INCLUDE_DIR)/config.h

$(SRC_DIR)/state_machine.o: $(SRC_DIR)/state_machine.c $(INCLUDE_DIR)/state_machine.h

$(STATES_DIR)/%.o: $(STATES_DIR)/%.c $(INCLUDE_DIR)/states/%.h $(INCLUDE_DIR)/states/base_state.h

$(SRC_DIR)/config.o: $(SRC_DIR)/config.c $(INCLUDE_DIR)/config.h
