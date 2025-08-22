# Makefile for Safety Supervisor module

PREFIX ?= /opt/smartmower
BINDIR ?= $(PREFIX)/bin

CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O2 -Iinclude -I/usr/include/nlohmann -pthread
LDFLAGS := -lmosquitto -pthread

SRC_DIR := src
OBJ_DIR := obj
BIN_DIR := bin

SRCS := \
	$(SRC_DIR)/safety_supervisor.cpp \
	$(SRC_DIR)/mqtt/mqtt_client.cpp \
	$(SRC_DIR)/config/config_manager.cpp

OBJS := $(SRCS:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)

TARGET := $(BIN_DIR)/safety_supervisor

all: dirs $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

.PHONY: clean install uninstall dirs

clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR)

install: all
	@mkdir -p $(BINDIR)
	install -m 755 $(TARGET) $(BINDIR)/

uninstall:
	rm -f $(BINDIR)/safety_supervisor

dirs:
	@mkdir -p $(OBJ_DIR) $(BIN_DIR)
