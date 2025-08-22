# Makefile per Map Server (C++)

APP_NAME := map_server
TARGET_DIR := bin
OBJ_DIR := obj
SRC_DIR := src
INC_DIR := include
PREFIX := /opt/smartmower/bin

CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -I$(INC_DIR)
LDFLAGS := -lmosquitto

SOURCES := $(wildcard $(SRC_DIR)/*.cpp)
OBJECTS := $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SOURCES))
BINARY := $(TARGET_DIR)/$(APP_NAME)

.PHONY: all clean install dirs

all: dirs $(BINARY)

$(BINARY): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | dirs
	$(CXX) $(CXXFLAGS) -c $< -o $@

dirs:
	@mkdir -p $(TARGET_DIR) $(OBJ_DIR)

install: all
	install -d $(PREFIX)
	install -m 0755 $(BINARY) $(PREFIX)/$(APP_NAME)

clean:
	rm -rf $(OBJ_DIR) $(TARGET_DIR)
