# SmartMower Web Admin - Makefile

CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra \
	-Iinclude \
	-I../src/supervisor/include
LDFLAGS := -lmicrohttpd -lmosquitto -pthread

SRC_DIR := src
OBJ_DIR := obj
BIN_DIR := bin
TARGET := $(BIN_DIR)/web_admin

SRCS := $(SRC_DIR)/web_admin.cpp
# Sorgenti esterni (fuori da $(SRC_DIR)) con oggetti dedicati
SUP_MQTT_SRC := ../src/supervisor/src/mqtt/mqtt_client.cpp
SUP_MQTT_OBJ := $(OBJ_DIR)/mqtt_client.o

OBJS := $(SRCS:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o) $(SUP_MQTT_OBJ)

.PHONY: all clean install

all: $(TARGET)

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Regola per compilare la sorgente esterna del supervisor
$(SUP_MQTT_OBJ): $(SUP_MQTT_SRC) | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(TARGET): $(OBJS) | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(OBJS) -o $(TARGET) $(LDFLAGS)

clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR)

install: all
	install -d /opt/smartmower/bin
	install -m 0755 $(TARGET) /opt/smartmower/bin/web_admin
	# User area (serve da /static)
	install -d /opt/smartmower/web/static
	cp -a static/. /opt/smartmower/web/static/
