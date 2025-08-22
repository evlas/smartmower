CXX=g++
CXXFLAGS=-std=c++17 -O2 -Wall -Wextra -Iinclude
LDFLAGS=-lmosquitto

BIN_DIR=bin
OBJ_DIR=obj
SRC=src/SupervisorNode.cpp \
    src/mqtt/mqtt_client.cpp
OBJS=$(SRC:src/%.cpp=$(OBJ_DIR)/%.o)
TARGET=$(BIN_DIR)/supervisor_node

all: $(TARGET)

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

$(OBJ_DIR):
	@mkdir -p $(OBJ_DIR)

$(OBJ_DIR)/%.o: src/%.cpp | $(OBJ_DIR)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(TARGET): $(BIN_DIR) $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) -o $(TARGET) $(LDFLAGS)

clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR)

install:
	install -d /opt/smartmower/bin
	install -m 0755 $(TARGET) /opt/smartmower/bin/

install-systemd:
	install -m 0644 systemd/smartmower-supervisor.service /etc/systemd/system/
	systemctl daemon-reload

.PHONY: all clean install install-systemd
