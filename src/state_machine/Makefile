CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 \
           -Iinclude \
           -I/usr/include/nlohmann \
           -pthread
LDFLAGS = -pthread -lmosquitto

# Dir output
OBJ_DIR = obj
BIN_DIR = bin
TARGET = $(BIN_DIR)/state_machine

# Sorgenti
SRCS = \
	src/main.cpp \
	src/state_machine.cpp \
	src/mqtt/mqtt_client.cpp \
	src/config/config_manager.cpp \
	src/states/init_state.cpp \
	src/states/idle_state.cpp \
	src/states/manual_state.cpp \
	src/states/emergency_stop_state.cpp \
	src/states/mowing_state.cpp \
	src/states/paused_state.cpp \
	src/states/docking_state.cpp \
	src/states/undocking_state.cpp \
	src/states/charging_state.cpp \
	src/states/error_state.cpp

OBJS = $(patsubst src/%.cpp,$(OBJ_DIR)/%.o,$(SRCS))

all: $(TARGET)

$(TARGET): $(OBJS)
	@mkdir -p $(BIN_DIR)
	$(CXX) -o $@ $^ $(LDFLAGS)

$(OBJ_DIR)/%.o: src/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR)

install: $(TARGET)
	install -d $(DESTDIR)/opt/smartmower/bin
	install -m 755 $(TARGET) $(DESTDIR)/opt/smartmower/bin/

uninstall:
	rm -f $(DESTDIR)/opt/smartmower/bin/state_machine

.PHONY: all clean install uninstall
