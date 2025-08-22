CXX = g++
CXXFLAGS = -std=c++17 -Wall -fext-numeric-literals \
           -Iinclude \
           -I../gps/include \
           -I../../camera/include \
           $(shell pkg-config --cflags libmosquitto) \
           -I/usr/include/nlohmann
LDFLAGS = $(shell pkg-config --libs libmosquitto) -ljsoncpp

# Directory di output per gli oggetti
OBJ_DIR = obj
BIN_DIR = bin

# File sorgente
SRCS = src/main.cpp \
       src/pico/pico_interface.cpp \
       src/pico/serial_interface.cpp \
       src/pico/pico_protocol.cpp \
       src/mqtt/mqtt_client.cpp \
       src/mqtt/mqtt_topics.cpp \
       src/config/config_manager.cpp

# File oggetto (stessa struttura delle sorgenti ma nella directory obj)
OBJS = $(patsubst src/%.cpp,$(OBJ_DIR)/%.o,$(SRCS))

# Nome dell'eseguibile
TARGET = $(BIN_DIR)/pico_bridge

# Regola predefinita
all: $(TARGET)

# Link dell'eseguibile
$(TARGET): $(OBJS)
	@mkdir -p $(BIN_DIR)
	$(CXX) -o $@ $^ $(LDFLAGS)

# Regola generica per compilare i file oggetto
$(OBJ_DIR)/%.o: src/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Pulizia
clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR)

# Installazione
install: $(TARGET)
	install -d $(DESTDIR)/opt/smartmower/bin
	install -m 755 $(TARGET) $(DESTDIR)/opt/smartmower/bin/

.PHONY: all clean install
