CXX = g++
CXXFLAGS = -std=c++17 -Wall -Iinclude $(shell pkg-config --cflags libmosquitto) -I/usr/include/nlohmann
LDFLAGS = $(shell pkg-config --libs libmosquitto) -lcjson

# Directory di output per gli oggetti
OBJ_DIR = obj

# File sorgente
SRCS = src/main.cpp \
       src/gps/serial_interface.cpp \
       src/gps/nmea_parser.cpp \
       src/mqtt/mqtt_client.cpp \
       src/mqtt/mqtt_topics.cpp \
       src/config/config_manager.cpp

# File oggetto per i moduli MQTT
MQTT_OBJ = $(OBJ_DIR)/mqtt/mqtt_client.o \
           $(OBJ_DIR)/mqtt/mqtt_topics.o

# File oggetto (stessa struttura delle sorgenti ma nella directory obj)
OBJS = $(patsubst src/%.cpp,$(OBJ_DIR)/%.o,$(SRCS))

# Nome dell'eseguibile
TARGET = gps_bridge

# Regola predefinita
all: $(TARGET)

# Link dell'eseguibile
$(TARGET): $(OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)

# Regola generica per compilare i file oggetto
$(OBJ_DIR)/%.o: src/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Pulizia
clean:
	rm -rf $(OBJ_DIR) $(TARGET)

# Installazione
install: $(TARGET)
	install -d $(DESTDIR)/opt/smartmower/bin
	install -m 755 $(TARGET) $(DESTDIR)/opt/smartmower/bin/
	install -d $(DESTDIR)/etc/systemd/system
	install -m 644 systemd/gps_bridge.service $(DESTDIR)/etc/systemd/system/

.PHONY: all clean install
