CXX = g++
CXXFLAGS = -std=c++17 -Wall \
           -Iinclude \
           -I../gps/include \
           -I../../camera/include \
           $(shell pkg-config --cflags libmosquitto) \
           -I/usr/include/nlohmann
LDFLAGS = $(shell pkg-config --libs libmosquitto) -ljsoncpp

# Directory di output per gli oggetti
OBJ_DIR = obj

# File sorgente
SRCS = src/main.cpp \
       src/pico/pico_interface.cpp \
       src/pico/serial_interface.cpp \
       src/mqtt/mqtt_client.cpp \
       src/mqtt/mqtt_topics.cpp \
       src/config/config_manager.cpp

# File oggetto (stessa struttura delle sorgenti ma nella directory obj)
OBJS = $(patsubst src/%.cpp,$(OBJ_DIR)/%.o,$(SRCS))

# Nome dell'eseguibile
TARGET = pico_bridge

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
	install -m 644 systemd/pico_bridge.service $(DESTDIR)/etc/systemd/system/

.PHONY: all clean install
