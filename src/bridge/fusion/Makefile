CXX = g++
# Assicurati che la directory include locale abbia la precedenza
CXXFLAGS = -std=c++17 -Wall \
           -I$(PWD)/include \
           -I$(PWD)/../gps/include \
           -I$(PWD)/../pico/include \
           $(shell pkg-config --cflags libmosquitto) \
           -I/usr/include/nlohmann \
           -I/usr/include/eigen3

LDFLAGS = $(shell pkg-config --libs libmosquitto) -ljsoncpp

# Directory di output per gli oggetti
OBJ_DIR = obj

# File sorgente
SRCS = src/main.cpp \
       src/fusion/ekf.cpp \
       src/fusion/sensor_fusion.cpp \
       src/fusion/state.cpp \
       src/mqtt/mqtt_client.cpp \
       src/mqtt/mqtt_topics.cpp \
       src/config/config_manager.cpp \
       src/config/fusion_config.cpp

# File oggetto (stessa struttura delle sorgenti ma nella directory obj)
OBJS = $(patsubst src/%.cpp,$(OBJ_DIR)/%.o,$(SRCS))

# Nome dell'eseguibile
TARGET = fusion_bridge

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
	install -m 644 systemd/$(TARGET).service $(DESTDIR)/etc/systemd/system/

.PHONY: all clean install
