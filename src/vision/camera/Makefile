CXX = g++
CXXFLAGS = -std=c++17 -Wall -Iinclude $(shell pkg-config --cflags opencv4 libmosquitto) -I/usr/include/libcamera
LDFLAGS = $(shell pkg-config --libs opencv4 libmosquitto libcamera) -lcjson -lstdc++fs

# Directory di output per gli oggetti
OBJ_DIR = obj
BIN_DIR = bin

SRCS = src/main.cpp \
       src/camera/usb_camera.cpp \
       src/camera/rpi_camera.cpp \
       src/camera/logging.cpp \
       src/mqtt/mqtt_client.cpp \
       src/mqtt/mqtt_topics.cpp \
       src/config/config_manager.cpp

OBJS = $(patsubst src/%.cpp,$(OBJ_DIR)/%.o,$(SRCS))
TARGET = $(BIN_DIR)/vision_camera

all: $(TARGET)

$(TARGET): $(OBJS)
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

# Crea la directory di output se non esiste
$(OBJ_DIR)/%.o: src/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR)

# Installazione
install: $(TARGET)
	install -d $(DESTDIR)/opt/smartmower/bin
	install -m 755 $(TARGET) $(DESTDIR)/opt/smartmower/bin/

.PHONY: all clean install
