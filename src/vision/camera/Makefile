CXX = g++
CXXFLAGS = -std=c++17 -Wall -Iinclude $(shell pkg-config --cflags opencv4 libmosquitto) -I/usr/include/libcamera
LDFLAGS = $(shell pkg-config --libs opencv4 libmosquitto libcamera) -lcjson

# Directory di output per gli oggetti
OBJ_DIR = obj

SRCS = src/main.cpp \
       src/camera/usb_camera.cpp \
       src/camera/rpi_camera.cpp \
       src/mqtt/mqtt_client.cpp \
       src/mqtt/mqtt_topics.cpp \
       src/config/config_manager.cpp

OBJS = $(patsubst src/%.cpp,$(OBJ_DIR)/%.o,$(SRCS))
TARGET = vision_camera

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

# Crea la directory di output se non esiste
$(OBJ_DIR)/%.o: src/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean
