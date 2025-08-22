# Makefile per il nodo SLAM

# Compilatore e flag
CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -Iinclude -I/usr/include/opencv4 -I/usr/include/eigen3
LDFLAGS := -lmosquitto -ljsoncpp -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_features2d -lopencv_calib3d -lopencv_imgcodecs -lopencv_videoio -lopencv_video

# Directory
SRC_DIR := src
BUILD_DIR := obj
BIN_DIR := bin

# File sorgenti
SRCS := $(shell find $(SRC_DIR) -name '*.cpp')
OBJS := $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(SRCS))

# Eseguibile
TARGET := $(BIN_DIR)/slam_node

# Regole
.PHONY: all clean install

all: $(TARGET)

$(TARGET): $(OBJS)
	@mkdir -p $(@D)
	$(CXX) $^ -o $@ $(LDFLAGS)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR) $(BIN_DIR)

install: $(TARGET)
	install -d $(DESTDIR)/opt/smartmower/bin
	install -m 755 $(TARGET) $(DESTDIR)/opt/smartmower/bin/
