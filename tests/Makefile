# Makefile per tests MQTT (IMU/GPS/Camera)
# Regole: usare Makefile, binari in tests/bin, oggetti in tests/obj,
# installazione in /opt/smartmower/bin/tests

CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Wpedantic
LDFLAGS := 
LDLIBS_MQTT := -lmosquitto

# Rilevazione opzionale OpenCV per cam_publisher
OPENCV_CFLAGS := $(shell pkg-config --cflags opencv4 2>/dev/null)
OPENCV_LIBS := $(shell pkg-config --libs opencv4 2>/dev/null)
HAVE_OPENCV := $(if $(OPENCV_LIBS),1,0)

PREFIX := /opt/smartmower
TESTS_PREFIX := $(PREFIX)/bin/tests

SRC_DIR := mqtt
BIN_DIR := bin
OBJ_DIR := obj

VO_SRC := $(SRC_DIR)/vo_publisher.cpp
VOBS_SRC := $(SRC_DIR)/vobstacle_publisher.cpp
PDATA_SRC := $(SRC_DIR)/pico_data_publisher.cpp

IMU_SRC := $(SRC_DIR)/imu_publisher.cpp
GPS_SRC := $(SRC_DIR)/gps_publisher.cpp
CAM_SRC := $(SRC_DIR)/cam_publisher.cpp
FSMT_SRC := $(SRC_DIR)/fsm_tester.cpp
PSTAT_SRC := $(SRC_DIR)/pico_status_publisher.cpp

VO_OBJ := $(OBJ_DIR)/vo_publisher.o
VOBS_OBJ := $(OBJ_DIR)/vobstacle_publisher.o
PDATA_OBJ := $(OBJ_DIR)/pico_data_publisher.o

IMU_OBJ := $(OBJ_DIR)/imu_publisher.o
GPS_OBJ := $(OBJ_DIR)/gps_publisher.o
CAM_OBJ := $(OBJ_DIR)/cam_publisher.o
FSMT_OBJ := $(OBJ_DIR)/fsm_tester.o
PSTAT_OBJ := $(OBJ_DIR)/pico_status_publisher.o

VO_BIN := $(BIN_DIR)/vo_pub
VOBS_BIN := $(BIN_DIR)/vobstacle_pub
PDATA_BIN := $(BIN_DIR)/pico_data_pub

IMU_BIN := $(BIN_DIR)/imu_pub
GPS_BIN := $(BIN_DIR)/gps_pub
CAM_BIN := $(BIN_DIR)/cam_pub
FSMT_BIN := $(BIN_DIR)/fsm_tester
PSTAT_BIN := $(BIN_DIR)/pico_status_pub

ALL_TARGETS := $(IMU_BIN) $(GPS_BIN) $(FSMT_BIN) $(PSTAT_BIN) $(VO_BIN) $(VOBS_BIN) $(PDATA_BIN)
ifeq ($(HAVE_OPENCV),1)
ALL_TARGETS += $(CAM_BIN)
endif

.PHONY: all clean install uninstall info

all: info $(BIN_DIR) $(OBJ_DIR) $(ALL_TARGETS)

info:
	@echo "[tests] Build publishers: IMU/GPS/VO/VObstacle/PicoData (OpenCV per Camera: $(if $(HAVE_OPENCV),trovato,NON trovato))"

$(BIN_DIR):
	@mkdir -p $(BIN_DIR)

$(OBJ_DIR):
	@mkdir -p $(OBJ_DIR)

# IMU
$(IMU_OBJ): $(IMU_SRC)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(IMU_BIN): $(IMU_OBJ)
	$(CXX) $(LDFLAGS) $^ -o $@ $(LDLIBS_MQTT)

# GPS
$(GPS_OBJ): $(GPS_SRC)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(GPS_BIN): $(GPS_OBJ)
	$(CXX) $(LDFLAGS) $^ -o $@ $(LDLIBS_MQTT)

# FSM Tester
$(FSMT_OBJ): $(FSMT_SRC)
	$(CXX) $(CXXFLAGS) -I/usr/include/nlohmann -c $< -o $@

$(FSMT_BIN): $(FSMT_OBJ)
	$(CXX) $(LDFLAGS) $^ -o $@ $(LDLIBS_MQTT)

# Pico status publisher
$(PSTAT_OBJ): $(PSTAT_SRC)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(PSTAT_BIN): $(PSTAT_OBJ)
	$(CXX) $(LDFLAGS) $^ -o $@ $(LDLIBS_MQTT)

# VO (visual odometry)
$(VO_OBJ): $(VO_SRC)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(VO_BIN): $(VO_OBJ)
	$(CXX) $(LDFLAGS) $^ -o $@ $(LDLIBS_MQTT)

# Vision obstacle
$(VOBS_OBJ): $(VOBS_SRC)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(VOBS_BIN): $(VOBS_OBJ)
	$(CXX) $(LDFLAGS) $^ -o $@ $(LDLIBS_MQTT)

# Pico data
$(PDATA_OBJ): $(PDATA_SRC)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(PDATA_BIN): $(PDATA_OBJ)
	$(CXX) $(LDFLAGS) $^ -o $@ $(LDLIBS_MQTT)

# Camera (facoltativo se OpenCV presente)
ifeq ($(HAVE_OPENCV),1)
$(CAM_OBJ): $(CAM_SRC)
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) -c $< -o $@

$(CAM_BIN): $(CAM_OBJ)
	$(CXX) $(LDFLAGS) $^ -o $@ $(LDLIBS_MQTT) $(OPENCV_LIBS)
else
$(CAM_BIN):
	@echo "[tests] OpenCV non rilevato: salta cam_publisher. Installa opencv (libopencv-dev) per abilitarlo."
endif

clean:
	@rm -rf $(OBJ_DIR) $(BIN_DIR)

install: all
	@install -d $(TESTS_PREFIX)
	@install -m 0755 $(IMU_BIN) $(TESTS_PREFIX)/
	@install -m 0755 $(GPS_BIN) $(TESTS_PREFIX)/
	@install -m 0755 $(PSTAT_BIN) $(TESTS_PREFIX)/
	@install -m 0755 $(VO_BIN) $(TESTS_PREFIX)/
	@install -m 0755 $(VOBS_BIN) $(TESTS_PREFIX)/
	@install -m 0755 $(PDATA_BIN) $(TESTS_PREFIX)/
ifeq ($(HAVE_OPENCV),1)
	@install -m 0755 $(CAM_BIN) $(TESTS_PREFIX)/
endif
	@echo "[tests] Installati in $(TESTS_PREFIX)"

uninstall:
	@rm -f $(TESTS_PREFIX)/imu_pub $(TESTS_PREFIX)/gps_pub $(TESTS_PREFIX)/cam_pub $(TESTS_PREFIX)/pico_status_pub $(TESTS_PREFIX)/vo_pub $(TESTS_PREFIX)/vobstacle_pub $(TESTS_PREFIX)/pico_data_pub 2>/dev/null || true
	@echo "[tests] Rimossi binari da $(TESTS_PREFIX)"
