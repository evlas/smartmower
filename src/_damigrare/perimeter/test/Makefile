# Makefile per mqtt_viewer
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2
TARGET = mqtt_viewer
SOURCE = mqtt_viewer.cpp

# Librerie necessarie
LIBS = -lmosquitto -lcjson -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_highgui -lpthread

# Percorsi di include (potrebbero essere necessari su alcuni sistemi)
INCLUDES = -I/usr/include/opencv4

# Regola principale
$(TARGET): $(SOURCE)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $(TARGET) $(SOURCE) $(LIBS)

# Regola per pulire i file compilati
clean:
	rm -f $(TARGET)

# Regola per installare le dipendenze (Ubuntu/Debian)
install-deps:
	sudo apt-get update
	sudo apt-get install -y libmosquitto-dev libcjson-dev libopencv-dev pkg-config

# Regola per testare la compilazione
test: $(TARGET)
	@echo "Compilazione completata con successo!"
	@echo "Esegui con: ./$(TARGET)"

# Regola per debug
debug: CXXFLAGS += -g -DDEBUG
debug: $(TARGET)

.PHONY: clean install-deps test debug
