#include "pico/serial_interface.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include <iostream>
#include <chrono>

namespace pico {

SerialInterface::SerialInterface(const std::string& device, int baudrate, int timeout_ms)
    : device_(device), baudrate_(baudrate), timeout_ms_(timeout_ms), fd_(-1), running_(false) {}

SerialInterface::~SerialInterface() {
    close();
}

bool SerialInterface::open() {
    fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ == -1) {
        std::cerr << "Errore nell'apertura del dispositivo " << device_ << std::endl;
        return false;
    }
    
    // Configura la porta seriale
    struct termios options;
    tcgetattr(fd_, &options);
    
    // Imposta la velocità in baud
    cfsetispeed(&options, baudrate_);
    cfsetospeed(&options, baudrate_);
    
    // Imposta i parametri di base
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;         // 8-bit characters
    options.c_cflag &= ~PARENB;     // No parity
    options.c_cflag &= ~CSTOPB;     // 1 stop bit
    options.c_cflag &= ~CRTSCTS;    // No hardware flow control
    
    // Configura il timeout di lettura
    options.c_cc[VMIN] = 0;  // Non bloccante
    options.c_cc[VTIME] = timeout_ms_ / 100;  // Timeout in decimi di secondo
    
    // Configura i parametri della porta
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB; // No parity
    options.c_cflag &= ~CSTOPB; // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;     // 8 bit
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw mode
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // No software flow control
    
    // Applica le impostazioni
    tcsetattr(fd_, TCSANOW, &options);
    
    // Avvia il thread di lettura
    running_ = true;
    std::thread(&SerialInterface::readThread, this).detach();
    
    return true;
}

void SerialInterface::close() {
    running_ = false;
    if (fd_ != -1) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SerialInterface::isOpen() const {
    return (fd_ != -1);
}

ssize_t SerialInterface::write(const uint8_t* data, size_t size) {
    if (fd_ == -1) return -1;
    return ::write(fd_, data, size);
}

ssize_t SerialInterface::read(uint8_t* buffer, size_t size) {
    if (fd_ == -1) return -1;
    return ::read(fd_, buffer, size);
}

void SerialInterface::setDataCallback(DataCallback callback) {
    data_callback_ = callback;
}

void SerialInterface::readThread() {
    std::vector<uint8_t> buffer(BUFFER_SIZE);
    
    while (running_) {
        ssize_t bytes_read = read(buffer.data(), buffer.size());
        if (bytes_read > 0 && data_callback_) {
            std::vector<uint8_t> data(buffer.begin(), buffer.begin() + bytes_read);
            data_callback_(data);
        } else if (bytes_read < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

} // namespace pico
