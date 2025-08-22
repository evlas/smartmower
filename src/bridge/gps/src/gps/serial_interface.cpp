#include "gps/gps_interface.h"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdexcept>
#include <system_error>
#include <cstring>
#include <iostream>
#include <string>

class SerialGPS : public GPSInterface {
    int serial_fd_ = -1;
    std::string device_;
    int baud_rate_;
    int timeout_ms_;
    int max_satellites_;
    
    int translateBaudRate(int baud) {
        switch(baud) {
            case 50: return B50;
            case 75: return B75;
            case 110: return B110;
            case 134: return B134;
            case 150: return B150;
            case 200: return B200;
            case 300: return B300;
            case 600: return B600;
            case 1200: return B1200;
            case 1800: return B1800;
            case 2400: return B2400;
            case 4800: return B4800;
            case 9600: return B9600;
            case 19200: return B19200;
            case 38400: return B38400;
            case 57600: return B57600;
            case 115200: return B115200;
            case 230400: return B230400;
            default: return -1;
        }
    }
    
    bool configureSerialPort() {
        struct termios tty;
        
        // Get current serial port settings
        if (tcgetattr(serial_fd_, &tty) != 0) {
            std::cerr << "Errore nella configurazione della porta seriale: " << strerror(errno) << std::endl;
            return false;
        }
        
        // Set baud rate
        int baud = translateBaudRate(baud_rate_);
        if (baud == -1) {
            std::cerr << "Baud rate non supportato: " << baud_rate_ << std::endl;
            return false;
        }
        
        cfsetospeed(&tty, baud);
        cfsetispeed(&tty, baud);
        
        // Set port parameters
        tty.c_cflag &= ~PARENB;  // No parity
        tty.c_cflag &= ~CSTOPB;  // 1 stop bit
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;      // 8 bits per byte
        tty.c_cflag &= ~CRTSCTS; // No hardware flow control
        tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore control lines
        
        // Non-canonical mode
        tty.c_lflag &= ~ICANON;
        // Disable special character interpretation
        tty.c_lflag &= ~(ECHO | ECHOE | ECHONL | IEXTEN | ISIG);
        
        // Disable input processing
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        
        // Disable output processing
        tty.c_oflag &= ~OPOST;
        tty.c_oflag &= ~ONLCR;
        
        // Set read timeout
        tty.c_cc[VTIME] = timeout_ms_ / 100;  // Timeout in tenths of seconds
        tty.c_cc[VMIN]  = 0;                  // Return immediately even with 0 bytes
        
        // Apply settings
        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            std::cerr << "Errore nell'applicazione delle impostazioni seriali: " << strerror(errno) << std::endl;
            return false;
        }
        
        // Flush input and output buffers
        tcflush(serial_fd_, TCIOFLUSH);
        
        return true;
    }
    
public:
    SerialGPS(const std::string& device, int baud_rate, int timeout_ms, int max_satellites)
        : device_(device), baud_rate_(baud_rate), 
          timeout_ms_(timeout_ms), max_satellites_(max_satellites) {}
    
    ~SerialGPS() override {
        shutdown();
    }
    
    bool initialize() override {
        // Se la porta è già aperta, riconfigurala
        if (serial_fd_ >= 0) {
            return configureSerialPort();
        }
        
        // Apri il dispositivo seriale
        serial_fd_ = open(device_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ < 0) {
            std::cerr << "Errore nell'apertura del dispositivo " << device_ << ": " << strerror(errno) << std::endl;
            return false;
        }
        
        // Configura la porta seriale
        if (!configureSerialPort()) {
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }
        
        std::cout << "Porta seriale " << device_ << " configurata con successo (" 
                  << baud_rate_ << " baud, timeout: " << timeout_ms_ << " ms)" << std::endl;
        
        return true;
    }
    
    std::string readData() override {
        if (serial_fd_ < 0) {
            return "";
        }
        
        char buffer[1024];
        ssize_t n = read(serial_fd_, buffer, sizeof(buffer) - 1);
        
        if (n > 0) {
            buffer[n] = '\0';
            return std::string(buffer, n);
        }
        
        return "";
    }
    
    bool isConnected() const override {
        return (serial_fd_ >= 0);
    }
    
    void shutdown() override {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
            serial_fd_ = -1;
        }
    }
};

// Factory function
std::unique_ptr<GPSInterface> createGPSInterface(
    const std::string& device, 
    int baud_rate,
    int timeout_ms,
    int max_satellites
) {
    return std::make_unique<SerialGPS>(device, baud_rate, timeout_ms, max_satellites);
}
