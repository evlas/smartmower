#include "gps/gps_interface.h"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdexcept>
#include <system_error>
#include <cstring>

class SerialGPS : public GPSInterface {
    int serial_fd_ = -1;
    std::string device_;
    int baud_rate_;
    
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
    
public:
    SerialGPS(const std::string& device, int baud_rate) 
        : device_(device), baud_rate_(baud_rate) {}
    
    ~SerialGPS() override {
        shutdown();
    }
    
    bool initialize() override {
        if (serial_fd_ >= 0) {
            return true; // Già inizializzato
        }
        
        // Apri la porta seriale
        serial_fd_ = open(device_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ < 0) {
            throw std::system_error(errno, std::system_category(), 
                                  "Impossibile aprire il dispositivo " + device_);
        }
        
        // Configura la porta seriale
        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) {
            close(serial_fd_);
            serial_fd_ = -1;
            throw std::system_error(errno, std::system_category(), 
                                  "Errore nella configurazione della porta seriale");
        }
        
        // Imposta i parametri della porta seriale
        cfsetospeed(&tty, translateBaudRate(baud_rate_));
        cfsetispeed(&tty, translateBaudRate(baud_rate_));
        
        tty.c_cflag &= ~PARENB; // No parity
        tty.c_cflag &= ~CSTOPB; // 1 stop bit
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;     // 8 bits per byte
        tty.c_cflag &= ~CRTSCTS; // No hardware flow control
        tty.c_cflag |= CREAD | CLOCAL; // Abilita lettura, ignora linee di controllo
        
        // Modalità non canonica
        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO; // Disabilita echo
        tty.c_lflag &= ~ECHOE;
        tty.c_lflag &= ~ECHONL;
        tty.c_lflag &= ~ISIG; // Disabilita interpretazione caratteri speciali
        
        // Disabilita l'elaborazione dell'input
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disabilita controllo flusso software
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
        
        // Disabilita l'elaborazione dell'output
        tty.c_oflag &= ~OPOST; // Elaborazione output non elaborata
        tty.c_oflag &= ~ONLCR;
        
        // Timeout di lettura: 100ms
        tty.c_cc[VTIME] = 1;
        tty.c_cc[VMIN] = 0;
        
        // Applica le impostazioni
        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            close(serial_fd_);
            serial_fd_ = -1;
            throw std::system_error(errno, std::system_category(), 
                                  "Errore nell'applicazione delle impostazioni della porta seriale");
        }
        
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
std::unique_ptr<GPSInterface> createGPSInterface(const std::string& device, int baud_rate) {
    return std::make_unique<SerialGPS>(device, baud_rate);
}
