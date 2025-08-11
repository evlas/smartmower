#ifndef SERIAL_INTERFACE_H
#define SERIAL_INTERFACE_H

#include <string>
#include <functional>
#include <vector>

namespace pico {

class SerialInterface {
public:
    using DataCallback = std::function<void(const std::vector<uint8_t>&)>;
    
    SerialInterface(const std::string& device, int baudrate);
    ~SerialInterface();
    
    bool open();
    void close();
    bool isOpen() const;
    
    ssize_t write(const uint8_t* data, size_t size);
    ssize_t read(uint8_t* buffer, size_t size);
    
    void setDataCallback(DataCallback callback);
    
private:
    std::string device_;
    int baudrate_;
    int fd_;
    bool running_;
    DataCallback data_callback_;
    
    static constexpr size_t BUFFER_SIZE = 4096;
    
    void readThread();
};

} // namespace pico

#endif // SERIAL_INTERFACE_H
