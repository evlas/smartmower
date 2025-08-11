#ifndef CAMERA_INTERFACE_H
#define CAMERA_INTERFACE_H

#include <vector>
#include <cstdint>

namespace camera {

class CameraInterface {
public:
    virtual ~CameraInterface() = default;
    virtual bool initialize() = 0;
    virtual bool captureFrame(std::vector<uint8_t>& buffer) = 0;
    virtual void shutdown() = 0;
};

} // namespace camera

#endif // CAMERA_INTERFACE_H
