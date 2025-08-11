#ifndef USB_CAMERA_H
#define USB_CAMERA_H

#include "camera_interface.h"
#include <vector>
#include <cstdint>

namespace camera {

class USBCamera : public CameraInterface {
public:
    USBCamera(int device_index = 0, int width = 640, int height = 480, int fps = 15);
    ~USBCamera() override;

    bool initialize() override;
    bool captureFrame(std::vector<uint8_t>& buffer) override;
    void shutdown() override;

private:
    int device_index_;
    int width_;
    int height_;
    int fps_;
    void* capture_handle_; // e.g., cv::VideoCapture*
};

} // namespace camera

#endif // USB_CAMERA_H
