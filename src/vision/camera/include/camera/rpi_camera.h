#ifndef RPI_CAMERA_H
#define RPI_CAMERA_H

#include "camera_interface.h"
#include <vector>
#include <cstdint>
#include <memory>
#include <libcamera/libcamera.h>
#include <libcamera/framebuffer_allocator.h>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <cstdint>
#include <opencv2/opencv.hpp>

namespace camera {

class RPiCamera : public CameraInterface {
public:
    RPiCamera(int width = 640, int height = 480, int fps = 15);
    ~RPiCamera() override;

    bool initialize() override;
    bool captureFrame(std::vector<uint8_t>& buffer) override;
    void shutdown() override;

private:
    int width_;
    int height_;
    int fps_;
    std::unique_ptr<libcamera::CameraManager> cameraManager_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> config_;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    libcamera::Stream* stream_;
    std::vector<std::unique_ptr<libcamera::Request>> requests_;
    bool started_;
    std::vector<uint8_t> lastFrame_;
    std::mutex frameMutex_;
    std::condition_variable frameCV_;
};

} // namespace camera

#endif // RPI_CAMERA_H
