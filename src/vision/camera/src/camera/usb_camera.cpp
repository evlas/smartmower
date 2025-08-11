#include "camera/usb_camera.h"
#include <opencv2/opencv.hpp>
#include <iostream>

namespace camera {

USBCamera::USBCamera(int device_index, int width, int height, int fps)
    : device_index_(device_index), width_(width), height_(height), fps_(fps), capture_handle_(nullptr) {}

USBCamera::~USBCamera() {
    shutdown();
}

bool USBCamera::initialize() {
    auto cap = new cv::VideoCapture(device_index_);
    if (!cap->isOpened()) {
        std::cerr << "USBCamera: cannot open device " << device_index_ << std::endl;
        delete cap;
        return false;
    }
    cap->set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cap->set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap->set(cv::CAP_PROP_FPS, fps_);
    capture_handle_ = cap;
    return true;
}

bool USBCamera::captureFrame(std::vector<uint8_t>& buffer) {
    auto cap = static_cast<cv::VideoCapture*>(capture_handle_);
    if (!cap) return false;
    cv::Mat frame;
    if (!cap->read(frame) || frame.empty()) {
        return false;
    }
    std::vector<uchar> buf;
    if (!cv::imencode(".jpg", frame, buf)) {
        return false;
    }
    buffer.assign(buf.begin(), buf.end());
    return true;
}

void USBCamera::shutdown() {
    auto cap = static_cast<cv::VideoCapture*>(capture_handle_);
    if (cap) {
        cap->release();
        delete cap;
        capture_handle_ = nullptr;
    }
}

} // namespace camera
