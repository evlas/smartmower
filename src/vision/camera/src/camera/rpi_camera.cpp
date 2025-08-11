#include "camera/rpi_camera.h"
#include <libcamera/controls.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/request.h>
#include <opencv2/imgcodecs.hpp>
#include <sys/mman.h>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <iostream>

namespace camera {

RPiCamera::RPiCamera(int width, int height, int fps)
    : width_(width), height_(height), fps_(fps), stream_(nullptr), started_(false) {}

RPiCamera::~RPiCamera() {
    shutdown();
}

bool RPiCamera::initialize() {
    try {
        cameraManager_ = std::make_unique<libcamera::CameraManager>();
        cameraManager_->start();
        if (cameraManager_->cameras().empty()) return false;
        auto camInfo = cameraManager_->cameras()[0];
        camera_ = cameraManager_->get(camInfo->id());
        if (!camera_) return false;
        if (camera_->acquire()) return false;

        config_ = camera_->generateConfiguration({libcamera::StreamRole::Viewfinder});
        auto &cfg = config_->at(0);
        cfg.size.width = width_;
        cfg.size.height = height_;
        cfg.pixelFormat = libcamera::formats::BGR888;
        cfg.bufferCount = 4;
        if (config_->validate() == libcamera::CameraConfiguration::Invalid) return false;
        if (camera_->configure(config_.get()) < 0) return false;

        stream_ = cfg.stream();

        allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);
        if (allocator_->allocate(stream_) < 0) return false;
        for (auto &b : allocator_->buffers(stream_)) {
            auto req = camera_->createRequest();
            if (!req) return false;
            if (req->addBuffer(stream_, b.get()) < 0) return false;
            requests_.push_back(std::move(req));
        }

        camera_->requestCompleted.connect(camera_.get(), [this](libcamera::Request *request) {
            const auto &buffers = request->buffers();
            for (auto &p : buffers) {
                auto fb = p.second;
                const auto &planes = fb->planes();
                if (planes.empty()) continue;
                auto &plane = planes[0];
                void *map = mmap(NULL, plane.length, PROT_READ, MAP_SHARED,
                                 plane.fd.get(), plane.offset);
                if (map == MAP_FAILED) continue;
                cv::Mat img(height_, width_, CV_8UC3, map);
                std::vector<uchar> buf;
                cv::imencode(".jpg", img, buf);
                munmap(map, plane.length);
                {
                    std::lock_guard<std::mutex> lk(frameMutex_);
                    lastFrame_ = std::move(buf);
                }
                frameCV_.notify_one();
                break;
            }
        });

        return true;
    } catch (const std::exception &e) {
        std::cerr << "RPiCamera init error: " << e.what() << std::endl;
        return false;
    }
}

bool RPiCamera::captureFrame(std::vector<uint8_t>& buffer) {
    if (!camera_) return false;
    if (!started_) {
        if (camera_->start()) return false;
        for (auto &r : requests_) {
            if (camera_->queueRequest(r.get()) < 0) return false;
        }
        started_ = true;
    }
    std::unique_lock<std::mutex> lk(frameMutex_);
    auto status = frameCV_.wait_for(lk, std::chrono::milliseconds(1000 / fps_));
    if (status == std::cv_status::timeout) return false;
    buffer = lastFrame_;
    return true;
}

void RPiCamera::shutdown() {
    if (camera_ && started_) {
        camera_->stop();
        started_ = false;
    }
    if (camera_) {
        camera_->release();
        camera_.reset();
    }
    if (allocator_) allocator_.reset();
    if (cameraManager_) {
        cameraManager_->stop();
        cameraManager_.reset();
    }
}

} // namespace camera
