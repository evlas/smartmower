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
#include <sstream>
#include <fstream>

namespace camera {

RPiCamera::RPiCamera(int width, int height, int fps)
    : width_(width), height_(height), fps_(fps), stream_(nullptr), started_(false) {
    std::cout << "RPiCamera constructor - Width: " << width_ << ", Height: " << height_ << ", FPS: " << fps_ << std::endl;
    // Inizializza la variabile di shutdown
    shutdown_ = false;
}

RPiCamera::~RPiCamera() {
    shutdown();
}

bool RPiCamera::initialize() {
    try {
        std::cout << "Initializing RPiCamera..." << std::endl;
        
        cameraManager_ = std::make_unique<libcamera::CameraManager>();
        cameraManager_->start();
        
        std::cout << "Camera manager started. Found " << cameraManager_->cameras().size() << " cameras." << std::endl;
        
        if (cameraManager_->cameras().empty()) {
            std::cerr << "No cameras found!" << std::endl;
            return false;
        }
        
        auto camInfo = cameraManager_->cameras()[0];
        std::cout << "Using camera: " << camInfo->id() << std::endl;
        
        camera_ = cameraManager_->get(camInfo->id());
        if (!camera_) {
            std::cerr << "Failed to get camera instance" << std::endl;
            return false;
        }
        
        if (camera_->acquire()) {
            std::cerr << "Failed to acquire camera" << std::endl;
            return false;
        }

        // Stampa informazioni sulla configurazione della telecamera
        std::cout << "Configuring camera with resolution: " << width_ << "x" << height_ << " at " << fps_ << " FPS" << std::endl;
        
        config_ = camera_->generateConfiguration({libcamera::StreamRole::Viewfinder});
        auto &cfg = config_->at(0);
        cfg.size.width = width_;
        cfg.size.height = height_;
        
        // Usiamo il formato BGR888 per la compatibilità
        cfg.pixelFormat = libcamera::formats::BGR888;
        std::cout << "Using BGR888 pixel format" << std::endl;
        
        cfg.bufferCount = 4;
        
        std::cout << "Validating configuration..." << std::endl;
        if (config_->validate() == libcamera::CameraConfiguration::Invalid) {
            std::cerr << "Configuration validation failed" << std::endl;
            return false;
        }
        
        std::cout << "Configuring camera with: " << cfg.toString() << std::endl;
        if (camera_->configure(config_.get()) < 0) {
            std::cerr << "Failed to configure camera" << std::endl;
            return false;
        }

        stream_ = cfg.stream();

        allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);
        if (allocator_->allocate(stream_) < 0) {
            std::cerr << "Failed to allocate frame buffers" << std::endl;
            return false;
        }
        
        std::cout << "Allocated " << allocator_->buffers(stream_).size() << " frame buffers" << std::endl;
        
        for (auto &b : allocator_->buffers(stream_)) {
            auto req = camera_->createRequest();
            if (!req) {
                std::cerr << "Failed to create request" << std::endl;
                return false;
            }
            if (req->addBuffer(stream_, b.get()) < 0) {
                std::cerr << "Failed to add buffer to request" << std::endl;
                return false;
            }
            requests_.push_back(std::move(req));
        }

        camera_->requestCompleted.connect(camera_.get(), [this](libcamera::Request *request) {
            try {
                const auto &buffers = request->buffers();
                for (auto &p : buffers) {
                    auto fb = p.second;
                    const auto &planes = fb->planes();
                    if (planes.empty()) {
                        std::cerr << "No planes in frame buffer" << std::endl;
                        continue;
                    }
                    
                    auto &plane = planes[0];
                    void *map = mmap(NULL, plane.length, PROT_READ, MAP_SHARED,
                                   plane.fd.get(), plane.offset);
                    
                    if (map == MAP_FAILED) {
                        std::cerr << "mmap failed: " << strerror(errno) << std::endl;
                        continue;
                    }
                    
                    try {
                        cv::Mat img;
                        if (stream_->configuration().pixelFormat == libcamera::formats::YUV420) {
                            // Handle YUV420 format
                            cv::Mat yuv(height_ + height_/2, width_, CV_8UC1, map);
                            cv::cvtColor(yuv, img, cv::COLOR_YUV2BGR_I420);
                        } else {
                            // Handle BGR format
                            img = cv::Mat(height_, width_, CV_8UC3, map);
                        }
                        
                        std::vector<uchar> buf;
                        cv::imencode(".jpg", img, buf);
                        
                        {
                            std::lock_guard<std::mutex> lk(frameMutex_);
                            lastFrame_ = std::move(buf);
                        }
                        frameCV_.notify_one();
                        
                    } catch (const cv::Exception& e) {
                        std::cerr << "OpenCV error: " << e.what() << std::endl;
                    }
                    
                    munmap(map, plane.length);
                    break;
                }
                
                // Requeue the request for the next frame
                if (camera_ && started_) {
                    request->reuse(libcamera::Request::ReuseFlag::ReuseBuffers);
                    camera_->queueRequest(request);
                }
                
            } catch (const std::exception &e) {
                std::cerr << "Error in requestCompleted handler: " << e.what() << std::endl;
            }
        });

        return true;
    } catch (const std::exception &e) {
        std::cerr << "RPiCamera init error: " << e.what() << std::endl;
        return false;
    }
}

bool RPiCamera::captureFrame(std::vector<uint8_t>& buffer) {
    try {
        if (shutdown_) {
            std::cerr << "Camera is shutting down" << std::endl;
            return false;
        }

        if (!camera_) {
            std::cerr << "Camera not initialized" << std::endl;
            return false;
        }
        
        if (!started_) {
            std::cout << "Starting camera capture..." << std::endl;
            if (camera_->start()) {
                std::cerr << "Failed to start camera" << std::endl;
                return false;
            }
            
            for (auto &r : requests_) {
                if (camera_->queueRequest(r.get()) < 0) {
                    std::cerr << "Failed to queue request" << std::endl;
                    return false;
                }
            }
            started_ = true;
            std::cout << "Camera capture started successfully" << std::endl;
            
            // Aspetta un po' per permettere alla fotocamera di avviarsi
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        
        std::unique_lock<std::mutex> lk(frameMutex_);
        // Aumenta il timeout a 3 secondi (3000ms) invece di 1000/fps
        auto status = frameCV_.wait_for(lk, std::chrono::milliseconds(3000), [this] { 
            return !lastFrame_.empty() || shutdown_; 
        });
        
        if (shutdown_) {
            std::cout << "Shutdown requested during frame capture" << std::endl;
            return false;
        }
        
        if (!status) {
            std::cerr << "Frame capture timeout (3s)" << std::endl;
            return false;
        }
        
        if (lastFrame_.empty()) {
            std::cerr << "Received empty frame" << std::endl;
            return false;
        }
        
        buffer = lastFrame_;
        return true;
        
    } catch (const std::exception &e) {
        std::cerr << "Error in captureFrame: " << e.what() << std::endl;
        return false;
    }
}

void RPiCamera::shutdown() {
    std::cout << "Shutting down RPiCamera..." << std::endl;
    
    // Imposta il flag di shutdown
    {
        std::lock_guard<std::mutex> lk(frameMutex_);
        if (shutdown_) {
            std::cout << "Shutdown already in progress" << std::endl;
            return;
        }
        shutdown_ = true;
    }
    frameCV_.notify_all();
    
    if (camera_) {
        std::cout << "Stopping camera..." << std::endl;
        if (started_) {
            try {
                // Disabilita i segnali prima di fermare la fotocamera
                camera_->requestCompleted.disconnect();
                
                // Ferma la fotocamera
                int ret = camera_->stop();
                if (ret) {
                    std::cerr << "Error stopping camera, code: " << ret << std::endl;
                }
                started_ = false;
            } catch (const std::exception& e) {
                std::cerr << "Exception while stopping camera: " << e.what() << std::endl;
            }
        }
        
        std::cout << "Releasing camera..." << std::endl;
        try {
            camera_->release();
            camera_.reset();
        } catch (const std::exception& e) {
            std::cerr << "Exception while releasing camera: " << e.what() << std::endl;
        }
    }
    
    // Pulisci le richieste prima di distruggere l'allocatore
    std::cout << "Cleaning up requests..." << std::endl;
    requests_.clear();
    
    if (allocator_) {
        std::cout << "Resetting allocator..." << std::endl;
        try {
            allocator_.reset();
        } catch (const std::exception& e) {
            std::cerr << "Exception while resetting allocator: " << e.what() << std::endl;
        }
    }
    
    if (cameraManager_) {
        std::cout << "Stopping camera manager..." << std::endl;
        try {
            cameraManager_->stop();
            cameraManager_.reset();
        } catch (const std::exception& e) {
            std::cerr << "Exception while stopping camera manager: " << e.what() << std::endl;
        }
    }
    
    // Pulisci l'ultimo frame
    {
        std::lock_guard<std::mutex> lk(frameMutex_);
        lastFrame_.clear();
    }
    
    std::cout << "RPiCamera shutdown complete" << std::endl;
}

} // namespace camera
