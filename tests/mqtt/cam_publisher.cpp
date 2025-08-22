// tests/mqtt/cam_publisher.cpp
// Publisher MQTT per immagini compresse (JPEG/PNG) decodificabili da OpenCV
// Dipendenze: libmosquitto-dev, OpenCV (core, imgcodecs)

#include <mosquitto.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <cstdlib>

int main(int argc, char** argv) {
    const char* broker = (argc > 1) ? argv[1] : "localhost";
    int port = (argc > 2) ? std::atoi(argv[2]) : 1883;
    const char* topic = (argc > 3) ? argv[3] : "smartmower/vision/camera/data";
    const char* image_path = (argc > 4) ? argv[4] : "tests/data/sample_camera.jpg";

    // Carica immagine
    cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
    if (img.empty()) {
        std::cerr << "Impossibile aprire l'immagine: " << image_path << std::endl;
        return 1;
    }

    // Encoda in JPEG
    std::vector<uchar> buf;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 85};
    if (!cv::imencode(".jpg", img, buf, params)) {
        std::cerr << "Imencode JPEG fallita" << std::endl;
        return 1;
    }

    mosquitto_lib_init();
    mosquitto* mosq = mosquitto_new("cam_publisher_example", true, nullptr);
    if (!mosq) {
        std::cerr << "Errore: impossibile creare client mosquitto" << std::endl;
        return 1;
    }

    int rc = mosquitto_connect(mosq, broker, port, 60);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "Connessione MQTT fallita: " << mosquitto_strerror(rc) << std::endl;
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return 1;
    }

    rc = mosquitto_publish(mosq, nullptr, topic, (int)buf.size(), buf.data(), 1, false);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "Publish fallito: " << mosquitto_strerror(rc) << std::endl;
    } else {
        std::cout << "Frame inviato su " << topic << " (" << buf.size() << " bytes)" << std::endl;
    }

    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    return 0;
}
