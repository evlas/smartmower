// tests/mqtt/vo_publisher.cpp
// Publisher MQTT per Visual Odometry (vodometry) in JSON conforme a onOdometryMessage()
// Dipendenze: libmosquitto-dev

#include <mosquitto.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <csignal>
#include <cstdlib>
#include <thread>

static volatile sig_atomic_t keep_running = 1;
static void handle_sigint(int) { keep_running = 0; }

static unsigned long long now_us() {
    using namespace std::chrono;
    return duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
}

int main(int argc, char** argv) {
    const char* broker = (argc > 1) ? argv[1] : "localhost";
    int port = (argc > 2) ? std::atoi(argv[2]) : 1883;
    const char* topic = (argc > 3) ? argv[3] : "smartmower/vision/odometry/data"; // fallback come in SLAM

    std::signal(SIGINT, handle_sigint);

    mosquitto_lib_init();
    mosquitto* mosq = mosquitto_new("vo_publisher_example", true, nullptr);
    if (!mosq) {
        std::cerr << "Errore: impossibile creare client mosquitto" << std::endl;
        return 1;
    }

    // Credenziali MQTT (default repo)
    mosquitto_username_pw_set(mosq, "mower", "smart");

    int rc = mosquitto_connect(mosq, broker, port, 60);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "Connessione MQTT fallita: " << mosquitto_strerror(rc) << std::endl;
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return 1;
    }

    rc = mosquitto_loop_start(mosq);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "Errore loop MQTT: " << mosquitto_strerror(rc) << std::endl;
        mosquitto_disconnect(mosq);
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return 1;
    }

    std::cout << "[vo_publisher] Pubblico su " << topic << " (Ctrl-C per uscire)" << std::endl;

    // Genera piccoli delta di traslazione con rotazione nulla
    double t = 0.0;
    while (keep_running) {
        unsigned long long ts_us = now_us();
        double dx = 0.01 * std::cos(t);
        double dy = 0.01 * std::sin(t);
        double dz = 0.0;
        t += 0.1;

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6)
            << "{";
        // timestamp_us
        oss << "\"timestamp_us\": " << ts_us << ",";
        // translation
        oss << "\"translation\": {"
            << "\"x\": " << dx << ","
            << "\"y\": " << dy << ","
            << "\"z\": " << dz << "},";
        // rotation_quat (identità)
        oss << "\"rotation_quat\": {\"w\": 1.0, \"x\": 0.0, \"y\": 0.0, \"z\": 0.0},";
        // confidence
        oss << "\"confidence\": 0.8";
        oss << "}";

        std::string payload = oss.str();
        rc = mosquitto_publish(mosq, nullptr, topic, (int)payload.size(), payload.data(), 1, false);
        if (rc != MOSQ_ERR_SUCCESS) {
            std::cerr << "Publish fallito: " << mosquitto_strerror(rc) << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10 Hz
    }

    mosquitto_loop_stop(mosq, true);
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    return 0;
}
