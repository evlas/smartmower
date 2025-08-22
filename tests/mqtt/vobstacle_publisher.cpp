// tests/mqtt/vobstacle_publisher.cpp
// Publisher MQTT per topic ostacoli da vision (vobstacle) conforme a onObstacleMessage()
// Dipendenze: libmosquitto-dev

#include <mosquitto.h>
#include <chrono>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <thread>
#include <csignal>
#include <cstdlib>

static volatile sig_atomic_t keep_running = 1;
static void handle_sigint(int) { keep_running = 0; }

static unsigned long long now_us() {
    using namespace std::chrono;
    return duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
}
static unsigned long long now_ms() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

int main(int argc, char** argv) {
    const char* broker = (argc > 1) ? argv[1] : "localhost";
    int port = (argc > 2) ? std::atoi(argv[2]) : 1883;
    const char* topic = (argc > 3) ? argv[3] : "smartmower/vision/obstacle/data"; // fallback come in SLAM

    std::signal(SIGINT, handle_sigint);

    mosquitto_lib_init();
    mosquitto* mosq = mosquitto_new("vobstacle_publisher_example", true, nullptr);
    if (!mosq) {
        std::cerr << "Errore: impossibile creare client mosquitto" << std::endl;
        return 1;
    }
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

    std::cout << "[vobstacle_publisher] Pubblico su " << topic << " (Ctrl-C per uscire)" << std::endl;

    // Alterna tra nessun ostacolo e ostacolo con distanza/confidenza
    int counter = 0;
    while (keep_running) {
        bool detected = (counter++ % 5) == 0; // ostacolo ogni 5 messaggi
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3) << "{";
        oss << "\"obstacle_detected\": " << (detected ? "true" : "false") << ",";
        if (detected) {
            double dist = 1.2; // m
            double conf = 0.7;
            oss << "\"estimated_distance_m\": " << dist << ",";
            oss << "\"confidence\": " << conf << ",";
        }
        // timestamp_us preferito; forniamo anche timestamp ms come fallback
        oss << "\"timestamp_us\": " << now_us() << ",";
        oss << "\"timestamp\": " << now_ms();
        oss << "}";

        std::string payload = oss.str();
        rc = mosquitto_publish(mosq, nullptr, topic, (int)payload.size(), payload.data(), 1, false);
        if (rc != MOSQ_ERR_SUCCESS) {
            std::cerr << "Publish fallito: " << mosquitto_strerror(rc) << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200)); // 5 Hz
    }

    mosquitto_loop_stop(mosq, true);
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    return 0;
}
