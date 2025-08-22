// tests/mqtt/pico_data_publisher.cpp
// Publisher MQTT per il topic pico data (IMU/mag/ultrasonic/safety) conforme a onDataMessage()
// Dipendenze: libmosquitto-dev

#include <mosquitto.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <thread>
#include <csignal>
#include <cstdlib>

static volatile sig_atomic_t keep_running = 1;
static void handle_sigint(int) { keep_running = 0; }

static long long now_ms() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

int main(int argc, char** argv) {
    const char* broker = (argc > 1) ? argv[1] : "localhost";
    int port = (argc > 2) ? std::atoi(argv[2]) : 1883;
    const char* topic = (argc > 3) ? argv[3] : "smartmower/bridge/pico/data"; // default costruito in SLAM

    std::signal(SIGINT, handle_sigint);

    mosquitto_lib_init();
    mosquitto* mosq = mosquitto_new("pico_data_publisher_example", true, nullptr);
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

    std::cout << "[pico_data_publisher] Pubblico su " << topic << " (Ctrl-C per uscire)" << std::endl;

    double t = 0.0;
    int counter = 0;
    while (keep_running) {
        long long ts = now_ms();
        // IMU sintetica
        double ax = 0.0 + 0.01 * std::sin(t);
        double ay = 0.0 + 0.01 * std::cos(t);
        double az = 9.81;
        double gx = 0.0;
        double gy = 0.0;
        double gz = 0.01 * std::sin(0.5 * t);
        // Mag opzionale
        double mx = 0.12, my = 0.03, mz = 0.45;
        // Ultrasonic mock
        double us_left = 1.5, us_center = 1.2, us_right = 1.8;
        // Safety mock (bumper sporadico)
        bool bumper = ((counter % 50) == 0);

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(4)
            << "{";
        // timestamp ms
        oss << "\"timestamp\": " << ts << ",";
        // accel
        oss << "\"accel\": {"
            << "\"x\": " << ax << ","
            << "\"y\": " << ay << ","
            << "\"z\": " << az << "},";
        // gyro
        oss << "\"gyro\": {"
            << "\"x\": " << gx << ","
            << "\"y\": " << gy << ","
            << "\"z\": " << gz << "},";
        // mag opzionale
        oss << "\"mag\": {"
            << "\"x\": " << mx << ","
            << "\"y\": " << my << ","
            << "\"z\": " << mz << "},";
        // ultrasonic come oggetto con left/center/right
        oss << "\"ultrasonic\": {"
            << "\"left\": " << us_left << ","
            << "\"center\": " << us_center << ","
            << "\"right\": " << us_right << "},";
        // safety
        oss << "\"safety\": {\"bumper\": " << (bumper ? "true" : "false") << "}";
        oss << "}";

        std::string payload = oss.str();
        rc = mosquitto_publish(mosq, nullptr, topic, (int)payload.size(), payload.data(), 1, false);
        if (rc != MOSQ_ERR_SUCCESS) {
            std::cerr << "Publish fallito: " << mosquitto_strerror(rc) << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 20 Hz
        t += 0.05;
        counter++;
    }

    mosquitto_loop_stop(mosq, true);
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    return 0;
}
