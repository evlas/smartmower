// tests/mqtt/imu_publisher.cpp
// Publisher MQTT per pacchetti IMU binari conformi a pico::SensorData
// Dipendenze: libmosquitto-dev

#include <mosquitto.h>
#include <chrono>
#include <thread>
#include <cstring>
#include <iostream>

#pragma pack(push, 1)
struct SensorData {
    uint8_t  type;            // 0x01
    uint32_t timestamp;       // ms
    float    accel[3];        // m/s^2
    float    gyro[3];         // rad/s
    float    mag[3];          // uT
    float    us_distances[3]; // m
    float    bus_voltage;     // V
    float    current;         // A
    uint8_t  safety_flags;    // bitmask
};
#pragma pack(pop)

static uint32_t now_ms() {
    using namespace std::chrono;
    return static_cast<uint32_t>(
        duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count()
    );
}

int main(int argc, char** argv) {
    const char* broker = (argc > 1) ? argv[1] : "localhost";
    int port = (argc > 2) ? std::atoi(argv[2]) : 1883;
    const char* topic = (argc > 3) ? argv[3] : "smartmower/pico/data";

    mosquitto_lib_init();
    mosquitto* mosq = mosquitto_new("imu_publisher_example", true, nullptr);
    if (!mosq) {
        std::cerr << "Errore: impossibile creare client mosquitto" << std::endl;
        return 1;
    }

    // Imposta credenziali MQTT (utente: mower, password: smart)
    mosquitto_username_pw_set(mosq, "mower", "smart");

    int rc = mosquitto_connect(mosq, broker, port, 60);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "Connessione MQTT fallita: " << mosquitto_strerror(rc) << std::endl;
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return 1;
    }

    // Invia 100 pacchetti a ~100 Hz
    for (int i = 0; i < 100; ++i) {
        SensorData pkt{};
        pkt.type = 0x01;
        pkt.timestamp = now_ms();
        pkt.accel[0] = 0.0f; pkt.accel[1] = 0.0f; pkt.accel[2] = 9.81f;
        pkt.gyro[0]  = 0.0f; pkt.gyro[1]  = 0.0f; pkt.gyro[2]  = 0.0f;
        pkt.mag[0]   = 0.0f; pkt.mag[1]   = 0.0f; pkt.mag[2]   = 0.0f;
        pkt.us_distances[0] = 2.0f; pkt.us_distances[1] = 2.0f; pkt.us_distances[2] = 2.0f;
        pkt.bus_voltage = 24.0f; pkt.current = 1.2f; pkt.safety_flags = 0x00;

        rc = mosquitto_publish(mosq, nullptr, topic, sizeof(pkt), &pkt, 1, false);
        if (rc != MOSQ_ERR_SUCCESS) {
            std::cerr << "Publish fallito: " << mosquitto_strerror(rc) << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    std::cout << "IMU sample inviati su " << topic << std::endl;
    return 0;
}
