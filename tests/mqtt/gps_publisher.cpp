// tests/mqtt/gps_publisher.cpp
// Publisher MQTT per messaggi GPS in JSON
// Dipendenze: libmosquitto-dev

#include <mosquitto.h>
#include <chrono>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cstdlib>

static long long now_ms() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(
        system_clock::now().time_since_epoch()).count();
}

int main(int argc, char** argv) {
    const char* broker = (argc > 1) ? argv[1] : "localhost";
    int port = (argc > 2) ? std::atoi(argv[2]) : 1883;
    const char* topic = (argc > 3) ? argv[3] : "smartmower/gps/data";

    mosquitto_lib_init();
    mosquitto* mosq = mosquitto_new("gps_publisher_example", true, nullptr);
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

    // Invia un singolo messaggio di esempio
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6)
        << "{"
        << "\"lat\": " << 45.123456 << ","
        << "\"lon\": " << 9.123456 << ","
        << "\"alt\": " << 120.5 << ","
        << "\"timestamp\": " << now_ms()
        << "}";

    std::string payload = oss.str();
    rc = mosquitto_publish(mosq, nullptr, topic, payload.size(), payload.data(), 1, false);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "Publish fallito: " << mosquitto_strerror(rc) << std::endl;
    } else {
        std::cout << "GPS sample inviato su " << topic << ": " << payload << std::endl;
    }

    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    return 0;
}
