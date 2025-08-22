#include "config/config_manager.h"
#include "mqtt/mqtt_client.h"
#include "state_machine/state_machine.h"

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>

using namespace sm;

int main() {
    std::unique_ptr<ConfigManager> cfg(createJsonConfigManager());
    if (!cfg->load()) {
        std::cerr << "[ERROR] Impossibile caricare la configurazione. Percorsi: /opt/smartmower/etc/config/robot_config.json oppure src/config/robot_config.json" << std::endl;
        return 1;
    }

    // MQTT (stub)
    std::unique_ptr<MqttClient> mqtt(createMqttClient());
    const std::string host = cfg->getString("mqtt.broker", "localhost");
    const int port = cfg->getInt("mqtt.port", 1883);
    const std::string user = cfg->getString("mqtt.username", "");
    const std::string pass = cfg->getString("mqtt.password", "");
    mqtt->connect(host, port, user, pass);

    StateMachine sm(*cfg, mqtt.get());
    sm.start();

    // Loop principale: tick periodico
    while (true) {
        sm.tick();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}
