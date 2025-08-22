#include "config/config_manager.h"
#include "mqtt/mqtt_client.h"
#include <nlohmann/json.hpp>
#include <iostream>
#include <atomic>
#include <chrono>
#include <thread>
#include <vector>
#include <algorithm>
#include <limits>
#include <cmath>

using json = nlohmann::json;

namespace {

struct SafetyState {
    std::atomic<bool> estop_pressed{false};
    std::atomic<bool> battery_low{false};
    std::atomic<bool> lift{false};
    std::atomic<bool> theft_alarm{false};
    std::atomic<bool> tilt{false};
    std::atomic<double> battery_voltage_v{std::numeric_limits<double>::quiet_NaN()};
    std::atomic<int> battery_percentage{-1};
};

} // namespace

int main() {
    // Config
    sm::ConfigManager* cfg = sm::createJsonConfigManager();
    if (!cfg->load()) {
        std::cerr << "[safety] Config load failed" << std::endl;
        return 1;
    }

    const std::string root = cfg->getString("mqtt.root_topic", "smartmower");
    const std::string host = cfg->getString("mqtt.broker", "localhost");
    const int port = cfg->getInt("mqtt.port", 1883);
    const std::string user = cfg->getString("mqtt.username", "");
    const std::string pass = cfg->getString("mqtt.password", "");

    const std::string pico_base = cfg->getString("mqtt.topics.pico.base", "bridge/pico");
    const std::string pico_data = cfg->getString("mqtt.topics.pico.subtopics.data", "data");
    const std::string pico_cmds = cfg->getString("mqtt.topics.pico.subtopics.commands", "commands");
    const std::string topic_pico_data = root + "/" + pico_base + "/" + pico_data; // smartmower/bridge/pico/data
    const std::string topic_pico_cmd_estop = root + "/" + pico_base + "/" + pico_cmds + "/emergency_stop"; // smartmower/bridge/pico/commands/emergency_stop

    const std::string gps_base = cfg->getString("mqtt.topics.gps.base", "bridge/gps");
    const std::string gps_data = cfg->getString("mqtt.topics.gps.subtopics.data", "data");
    const std::string topic_gps_data = root + "/" + gps_base + "/" + gps_data; // smartmower/bridge/gps/data

    const std::string safety_base = cfg->getString("mqtt.topics.safety.base", "safety");
    const std::string safety_state = cfg->getString("mqtt.topics.safety.subtopics.state", "state");
    const std::string safety_estop = cfg->getString("mqtt.topics.safety.subtopics.estop", "estop");
    const std::string safety_cmds = cfg->getString("mqtt.topics.safety.subtopics.commands", "commands");

    const std::string topic_safety_state = root + "/" + safety_base + "/" + safety_state; // smartmower/safety/state
    const std::string topic_safety_estop = root + "/" + safety_base + "/" + safety_estop; // smartmower/safety/estop
    const std::string topic_safety_cmds = root + "/" + safety_base + "/" + safety_cmds;   // smartmower/safety/commands

    // MQTT QoS/Retain parameters (topic-level)
    const int pico_qos = cfg->getInt("mqtt.topics.pico.qos", 0);
    const int safety_qos = cfg->getInt("mqtt.topics.safety.qos", 0);
    const bool safety_retain = cfg->getBool("mqtt.topics.safety.retain", false);

    // Safety feature flags
    const bool emergency_stop_enabled = cfg->getBool("system.safety.emergency_stop_enabled", true);
    const bool lift_sensor_enabled = cfg->getBool("system.safety.lift_sensor_enabled", true);
    const bool theft_alarm_enabled = cfg->getBool("system.safety.theft_alarm_enabled", false);
    const bool geofence_enabled = cfg->getBool("system.safety.geofence_enabled", true);

    // Geofence polygon (array di [lat,lon]) opzionale
    std::vector<std::pair<double,double>> geofence_poly;
    try {
        nlohmann::json geoj = cfg->get("system.safety.geofence.polygon");
        if (geoj.is_array()) {
            for (const auto& pt : geoj) {
                if (pt.is_array() && pt.size() == 2 && pt[0].is_number() && pt[1].is_number()) {
                    geofence_poly.emplace_back(pt[0].get<double>(), pt[1].get<double>());
                }
            }
        }
    } catch (...) {
        // no polygon defined
    }

    auto pointInPolygon = [&](double lat, double lon) -> bool {
        if (geofence_poly.size() < 3) return true; // no geofence
        bool inside = false;
        for (size_t i = 0, j = geofence_poly.size() - 1; i < geofence_poly.size(); j = i++) {
            const auto& pi = geofence_poly[i];
            const auto& pj = geofence_poly[j];
            double yi = pi.first, xi = pi.second;   // (lat,lon) -> (y,x)
            double yj = pj.first, xj = pj.second;
            bool intersect = ((yi > lat) != (yj > lat)) &&
                             (lon < (xj - xi) * (lat - yi) / (yj - yi + 1e-12) + xi);
            if (intersect) inside = !inside;
        }
        return inside;
    };

    // Battery config: low threshold and optional OCV->% mapping by battery type
    const int low_pct = cfg->getInt("system.safety.battery.low_threshold", 30);
    const std::string battery_type = cfg->getString("system.safety.battery.type", "");
    nlohmann::json profiles = cfg->getObject("system.safety.battery.profiles");

    // Parse OCV profile for the configured battery type, if any
    std::vector<std::pair<double,int>> ocv_to_pct;
    if (!battery_type.empty() && profiles.is_object() && profiles.contains(battery_type)) {
        const auto& prof = profiles[battery_type];
        if (prof.contains("ocv_to_pct") && prof["ocv_to_pct"].is_array()) {
            for (const auto& pt : prof["ocv_to_pct"]) {
                if (pt.is_array() && pt.size() == 2 && pt[0].is_number() && pt[1].is_number()) {
                    ocv_to_pct.emplace_back(pt[0].get<double>(), static_cast<int>(pt[1].get<double>() + 0.5));
                }
            }
            std::sort(ocv_to_pct.begin(), ocv_to_pct.end(), [](auto& a, auto& b){ return a.first < b.first; });
        }
    }

    auto pctFromVoltage = [&](double vpack) -> int {
        if (std::isnan(vpack)) return -1;
        if (ocv_to_pct.empty()) return -1;
        // Assumiamo ocv_to_pct ordinato per tensione crescente
        for (size_t i = 1; i < ocv_to_pct.size(); ++i) {
            double v0 = ocv_to_pct[i-1].first;
            double v1 = ocv_to_pct[i].first;
            int p0 = ocv_to_pct[i-1].second;
            int p1 = ocv_to_pct[i].second;
            if (vpack <= v0) return p0;
            if (vpack >= v1 && i == ocv_to_pct.size()-1) return p1;
            if (vpack > v0 && vpack <= v1) {
                double t = (vpack - v0) / (v1 - v0 + 1e-9);
                int p = static_cast<int>(std::round(p0 + t * (p1 - p0)));
                return std::max(0, std::min(100, p));
            }
        }
        return -1;
    };

    // Battery profile information could be used by a richer estimator. For now we expect Pico to send pack voltage.

    SafetyState state;

    // MQTT
    std::unique_ptr<sm::MqttClient> mqtt(sm::createMqttClient());
    if (!mqtt->connect(host, port, user, pass)) {
        std::cerr << "[safety] MQTT connect failed" << std::endl;
        return 2;
    }

    mqtt->setMessageHandler([&](const std::string& topic, const std::string& payload){
        try {
            if (topic == topic_pico_data) {
                json j = json::parse(payload);
                const bool estop_before = state.estop_pressed.load();
                // Determinare la possibile causa dell'E-Stop per la notifica al Pico
                bool cause_button = false;
                bool cause_lift = false;
                bool cause_tilt = false;

                // E-Stop (flat or nested)
                bool estop_val = state.estop_pressed.load();
                if (j.contains("estop") && j["estop"].is_boolean()) {
                    // il Pico può solo attivare, non resettare
                    if (j["estop"].get<bool>()) { estop_val = true; cause_button = true; }
                }
                if (j.contains("estop_pressed") && j["estop_pressed"].is_boolean()) {
                    if (j["estop_pressed"].get<bool>()) { estop_val = true; cause_button = true; }
                }
                if (j.contains("safety") && j["safety"].is_object()) {
                    const auto& js = j["safety"];
                    if (js.contains("emergency") && js["emergency"].is_boolean()) {
                        if (js["emergency"].get<bool>()) { estop_val = true; cause_button = true; }
                    }
                    if (js.contains("lift") && js["lift"].is_boolean()) {
                        bool lift_now = js["lift"].get<bool>();
                        state.lift.store(lift_now);
                        if (lift_sensor_enabled && lift_now) {
                            // lift attiva E-Stop e Theft (latching)
                            if (emergency_stop_enabled) state.estop_pressed.store(true);
                            if (theft_alarm_enabled) state.theft_alarm.store(true);
                            cause_lift = true;
                        }
                    }
                    if (js.contains("tilt") && js["tilt"].is_boolean()) {
                        bool tilt_now = js["tilt"].get<bool>();
                        state.tilt.store(tilt_now);
                        if (tilt_now) {
                            cause_tilt = true;
                            if (emergency_stop_enabled) state.estop_pressed.store(true); // tilt implica E-Stop
                        }
                    }
                }
                if (emergency_stop_enabled) {
                    // latch: solo set a true da Pico, reset solo via comandi utente
                    if (estop_val) state.estop_pressed.store(true);
                }
                // Se l'estop è passato da false->true, informiamo il Pico
                if (!estop_before && state.estop_pressed.load()) {
                    // Precedenza cause: lifted > tilt > button > fallback
                    std::string reason = "safety_triggered";
                    if (cause_lift) reason = "lifted";
                    else if (cause_tilt) reason = "tilt";
                    else if (cause_button) reason = "button";
                    json cmd = json::object();
                    cmd["command"] = "emergency_stop";
                    cmd["active"] = true;
                    cmd["reason"] = reason;
                    cmd["timestamp"] = (long long) std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count();
                    mqtt->publish(topic_pico_cmd_estop, cmd.dump(), cfg->getInt("mqtt.topics.pico.qos", 1), cfg->getBool("mqtt.topics.pico.retain", false));
                }

                bool battery_low = state.battery_low.load();
                int est_pct = -1;
                double v = std::numeric_limits<double>::quiet_NaN();
                double current_a = std::numeric_limits<double>::quiet_NaN();
                // Parse power block
                if (j.contains("power") && j["power"].is_object()) {
                    const auto& p = j["power"];
                    if (p.contains("bus_voltage") && p["bus_voltage"].is_number()) {
                        v = p["bus_voltage"].get<double>();
                    }
                    if (p.contains("current") && p["current"].is_number()) {
                        current_a = p["current"].get<double>();
                    }
                }
                // Compat: parse legacy 'battery' object
                if (j.contains("battery") && j["battery"].is_object()) {
                    const auto& b = j["battery"];
                    if (std::isnan(v) && b.contains("voltage") && b["voltage"].is_number()) {
                        v = b["voltage"].get<double>();
                    }
                    if (std::isnan(current_a) && b.contains("current") && b["current"].is_number()) {
                        current_a = b["current"].get<double>();
                    }
                    if (b.contains("level") && b["level"].is_number() && est_pct < 0) {
                        est_pct = static_cast<int>(std::round(b["level"].get<double>()));
                    }
                }
                // Fallback compatibilità: 'voltage_v' a livello top
                if (std::isnan(v) && j.contains("voltage_v") && j["voltage_v"].is_number()) {
                    v = j["voltage_v"].get<double>();
                }
                // Fallback non standard per corrente a livello top (se presente)
                if (std::isnan(current_a) && j.contains("current") && j["current"].is_number()) {
                    current_a = j["current"].get<double>();
                }
                // Estimare percentuale: prima da OCV, fallback a battery_pct
                est_pct = pctFromVoltage(v);
                if (est_pct < 0 && j.contains("battery_pct") && j["battery_pct"].is_number()) {
                    est_pct = static_cast<int>(std::round(j["battery_pct"].get<double>()));
                }
                if (est_pct >= 0) {
                    battery_low = (est_pct <= low_pct);
                }
                state.battery_low.store(battery_low);

                // Publish estop as plain boolean (true|false)
                const bool estop_pressed = state.estop_pressed.load();
                mqtt->publish(topic_safety_estop, estop_pressed ? "true" : "false", safety_qos, safety_retain);

                // Theft alarm evaluation (simple OR of geofence breach or lift)
                if (theft_alarm_enabled) {
                    bool theft = state.theft_alarm.load();
                    if (lift_sensor_enabled && state.lift.load()) theft = true;
                    // Geofence breach will be updated in GPS handler; here we don't change it.
                    state.theft_alarm.store(theft);
                }

                // Publish safety state (new schema)
                json js = json::object();
                if (!std::isnan(v)) { js["battery_voltage_v"] = v; state.battery_voltage_v.store(v); }
                if (est_pct >= 0) { js["battery_percentage"] = est_pct; state.battery_percentage.store(est_pct); }
                if (!std::isnan(current_a)) js["battery_state"] = (current_a < 0.0 ? "charge" : "discharge");
                js["battery_low"] = state.battery_low.load();
                if (lift_sensor_enabled) js["lift"] = state.lift.load();
                if (theft_alarm_enabled) js["theft_alarm"] = state.theft_alarm.load();
                js["tilt"] = state.tilt.load();
                mqtt->publish(topic_safety_state, js.dump(), safety_qos, safety_retain);
            } else if (topic == topic_gps_data) {
                // GPS position for geofence
                if (!geofence_enabled) return;
                json g = json::parse(payload);
                if (g.contains("latitude") && g.contains("longitude") && g["latitude"].is_number() && g["longitude"].is_number()) {
                    double lat = g["latitude"].get<double>();
                    double lon = g["longitude"].get<double>();
                    bool inside = pointInPolygon(lat, lon);
                    if (theft_alarm_enabled) {
                        // Theft alarm if outside geofence
                        if (!inside) state.theft_alarm.store(true); // latch to true
                    }
                    // Pubblica stato aggiornato subito dopo update geofence
                    json js = json::object();
                    // includi ultimi valori batteria se disponibili
                    double vb = state.battery_voltage_v.load();
                    int pp = state.battery_percentage.load();
                    if (!std::isnan(vb)) js["battery_voltage_v"] = vb;
                    if (pp >= 0) js["battery_percentage"] = pp;
                    js["battery_low"] = state.battery_low.load();
                    if (lift_sensor_enabled) js["lift"] = state.lift.load();
                    if (theft_alarm_enabled) js["theft_alarm"] = state.theft_alarm.load();
                    js["tilt"] = state.tilt.load();
                    mqtt->publish(topic_safety_state, js.dump(), safety_qos, safety_retain);
                }
            } else if (topic == topic_safety_cmds) {
                // User commands: reset estop/theft_alarm
                json c = json::parse(payload);
                if (c.contains("reset") && c["reset"].is_object()) {
                    const auto& r = c["reset"];
                    bool changed = false;
                    if (r.contains("estop") && r["estop"].is_boolean() && r["estop"].get<bool>()) {
                        state.estop_pressed.store(false);
                        changed = true;
                    }
                    if (r.contains("theft_alarm") && r["theft_alarm"].is_boolean() && r["theft_alarm"].get<bool>()) {
                        state.theft_alarm.store(false);
                        changed = true;
                    }
                    if (changed) {
                        mqtt->publish(topic_safety_estop, state.estop_pressed.load() ? "true" : "false", safety_qos, safety_retain);
                        json js = json::object();
                        double vb = state.battery_voltage_v.load();
                        int pp = state.battery_percentage.load();
                        if (!std::isnan(vb)) js["battery_voltage_v"] = vb;
                        if (pp >= 0) js["battery_percentage"] = pp;
                        js["battery_low"] = state.battery_low.load();
                        if (lift_sensor_enabled) js["lift"] = state.lift.load();
                        if (theft_alarm_enabled) js["theft_alarm"] = state.theft_alarm.load();
                        js["tilt"] = state.tilt.load();
                        mqtt->publish(topic_safety_state, js.dump(), safety_qos, safety_retain);
                    }
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "[safety] onMessage error: " << e.what() << std::endl;
        }
    });

    mqtt->subscribe(topic_pico_data, pico_qos);
    mqtt->subscribe(topic_gps_data, cfg->getInt("mqtt.topics.gps.qos", 1));
    mqtt->subscribe(topic_safety_cmds, cfg->getInt("mqtt.topics.safety.qos", 1));

    // Initial publications: set defaults
    if (emergency_stop_enabled) {
        mqtt->publish(topic_safety_estop, "false", safety_qos, safety_retain);
    }
    {
        json js0 = json::object();
        // inizialmente non abbiamo tensione/percentuale
        js0["battery_low"] = state.battery_low.load();
        if (lift_sensor_enabled) js0["lift"] = false;
        if (theft_alarm_enabled) js0["theft_alarm"] = false;
        js0["tilt"] = false;
        mqtt->publish(topic_safety_state, js0.dump(), safety_qos, safety_retain);
    }

    // Heartbeat loop (optional): keep process alive
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
