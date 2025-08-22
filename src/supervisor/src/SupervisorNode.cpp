#include <iostream>
#include <csignal>
#include <chrono>
#include <thread>
#include <vector>
#include <string>
#include <map>
#include <optional>
#include <mutex>
#include <deque>
#include <cstdlib>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <nlohmann/json.hpp>

#include "config/config.h"
#include "mqtt/mqtt_client.h"

using namespace std::chrono_literals;

static volatile std::sig_atomic_t g_stop = 0;
static void handle_sigint(int) { g_stop = 1; }

struct Service {
    std::string name;
    std::string exec_path; // es: /opt/smartmower/bin/costmap_node
    pid_t pid{0};
    std::string status_topic; // es: smartmower/nav/costmap/status
    std::chrono::steady_clock::time_point last_status{std::chrono::steady_clock::now()};
    bool running{false};
    int backoff_attempts{0};
};

static int64_t nowMs() {
    return static_cast<int64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
}

class ProcessManager {
public:
    static pid_t start(const std::string& path, const std::vector<std::string>& args) {
        pid_t pid = fork();
        if (pid == 0) {
            std::vector<char*> cargs;
            cargs.push_back(const_cast<char*>(path.c_str()));
            for (const auto& a : args) cargs.push_back(const_cast<char*>(a.c_str()));
            cargs.push_back(nullptr);
            execv(path.c_str(), cargs.data());
            _exit(127);
        }
        return pid;
    }
    static bool stop(pid_t pid, int grace_ms = 2000) {
        if (pid <= 0) return true;
        kill(pid, SIGTERM);
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(grace_ms);
        int status = 0;
        while (std::chrono::steady_clock::now() < deadline) {
            if (waitpid(pid, &status, WNOHANG) == pid) return true;
            std::this_thread::sleep_for(50ms);
        }
        kill(pid, SIGKILL);
        waitpid(pid, &status, 0);
        return true;
    }
};

int main(int argc, char** argv) {
    try {
        std::signal(SIGINT, handle_sigint);
        std::signal(SIGTERM, handle_sigint);

        // Override config: ENV/CLI
        std::string config_path;
        if (const char* envp = std::getenv("SMARTMOWER_CONFIG")) config_path = envp;
        for (int i = 1; i < argc; ++i) {
            std::string a = argv[i];
            if (a == "-c" || a == "--config") { if (i+1 < argc) config_path = argv[++i]; }
            else if (a.rfind("--config=", 0) == 0) { config_path = a.substr(9); }
        }

        smartmower::config::ConfigLoader cfg(config_path.empty() ? "/opt/smartmower/etc/config/robot_config.json" : config_path);

        // MQTT settings
        auto j = cfg.json();
        std::string broker = "localhost";
        int port = 1883;
        std::string username;
        std::string password;
        if (j.contains("mqtt")) {
            const auto& jm = j["mqtt"];
            broker = jm.value("broker", broker);
            port = jm.value("port", port);
            username = jm.value("username", std::string());
            password = jm.value("password", std::string());
        }
        const std::string root = cfg.rootTopic();

        // Registry servizi (bin in /opt/smartmower/bin)
        std::map<std::string, Service> services;
        {
            Service s; s.name = "map_server"; s.exec_path = "/opt/smartmower/bin/map_server"; s.status_topic = root + "/nav/costmap/status"; services[s.name] = s;
        }
        {
            Service s; s.name = "state_machine_node"; s.exec_path = "/opt/smartmower/bin/state_machine_node"; /* status_topic non definito */ services[s.name] = s;
        }
        {
            Service s; s.name = "gps_bridge"; s.exec_path = "/opt/smartmower/bin/gps_bridge"; /* status_topic non definito */ services[s.name] = s;
        }
        {
            Service s; s.name = "pico_bridge"; s.exec_path = "/opt/smartmower/bin/pico_bridge"; /* status_topic non definito */ services[s.name] = s;
        }
        {
            Service s; s.name = "vision_obstacle"; s.exec_path = "/opt/smartmower/bin/vision_obstacle"; /* status_topic non definito */ services[s.name] = s;
        }
        {
            Service s; s.name = "fusion_node"; s.exec_path = "/opt/smartmower/bin/fusion_node"; /* status_topic non definito */ services[s.name] = s;
        }
        {
            Service s; s.name = "safety_supervisor"; s.exec_path = "/opt/smartmower/bin/safety_supervisor"; /* status_topic non definito */ services[s.name] = s;
        }

        // Coda comandi da processare nel main loop (per evitare blocchi nel callback MQTT)
        struct Cmd { std::string action; std::string service; };
        std::mutex cmd_mu;
        std::deque<Cmd> cmd_queue;

        smartmower::mqtt::MqttClient mqtt(broker, port, "smartmower_supervisor", username, password);
        if (!mqtt.connect()) {
            std::cerr << "[Supervisor][ERROR] MQTT connect fallita verso " << broker << ":" << port << std::endl;
            return 2;
        }
        // subscribe status services (solo se definito)
        for (const auto& [name, s] : services) if (!s.status_topic.empty()) mqtt.subscribe(s.status_topic);
        // subscribe api
        const std::string t_cfg_get = root + "/supervisor/config/get";
        const std::string t_cfg_set = root + "/supervisor/config/set";
        const std::string t_cmd = root + "/supervisor/cmd";
        const std::string t_cfg_state = root + "/supervisor/config/state";
        const std::string t_result = root + "/supervisor/config/result";
        const std::string t_sup_status = root + "/supervisor/status";
        mqtt.subscribe(t_cfg_get);
        mqtt.subscribe(t_cfg_set);
        mqtt.subscribe(t_cmd);

        // Subscribe dinamiche per tutti i topic di status definiti in config (mqtt.topics.*.subtopics.status)
        try {
            if (j.contains("mqtt") && j["mqtt"].contains("topics") && j["mqtt"]["topics"].is_object()) {
                const auto& topics = j["mqtt"]["topics"];
                for (auto it = topics.begin(); it != topics.end(); ++it) {
                    const auto& obj = it.value();
                    if (!obj.is_object()) continue;
                    std::string base = obj.value("base", std::string());
                    if (base.empty()) continue;
                    if (!obj.contains("subtopics") || !obj["subtopics"].is_object()) continue;
                    const auto& subs = obj["subtopics"];
                    if (subs.contains("status") && subs["status"].is_string()) {
                        std::string status_key = subs["status"].get<std::string>();
                        if (!status_key.empty()) {
                            std::string topic = root + "/" + base + "/" + status_key;
                            mqtt.subscribe(topic);
                        }
                    }
                }
            }
        } catch (...) {
            // ignora eventuali formati inattesi
        }

        mqtt.setMessageCallback([&](const std::string& topic, const std::vector<uint8_t>& payload){
            try {
                // Status servizi
                for (auto& [name, s] : services) {
                    if (topic == s.status_topic) {
                        s.last_status = std::chrono::steady_clock::now();
                        s.running = true;
                    }
                }
                // Config GET: rispondi sempre, anche con payload vuoto/non-JSON
                if (topic == t_cfg_get) {
                    nlohmann::json resp;
                    // opzionale: se payload è JSON e contiene path, in futuro si potrà filtrare
                    // per ora, ritorna sempre l'intera configurazione
                    resp["data"] = cfg.json();
                    mqtt.publish(t_cfg_state, resp.dump());
                    return; // già gestito, esci
                }
                // Da qui in poi serve JSON valido
                auto txt = std::string(payload.begin(), payload.end());
                auto js = nlohmann::json::parse(txt);
                // Config SET: applica immediatamente (senza staging/versioning)
                if (topic == t_cfg_set) {
                    // supporta: { path, value, merge }
                    bool merge = js.value("merge", true);
                    if (js.contains("path")) {
                        // set puntuale semplice: path non implementato in dettaglio, fallback all'intero
                        // TODO: implementare set-by-path
                        (void)merge;
                        // fallback: merge globale se value è oggetto
                        if (js.contains("value") && js["value"].is_object()) {
                            auto current = cfg.json();
                            for (auto it = js["value"].begin(); it != js["value"].end(); ++it) {
                                current[it.key()] = it.value();
                            }
                            cfg.json() = current;
                        }
                    } else if (js.contains("value")) {
                        if (merge && js["value"].is_object()) {
                            auto current = cfg.json();
                            for (auto it = js["value"].begin(); it != js["value"].end(); ++it) {
                                current[it.key()] = it.value();
                            }
                            cfg.json() = current;
                        } else {
                            cfg.json() = js["value"];
                        }
                    }
                    // Persist atomico
                    try { cfg.saveAtomic(); } catch (const std::exception& e) {
                        nlohmann::json res{{"status","error"},{"error",e.what()}};
                        mqtt.publish(t_result, res.dump());
                        return;
                    }
                    // Impact map minima: se cambia navigation.costmap.* → restart map_server
                    // (qui non calcoliamo il diff; per ora sempre restart map_server)
                    // stop-start map_server se in esecuzione
                    auto& svc = services["map_server"];
                    if (svc.pid > 0) ProcessManager::stop(svc.pid);
                    svc.pid = ProcessManager::start(svc.exec_path, {});
                    svc.running = (svc.pid > 0);
                    nlohmann::json res{{"status","ok"}};
                    mqtt.publish(t_result, res.dump());
                }
                // Comandi start/stop/restart → accodati per elaborazione nel main loop
                if (topic == t_cmd) {
                    std::string action = js.value("action", "");
                    std::string name = js.value("service", "map_server");
                    std::lock_guard<std::mutex> lk(cmd_mu);
                    cmd_queue.push_back(Cmd{action, name});
                }
            } catch (...) {
                // ignore parse errors
            }
        });

        // Avvio iniziale: usa system.supervisor.autostart se presente, altrimenti mantieni comportamento precedente (costmap)
        try {
            if (j.contains("system") && j["system"].contains("supervisor") && j["system"]["supervisor"].contains("autostart")) {
                const auto& autojs = j["system"]["supervisor"]["autostart"];
                if (autojs.is_object()) {
                    for (auto it = autojs.begin(); it != autojs.end(); ++it) {
                        const std::string svcName = it.key();
                        bool enabled = false;
                        try { enabled = it.value().get<bool>(); } catch (...) { enabled = false; }
                        if (!enabled) continue;
                        auto sit = services.find(svcName);
                        if (sit != services.end()) {
                            auto& s = sit->second;
                            s.pid = ProcessManager::start(s.exec_path, {});
                            s.running = (s.pid > 0);
                        }
                    }
                }
            } else {
                // fallback legacy: avvia costmap
                services["costmap_node"].pid = ProcessManager::start(services["costmap_node"].exec_path, {});
                services["costmap_node"].running = (services["costmap_node"].pid > 0);
            }
        } catch (...) {
            // in caso di formati inattesi, fallback a costmap
            services["costmap_node"].pid = ProcessManager::start(services["costmap_node"].exec_path, {});
            services["costmap_node"].running = (services["costmap_node"].pid > 0);
        }

        auto last_hb = std::chrono::steady_clock::now();
        while (!g_stop) {
            mqtt.loop(100);

            // Elabora comandi accodati (non dal callback)
            {
                std::deque<Cmd> local;
                {
                    std::lock_guard<std::mutex> lk(cmd_mu);
                    local.swap(cmd_queue);
                }
                for (const auto& c : local) {
                    auto it = services.find(c.service);
                    if (it == services.end()) continue;
                    auto& s = it->second;
                    if (c.action == "start") {
                        if (s.pid <= 0) s.pid = ProcessManager::start(s.exec_path, {});
                        s.running = (s.pid > 0);
                    } else if (c.action == "stop") {
                        if (s.pid > 0) { ProcessManager::stop(s.pid); s.pid = 0; }
                        s.running = false;
                    } else if (c.action == "restart") {
                        if (s.pid > 0) ProcessManager::stop(s.pid);
                        s.pid = ProcessManager::start(s.exec_path, {});
                        s.running = (s.pid > 0);
                    }
                }
            }
            // Watch status timeout (5s)
            auto now = std::chrono::steady_clock::now();
            for (auto& [name, s] : services) {
                // Reap non-blocking: se il figlio è uscito, pulisci lo stato ed eventualmente riavvia
                if (s.pid > 0) {
                    int status = 0;
                    pid_t r = waitpid(s.pid, &status, WNOHANG);
                    if (r == s.pid) {
                        // processo terminato
                        s.pid = 0;
                        s.running = false;
                        s.last_status = std::chrono::steady_clock::now();
                        // politica semplice: riavvia
                        s.pid = ProcessManager::start(s.exec_path, {});
                        s.running = (s.pid > 0);
                        s.last_status = std::chrono::steady_clock::now();
                    }
                }
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - s.last_status).count();
                if (s.running && elapsed > 5) {
                    // consideralo morto → restart semplice
                    if (s.pid > 0) ProcessManager::stop(s.pid);
                    s.pid = ProcessManager::start(s.exec_path, {});
                    s.running = (s.pid > 0);
                    s.last_status = std::chrono::steady_clock::now();
                }
            }
            // Heartbeat ogni 1s
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_hb).count() >= 1000) {
                nlohmann::json hb;
                hb["ts_ms"] = nowMs();
                nlohmann::json svcs = nlohmann::json::object();
                for (const auto& [name, s] : services) {
                    nlohmann::json js; js["running"] = s.running; js["pid"] = s.pid;
                    svcs[name] = js;
                }
                hb["services"] = svcs;
                mqtt.publish(t_sup_status, hb.dump(), 0, false);
                last_hb = now;
            }
            std::this_thread::sleep_for(50ms);
        }

        // Shutdown
        for (auto& [name, s] : services) {
            if (s.pid > 0) ProcessManager::stop(s.pid);
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[Supervisor][ERROR] " << e.what() << std::endl;
        return 1;
    }
}
