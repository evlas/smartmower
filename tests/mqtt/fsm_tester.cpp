#include <mosquitto.h>
#include <nlohmann/json.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <mutex>
#include <unistd.h>
#include <fstream>

using json = nlohmann::json;

struct Transition {
    std::string from;
    std::string to;
    std::string reason;
};

static std::atomic<bool> got_transition{false};
static std::mutex last_mtx;
static Transition last_tr;

static void on_message(struct mosquitto* /*m*/, void* /*userdata*/, const struct mosquitto_message* msg) {
    try {
        if (!msg || !msg->payload) return;
        std::string topic = msg->topic ? msg->topic : "";
        if (topic == "smartmower/status/transition") {
            auto j = json::parse(std::string((char*)msg->payload, msg->payloadlen));
            Transition t;
            t.from = j.value("from", "");
            t.to = j.value("to", "");
            t.reason = j.value("reason", "");
            {
                std::lock_guard<std::mutex> lk(last_mtx);
                last_tr = t;
            }
            got_transition.store(true);
            std::cout << "[TEST] TRANSITION: " << j.dump() << std::endl;
        } else if (topic == "smartmower/status/event") {
            std::cout << "[TEST] EVENT: " << std::string((char*)msg->payload, msg->payloadlen) << std::endl;
        } else if (topic == "smartmower/status/state") {
            std::cout << "[TEST] STATE: " << std::string((char*)msg->payload, msg->payloadlen) << std::endl;
        }
    } catch (...) {}
}

static bool wait_for_transition_to(const std::string& expected_to, int timeout_ms) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (got_transition.load()) {
            got_transition.store(false);
            std::lock_guard<std::mutex> lk(last_mtx);
            if (last_tr.to == expected_to) return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return false;
}

int main(int argc, char** argv) {
    // Opzione: lanciare la state_machine
    pid_t child_pid = -1;
    bool launch = (argc > 1 && std::string(argv[1]) == "--launch");
    if (launch) {
        const char* bin = getenv("SMARTMOWER_SM_BIN");
        std::string path = bin ? bin : "../src/state_machine/bin/state_machine";
        child_pid = fork();
        if (child_pid == 0) {
            execl(path.c_str(), path.c_str(), (char*)NULL);
            _exit(127);
        } else if (child_pid < 0) {
            std::cerr << "[TEST] fork() fallita" << std::endl;
            return 2;
        } else {
            std::this_thread::sleep_for(std::chrono::seconds(2)); // tempo per avvio
        }
    }

    // Carica config MQTT da robot_config.json
    std::string cfg_path = [](){
        const char* p = getenv("SMARTMOWER_CONFIG");
        return std::string(p ? p : "../src/config/robot_config.json");
    }();
    std::string broker = "localhost";
    int port = 1883;
    std::string username;
    std::string password;
    std::string cmd_topic;
    std::string pico_status_topic = "smartmower/bridge/pico/status";
    std::string gps_status_topic  = "smartmower/bridge/gps/status";
    std::string cam_status_topic  = "smartmower/vision/camera/status";
    try {
        std::ifstream in(cfg_path);
        if (in.good()) {
            json j; in >> j;
            if (j.contains("mqtt") && j["mqtt"].is_object()) {
                auto& mj = j["mqtt"];
                if (mj.contains("broker") && mj["broker"].is_string()) broker = mj["broker"].get<std::string>();
                if (mj.contains("port") && mj["port"].is_number()) port = mj["port"].get<int>();
                if (mj.contains("username") && mj["username"].is_string()) username = mj["username"].get<std::string>();
                if (mj.contains("password") && mj["password"].is_string()) password = mj["password"].get<std::string>();
                // Costruisci topic comandi se non presente in state_machine
                std::string root = "smartmower";
                std::string base = "state_machine";
                std::string sub  = "commands";
                // Readiness defaults basati su root
                std::string pico_base = "bridge/pico";
                std::string gps_base  = "bridge/gps";
                std::string cam_base  = "vision/camera";
                std::string status_sub = "status";
                if (mj.contains("root_topic") && mj["root_topic"].is_string()) root = mj["root_topic"].get<std::string>();
                if (mj.contains("topics") && mj["topics"].is_object()) {
                    auto& t = mj["topics"];
                    if (t.contains("state_machine") && t["state_machine"].is_object()) {
                        auto& smt = t["state_machine"];
                        if (smt.contains("base") && smt["base"].is_string()) base = smt["base"].get<std::string>();
                        if (smt.contains("subtopics") && smt["subtopics"].is_object()) {
                            auto& st = smt["subtopics"];
                            if (st.contains("commands") && st["commands"].is_string()) sub = st["commands"].get<std::string>();
                        }
                    }
                    if (t.contains("pico") && t["pico"].is_object()) {
                        auto& pt = t["pico"];
                        if (pt.contains("base") && pt["base"].is_string()) pico_base = pt["base"].get<std::string>();
                        if (pt.contains("subtopics") && pt["subtopics"].is_object()) {
                            auto& pst = pt["subtopics"];
                            if (pst.contains("status") && pst["status"].is_string()) status_sub = pst["status"].get<std::string>();
                        }
                    }
                    // status_sub è comune per semplicità; se volessimo separare, leggere ogni sezione
                    if (t.contains("gps") && t["gps"].is_object()) {
                        auto& gt = t["gps"];
                        if (gt.contains("base") && gt["base"].is_string()) gps_base = gt["base"].get<std::string>();
                    }
                    if (t.contains("camera") && t["camera"].is_object()) {
                        auto& ct = t["camera"];
                        if (ct.contains("base") && ct["base"].is_string()) cam_base = ct["base"].get<std::string>();
                    }
                }
                cmd_topic = root + "/" + base + "/" + sub;
                pico_status_topic = root + "/" + pico_base + "/" + status_sub;
                gps_status_topic  = root + "/" + gps_base  + "/" + status_sub;
                cam_status_topic  = root + "/" + cam_base  + "/" + status_sub;
            }
            if (j.contains("state_machine") && j["state_machine"].is_object()) {
                auto& smj = j["state_machine"];
                if (smj.contains("command_topic") && smj["command_topic"].is_string()) {
                    cmd_topic = smj["command_topic"].get<std::string>();
                }
            }
            // Fallback legacy readiness topic
            if (j.contains("pico_logging") && j["pico_logging"].is_object()) {
                auto& pl = j["pico_logging"];
                if (pl.contains("status_topic") && pl["status_topic"].is_string()) pico_status_topic = pl["status_topic"].get<std::string>();
            }
            if (j.contains("gps_logging") && j["gps_logging"].is_object()) {
                auto& gl = j["gps_logging"];
                if (gl.contains("status_topic") && gl["status_topic"].is_string()) gps_status_topic = gl["status_topic"].get<std::string>();
            }
            if (j.contains("vision") && j["vision"].is_object()) {
                auto& vc = j["vision"];
                if (vc.contains("status_topic") && vc["status_topic"].is_string()) cam_status_topic = vc["status_topic"].get<std::string>();
            }
        }
    } catch (...) {
        // fallback su env o default
    }
    // Env override opzionali
    if (const char* h = getenv("MQTT_HOST")) broker = h;
    if (const char* p = getenv("MQTT_PORT")) port = atoi(p);

    mosquitto_lib_init();
    struct mosquitto* m = mosquitto_new("fsm_tester", true, nullptr);
    if (!m) {
        std::cerr << "[TEST] mosquitto_new failed" << std::endl;
        return 1;
    }
    if (!username.empty()) {
        mosquitto_username_pw_set(m, username.c_str(), password.empty()? nullptr : password.c_str());
    }
    mosquitto_message_callback_set(m, on_message);
    if (mosquitto_connect(m, broker.c_str(), port, 60) != MOSQ_ERR_SUCCESS) {
        std::cerr << "[TEST] connect failed" << std::endl;
        return 1;
    }
    mosquitto_subscribe(m, nullptr, "smartmower/status/#", 0);

    // Loop thread
    std::thread loop_thr([&](){ mosquitto_loop_forever(m, -1, 1); });
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    auto pub = [&](const std::string& topic, const std::string& payload){
        mosquitto_publish(m, nullptr, topic.c_str(), (int)payload.size(), payload.data(), 0, false);
    };

    auto step = [&](const char* title, const std::function<void()>& action, const std::string& expect_to){
        std::cout << "\n[TEST] STEP: " << title << std::endl;
        action();
        bool ok = wait_for_transition_to(expect_to, 5000);
        if (!ok) {
            std::cerr << "[TEST] FAIL: atteso to=" << expect_to << std::endl;
            return false;
        }
        std::cout << "[TEST] OK: to=" << expect_to << std::endl;
        return true;
    };

    bool all_ok = true;

    // INIT -> IDLE (readiness)
    all_ok &= step("INIT readiness (pico/gps/cam) → IDLE", [&]{
        pub(pico_status_topic, "{\"status\":\"running\"}");
        pub(gps_status_topic,  "{\"fix\":1}");
        pub(cam_status_topic,  "{\"online\":true}");
    }, "IDLE");

    // IDLE -> UNDOCKING
    all_ok &= step("IDLE start → UNDOCKING", [&]{
        pub(cmd_topic, "start");
    }, "UNDOCKING");

    // UNDOCKING -> MOWING
    all_ok &= step("UNDOCKING complete → MOWING", [&]{
        pub(cmd_topic, "undocking");
    }, "MOWING");

    // MOWING -> PAUSED
    all_ok &= step("MOWING pause → PAUSED", [&]{
        pub(cmd_topic, "pause");
    }, "PAUSED");

    // PAUSED -> MOWING
    all_ok &= step("PAUSED resume → MOWING", [&]{
        pub(cmd_topic, "resume");
    }, "MOWING");

    // EMERGENCY_STOP da qualunque stato → EMERGENCY_STOP
    all_ok &= step("EMERGENCY STOP", [&]{
        pub(cmd_topic, "estop");
    }, "EMERGENCY_STOP");

    // RECOVER → IDLE
    all_ok &= step("RECOVER → IDLE", [&]{
        pub(cmd_topic, "recover");
    }, "IDLE");

    // IDLE -> MANUAL_CONTROL
    all_ok &= step("IDLE manual → MANUAL_CONTROL", [&]{
        pub(cmd_topic, "manual");
    }, "MANUAL_CONTROL");

    // MANUAL_CONTROL -> IDLE (exit manual)
    all_ok &= step("MANUAL exit → IDLE", [&]{
        pub(cmd_topic, "{\"manual\":{\"exit\":true}} ");
    }, "IDLE");

    // Cleanup
    mosquitto_disconnect(m);
    mosquitto_destroy(m);
    mosquitto_lib_cleanup();
    if (loop_thr.joinable()) loop_thr.detach();

    if (launch && child_pid > 0) {
        kill(child_pid, SIGTERM);
    }

    std::cout << "\n[TEST] RESULT: " << (all_ok ? "SUCCESS" : "FAILED") << std::endl;
    return all_ok ? 0 : 1;
}
