#include <iostream>
#include <csignal>
#include <chrono>
#include <thread>
#include <mutex>
#include <vector>
#include <cmath>
#include <nlohmann/json.hpp>
#include <cstdlib>

#include "config/config.h"
#include "mqtt/mqtt_client.h"

using namespace std::chrono_literals;

static volatile std::sig_atomic_t g_stop = 0;
static void handle_sigint(int) { g_stop = 1; }

struct Pose2D {
    double x{0}, y{0}, yaw{0};
};

struct ObstaclePoint {
    double x{0}, y{0}; // metri in frame mappa
    int64_t ts_ms{0};
};

class MapServer {
public:
    MapServer(int w, int h, double res, double infl)
        : width_(w), height_(h), res_(res), inflation_(infl) {
        occupancy_.assign(width_*height_, 0);
        coverage_.assign(width_*height_, 0);
        nogo_.assign(width_*height_, 0);
    }

    void clearDynamic() {
        std::lock_guard<std::mutex> lk(m_);
        for (auto &v : occupancy_) if (v == 100) v = 0;
    }

    // Occupancy
    void integrateObstacle(double wx, double wy) { markOccupiedWorld(wx, wy); }

    // Coverage: marca un disco di raggio rw in metri attorno al punto
    void markCoverageWorld(double wx, double wy, double rw_m) {
        std::lock_guard<std::mutex> lk(m_);
        const int r = std::max(1, (int)std::round(rw_m / res_));
        const int cx = (int)std::round(wx / res_);
        const int cy = (int)std::round(wy / res_);
        for (int dy = -r; dy <= r; ++dy) {
            for (int dx = -r; dx <= r; ++dx) {
                if (dx*dx + dy*dy <= r*r) setCellLayer(cx+dx, cy+dy, coverage_, 100);
            }
        }
    }

    // No-Go: rasterizza rettangolo axis-aligned in metri
    void rasterizeNoGoRect(double ox, double oy, double w, double h) {
        std::lock_guard<std::mutex> lk(m_);
        int x0 = (int)std::floor(ox / res_);
        int y0 = (int)std::floor(oy / res_);
        int x1 = (int)std::ceil((ox + w) / res_);
        int y1 = (int)std::ceil((oy + h) / res_);
        for (int y = y0; y <= y1; ++y) {
            for (int x = x0; x <= x1; ++x) setCellLayer(x, y, nogo_, 100);
        }
    }

    // Pubblica JSON multilayer con RLE per ciascun layer
    nlohmann::json toJsonLayers(int seq, double origin_x, double origin_y) const {
        nlohmann::json msg;
        msg["seq"] = seq;
        msg["timestamp"] = nowMs();
        msg["width"] = width_;
        msg["height"] = height_;
        msg["resolution_m"] = res_;
        msg["origin_x"] = origin_x;
        msg["origin_y"] = origin_y;
        auto rle_layer = [&](const std::vector<uint8_t>& data){
            nlohmann::json rle = nlohmann::json::array();
            if (data.empty()) return rle;
            uint8_t prev = data[0];
            int run = 1;
            for (size_t i = 1; i < data.size(); ++i) {
                uint8_t v = data[i];
                if (v == prev) { ++run; }
                else { rle.push_back({prev, run}); prev = v; run = 1; }
            }
            rle.push_back({prev, run});
            return rle;
        };
        msg["layers"]["occupancy"] = rle_layer(occupancy_);
        msg["layers"]["coverage"]  = rle_layer(coverage_);
        msg["layers"]["nogo"]       = rle_layer(nogo_);
        return msg;
    }

    int width() const { return width_; }
    int height() const { return height_; }
    double res() const { return res_; }

    // Sweep coverage lungo il segmento tra due pose con pennello r = deck_width/2
    void sweepCoverage(const Pose2D& prev, const Pose2D& cur, double deck_width_m, double sample_step_m = 0.02) {
        const double r = 0.5 * deck_width_m;
        const double dx = cur.x - prev.x;
        const double dy = cur.y - prev.y;
        const double len = std::hypot(dx, dy);
        const int steps = std::max(1, (int)std::ceil(len / std::max(sample_step_m, res_)));
        for (int i = 0; i <= steps; ++i) {
            const double t = (double)i / (double)steps;
            const double wx = prev.x + t * dx;
            const double wy = prev.y + t * dy;
            markCoverageWorld(wx, wy, r);
        }
    }

private:
    static int64_t nowMs() {
        return static_cast<int64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
    }
    void setCellLayer(int x, int y, std::vector<uint8_t>& layer, uint8_t val) {
        if (x < 0 || y < 0 || x >= width_ || y >= height_) return;
        layer[y*width_ + x] = val;
    }
    void markOccupiedWorld(double wx, double wy) {
        int cx = static_cast<int>(std::round(wx / res_));
        int cy = static_cast<int>(std::round(wy / res_));
        const int rad_cells = std::max(1, (int)std::round(inflation_ / res_));
        std::lock_guard<std::mutex> lk(m_);
        for (int dy = -rad_cells; dy <= rad_cells; ++dy) {
            for (int dx = -rad_cells; dx <= rad_cells; ++dx) {
                if (dx*dx + dy*dy <= rad_cells*rad_cells) setCellLayer(cx+dx, cy+dy, occupancy_, 100);
            }
        }
    }

    int width_{0}, height_{0};
    double res_{0.1};
    double inflation_{0.3};
    std::vector<uint8_t> occupancy_;
    std::vector<uint8_t> coverage_;
    std::vector<uint8_t> nogo_;
    mutable std::mutex m_;
};

static int64_t nowMs() {
    return static_cast<int64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
}

int main(int argc, char** argv) {
    try {
        std::signal(SIGINT, handle_sigint);
        std::signal(SIGTERM, handle_sigint);

        // Override path config: ENV SMARTMOWER_CONFIG e CLI --config/-c
        std::string config_path;
        if (const char* envp = std::getenv("SMARTMOWER_CONFIG")) {
            config_path = envp;
        }
        for (int i = 1; i < argc; ++i) {
            std::string a = argv[i];
            if (a == "-c" || a == "--config") {
                if (i+1 < argc) { config_path = argv[++i]; }
            } else if (a.rfind("--config=", 0) == 0) {
                config_path = a.substr(std::string("--config=").size());
            }
        }

        // Lettura configurazione
        smartmower::config::ConfigLoader cfg(config_path.empty() ? "/opt/smartmower/etc/config/robot_config.json" : config_path);
        auto topic_map_slam_opt = cfg.topicFrom("slam", "map");
        const auto topic_pose = cfg.slamPoseTopic();
        const auto topic_obst = cfg.visionObstacleTopic();
        const auto topic_pico = cfg.picoDataTopic();
        const auto topic_status = cfg.navCostmapStatusTopic();
        // Nuovo topic mappa multilayer
        std::string topic_nav_map = cfg.topicFrom("nav", "map").value_or(cfg.rootTopic() + std::string("/nav/map"));

        // Parametri
        // Lettura parametri: navigation.map.* con fallback a navigation.costmap.* (vedi ConfigLoader)
        // risoluzione da cutting_deck_width/4 (fallback a config navigation.map.resolution_m)
        double res = cfg.costmapResolutionM();
        double deck_w = 0.0;
        if (cfg.json().contains("system") && cfg.json()["system"].contains("hardware")) {
            const auto& hw = cfg.json()["system"]["hardware"];
            // Path attuale
            if (hw.contains("blade") && hw["blade"].contains("cutting_deck_width") && hw["blade"]["cutting_deck_width"].is_number()) {
                deck_w = hw["blade"]["cutting_deck_width"].get<double>();
            }
            // Fallback legacy
            else if (hw.contains("cutting_deck") && hw["cutting_deck"].contains("cutting_deck_width") && hw["cutting_deck"]["cutting_deck_width"].is_number()) {
                deck_w = hw["cutting_deck"]["cutting_deck_width"].get<double>();
            }
        }
        if (deck_w > 0.0) res = std::max(0.01, deck_w / 4.0);
        const double infl = cfg.inflationRadiusM();
        const int pub_hz = cfg.publishRateHz();
        const double obst_timeout_s = cfg.obstacleTimeoutS();
        std::cout << "[MapServer][cfg] res=" << res
                  << ", infl=" << infl
                  << ", pub_hz=" << pub_hz
                  << ", obst_timeout_s=" << obst_timeout_s
                  << " (nav.map fallback costmap)" << std::endl;

        // Dimensioni mappa: 100x100 m o da config slam_config.mapping
        double width_m = 100.0, height_m = 100.0;
        if (cfg.json().contains("slam_config") && cfg.json()["slam_config"].contains("mapping")) {
            const auto& m = cfg.json()["slam_config"]["mapping"];
            width_m = m.value("width_meters", width_m);
            height_m = m.value("height_meters", height_m);
        }
        int width = std::max(1, (int)std::round(width_m / res));
        int height = std::max(1, (int)std::round(height_m / res));
        MapServer grid(width, height, res, infl);
        Pose2D pose;
        std::vector<ObstaclePoint> obst_cache;

        // Parametri MQTT dal json (best-effort)
        auto j = cfg.json();
        std::string broker = "localhost";
        int port = 1883;
        std::string user;
        std::string pass;
        if (j.contains("mqtt")) {
            const auto& jm = j["mqtt"];
            broker = jm.value("broker", broker);
            port = jm.value("port", port);
            user = jm.value("username", std::string(""));
            pass = jm.value("password", std::string(""));
        }

        std::cout << "[MapServer] start. sub: " << (topic_map_slam_opt ? *topic_map_slam_opt : std::string("<none>")) << ", " << topic_pose
                  << ", " << topic_obst << ", " << topic_pico
                  << " | pub: " << topic_nav_map << "\n";

        smartmower::mqtt::MqttClient mqtt(broker, port, "smartmower_map", user, pass);
        if (!mqtt.connect()) {
            std::cerr << "[MapServer][ERROR] MQTT connect failed to " << broker << ":" << port << "\n";
            return 1;
        }
        // Sottoscrizioni
        if (topic_map_slam_opt) mqtt.subscribe(*topic_map_slam_opt);
        mqtt.subscribe(topic_pose);
        mqtt.subscribe(topic_obst);
        mqtt.subscribe(topic_pico);

        Pose2D prev_pose = pose;
        mqtt.setMessageCallback([&](const std::string& topic, const std::vector<uint8_t>& payload){
            try {
                auto txt = std::string(payload.begin(), payload.end());
                auto js = nlohmann::json::parse(txt);
                if (topic == topic_pose) {
                    // Formato reale SLAM: { position:{x,y,z}, orientation:{w,x,y,z}, timestamp }
                    // Fallback legacy: { x, y, yaw }
                    if (js.contains("position") && js["position"].is_object() &&
                        js.contains("orientation") && js["orientation"].is_object()) {
                        const auto& p = js["position"];
                        const auto& q = js["orientation"];
                        double px = p.value("x", pose.x);
                        double py = p.value("y", pose.y);
                        double qw = q.value("w", 1.0);
                        double qx = q.value("x", 0.0);
                        double qy = q.value("y", 0.0);
                        double qz = q.value("z", 0.0);
                        // yaw da quaternione (REP 103, Yaw attorno a Z)
                        double siny_cosp = 2.0 * (qw * qz + qx * qy);
                        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
                        double yaw = std::atan2(siny_cosp, cosy_cosp);
                        prev_pose = pose;
                        pose.x = px;
                        pose.y = py;
                        pose.yaw = yaw;
                        // aggiorna coverage con sweep tra prev e pose correnti
                        grid.sweepCoverage(prev_pose, pose, std::max(0.01, deck_w));
                    } else {
                        // legacy
                        prev_pose = pose;
                        pose.x = js.value("x", pose.x);
                        pose.y = js.value("y", pose.y);
                        pose.yaw = js.value("yaw", pose.yaw);
                        grid.sweepCoverage(prev_pose, pose, std::max(0.01, deck_w));
                    }
                } else if (topic == topic_obst) {
                    // atteso: { detections: [ {dx,dy}, ... ] } nel frame robot
                    if (js.contains("detections") && js["detections"].is_array()) {
                        for (const auto& d : js["detections"]) {
                            double dx = d.value("dx", 0.0);
                            double dy = d.value("dy", 0.0);
                            // trasformazione in mappa assumendo pose
                            double wx = pose.x + (std::cos(pose.yaw)*dx - std::sin(pose.yaw)*dy);
                            double wy = pose.y + (std::sin(pose.yaw)*dx + std::cos(pose.yaw)*dy);
                            obst_cache.push_back({wx, wy, nowMs()});
                        }
                    }
                } else if (topic == topic_pico) {
                    // atteso: { sonar: [ {angle_deg, dist_m} ... ] }
                    if (js.contains("sonar") && js["sonar"].is_array()) {
                        for (const auto& s : js["sonar"]) {
                            double ang = s.value("angle_deg", 0.0) * M_PI/180.0 + pose.yaw;
                            double dist = s.value("dist_m", 0.0);
                            if (dist > 0.05) {
                                double wx = pose.x + std::cos(ang)*dist;
                                double wy = pose.y + std::sin(ang)*dist;
                                obst_cache.push_back({wx, wy, nowMs()});
                            }
                        }
                    }
                } else if (topic_map_slam_opt && topic == *topic_map_slam_opt) {
                    // opzionale: integrare layer statico in futuro
                }
            } catch (...) {
                // ignora parse error
            }
        });

        const auto pub_period = std::chrono::milliseconds(1000 / std::max(1, pub_hz));
        const auto status_period = std::chrono::milliseconds(1000);
        auto next_pub = std::chrono::steady_clock::now();
        auto next_status = std::chrono::steady_clock::now();
        int seq = 0;

        while (!g_stop) {
            mqtt.loop(50);

            // purga ostacoli vecchi
            const int64_t now = nowMs();
            const int64_t to_ms = (int64_t)std::llround(obst_timeout_s * 1000.0);
            std::vector<ObstaclePoint> fresh;
            fresh.reserve(obst_cache.size());
            for (const auto& o : obst_cache) if (now - o.ts_ms <= to_ms) fresh.push_back(o);
            obst_cache.swap(fresh);

            // ricostruisci layer dinamico occupancy da ostacoli
            grid.clearDynamic();
            for (const auto& o : obst_cache) grid.integrateObstacle(o.x, o.y);

            if (std::chrono::steady_clock::now() >= next_pub) {
                // origin presa da slam_config.mapping origin_x/y
                double origin_x = 0.0, origin_y = 0.0;
                if (cfg.json().contains("slam_config") && cfg.json()["slam_config"].contains("mapping")) {
                    const auto& m = cfg.json()["slam_config"]["mapping"];
                    origin_x = m.value("origin_x", 0.0);
                    origin_y = m.value("origin_y", 0.0);
                }
                auto js = grid.toJsonLayers(seq++, origin_x, origin_y);
                mqtt.publish(topic_nav_map, js.dump());
                next_pub = std::chrono::steady_clock::now() + pub_period;
            }

            if (std::chrono::steady_clock::now() >= next_status) {
                nlohmann::json st;
                st["online"] = true;
                st["ts_ms"] = nowMs();
                st["width"] = grid.width();
                st["height"] = grid.height();
                st["note"] = "map_server alive";
                mqtt.publish(topic_status, st.dump());
                next_status = std::chrono::steady_clock::now() + status_period;
            }

            std::this_thread::sleep_for(10ms);
        }

        mqtt.disconnect();
        std::cout << "[MapServer] terminazione." << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[MapServer][ERROR] " << e.what() << std::endl;
        return 1;
    }
}
