#include <iostream>
#include <csignal>
#include <vector>
#include <string>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>

#include "config/config_manager.h"
#include "mqtt/mqtt_client.h"
#include "mqtt/mqtt_topics.h"

using json = nlohmann::json;

static bool running = true;
void handle_signal(int) { running = false; }

struct ODParams {
    int filter_window_size = 5;             // smoothing sulla metrica area

    // Heuristics immagine (area edges)
    double min_area_ratio = 0.05;           // area contorno / frame
    int canny_low = 50;
    int canny_high = 150;
    int dilate_iter = 1;

    // SFM / Optical Flow params
    int max_corners = 150;
    double quality_level = 0.005;
    int min_distance = 15;
    int block_size = 5;
    double harris_k = 0.04;                 // attualmente non usato da GFTT standard
    int min_track_frames = 2;               // numero minimo frame per validare, qui usato come soglia di features
    double max_optical_flow_error = 100.0;  // errore LK massimo
    int min_points_threshold = 70;          // numero minimo di tracce valide per segnalare ostacolo
    double displacement_threshold = 2.0;    // pixel minimi di spostamento
};

struct AppCtx {
    mqtt::MqttClient* client;
    std::string outDataTopic;
    ODParams params;
    std::deque<double> area_hist;
    // Stato per LK
    cv::Mat prevGray;
    std::vector<cv::Point2f> prevPts;
};

static void onCameraMsg(const std::string& topic, const std::vector<uint8_t>& payload, void* user) {
    (void)topic;
    auto* ctx = static_cast<AppCtx*>(user);

    if (payload.empty()) return;
    cv::Mat buf(1, (int)payload.size(), CV_8UC1, (void*)payload.data());
    cv::Mat img = cv::imdecode(buf, cv::IMREAD_COLOR);
    if (img.empty()) return;

    cv::Mat gray, edges;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(5,5), 0);
    cv::Canny(gray, edges, ctx->params.canny_low, ctx->params.canny_high);
    if (ctx->params.dilate_iter > 0) {
        cv::dilate(edges, edges, cv::Mat(), cv::Point(-1,-1), ctx->params.dilate_iter);
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double best_area = 0.0;
    cv::Rect best_bbox;
    for (const auto& c : contours) {
        double a = cv::contourArea(c);
        if (a > best_area) {
            best_area = a;
            best_bbox = cv::boundingRect(c);
        }
    }

    double frame_area = (double)(img.cols * img.rows);
    double area_ratio = (frame_area > 0 ? best_area / frame_area : 0.0);

    // smoothing semplice
    ctx->area_hist.push_back(area_ratio);
    if ((int)ctx->area_hist.size() > ctx->params.filter_window_size)
        ctx->area_hist.pop_back();
    double area_smooth = 0.0;
    for (double v : ctx->area_hist) area_smooth += v;
    area_smooth /= std::max(1, (int)ctx->area_hist.size());

    // --- Optical Flow SFM ---
    int good_tracks = 0;
    double avg_disp = 0.0;
    if (ctx->prevGray.empty()) {
        // prime feature
        cv::goodFeaturesToTrack(gray, ctx->prevPts, ctx->params.max_corners, ctx->params.quality_level,
                                ctx->params.min_distance, cv::Mat(), ctx->params.block_size);
        ctx->prevGray = gray;
    } else {
        std::vector<cv::Point2f> nextPts;
        std::vector<unsigned char> status;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(ctx->prevGray, gray, ctx->prevPts, nextPts, status, err);
        double sum_disp = 0.0;
        int cnt = 0;
        for (size_t i = 0; i < ctx->prevPts.size(); ++i) {
            if (i < status.size() && status[i] && i < err.size() && err[i] <= ctx->params.max_optical_flow_error) {
                double dx = nextPts[i].x - ctx->prevPts[i].x;
                double dy = nextPts[i].y - ctx->prevPts[i].y;
                double d = std::sqrt(dx*dx + dy*dy);
                if (d >= ctx->params.displacement_threshold) {
                    good_tracks++;
                    sum_disp += d;
                    cnt++;
                }
            }
        }
        if (cnt > 0) avg_disp = sum_disp / (double)cnt;

        // aggiorna stato LK
        ctx->prevGray = gray;
        ctx->prevPts = nextPts;
        if ((int)ctx->prevPts.size() < ctx->params.min_points_threshold) {
            cv::goodFeaturesToTrack(gray, ctx->prevPts, ctx->params.max_corners, ctx->params.quality_level,
                                    ctx->params.min_distance, cv::Mat(), ctx->params.block_size);
        }
    }

    bool obstacle_area = area_smooth >= ctx->params.min_area_ratio;
    bool obstacle_motion = good_tracks >= ctx->params.min_points_threshold;
    bool obstacle = obstacle_area || obstacle_motion;
    // confidenza combinata semplice
    double conf_area = std::min(1.0, std::max(0.0, (area_smooth - ctx->params.min_area_ratio) / 0.25));
    double conf_motion = std::min(1.0, std::max(0.0, (double)good_tracks / std::max(1, ctx->params.min_points_threshold)));
    double confidence = std::max(conf_area, conf_motion);

    json j;
    j["timestamp_us"] = (uint64_t)std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    j["obstacle_detected"] = obstacle;
    j["confidence"] = confidence;
    j["area_ratio"] = area_smooth;
    j["image_size"] = { {"w", img.cols}, {"h", img.rows} };
    j["good_tracks"] = good_tracks;
    j["avg_track_disp_px"] = avg_disp;
    if (best_area > 0.0) {
        j["bbox"] = { {"x", best_bbox.x}, {"y", best_bbox.y}, {"w", best_bbox.width}, {"h", best_bbox.height} };
    }

    ctx->client->publish(ctx->outDataTopic, j.dump(), 1, false);
}

int main(int argc, char** argv) {
    std::string config_path = "/opt/smartmower/etc/config/robot_config.json";
    if (argc > 1) config_path = argv[1];

    config::ConfigManager cfg(config_path);
    if (!cfg.load()) {
        std::cerr << "Failed to load config: " << config_path << std::endl;
        return 1;
    }

    // MQTT settings
    std::string rootTopic = cfg.getString("mqtt.root_topic", "smartmower/");
    auto broker = cfg.getString("mqtt.broker", "localhost");
    int port = cfg.getInt("mqtt.port", 1883);
    auto username = cfg.getString("mqtt.username", "");
    auto password = cfg.getString("mqtt.password", "");

    // Topics
    auto camBase = cfg.getString("mqtt.topics.camera.base", "vision/camera");
    auto camSubs = cfg.getMap("mqtt.topics.camera.subtopics");
    std::string camData = rootTopic + camBase + "/" + camSubs["data"];

    auto obstBase = cfg.getString("mqtt.topics.obstacle.base", "vision/obstacle");
    auto obstSubs = cfg.getMap("mqtt.topics.obstacle.subtopics");
    mqtt::topics::TopicManager::getInstance().initialize(rootTopic, obstBase, obstSubs);
    std::string outData = mqtt::topics::TopicManager::getInstance().data();

    // Params (vision_config)
    ODParams params;
    // Preferisci la chiave sotto slam_config, fallback alla legacy root
    params.filter_window_size = cfg.getInt("slam_config.obstacle_detection.filter_window_size",
                                           cfg.getInt("obstacle_detection.filter_window_size", params.filter_window_size));
    // Parametri specifici visione (se presenti)
    params.min_area_ratio = cfg.getDouble("vision_config.detection.obstacle_detection.parameters.min_area_ratio", params.min_area_ratio);
    params.canny_low = cfg.getInt("vision_config.detection.obstacle_detection.parameters.canny_low", params.canny_low);
    params.canny_high = cfg.getInt("vision_config.detection.obstacle_detection.parameters.canny_high", params.canny_high);
    params.dilate_iter = cfg.getInt("vision_config.detection.obstacle_detection.parameters.dilate_iter", params.dilate_iter);

    // SFM parameters
    params.max_corners = cfg.getInt("vision_config.detection.obstacle_detection.sfm_parameters.max_corners", params.max_corners);
    params.quality_level = cfg.getDouble("vision_config.detection.obstacle_detection.sfm_parameters.quality_level", params.quality_level);
    params.min_distance = cfg.getInt("vision_config.detection.obstacle_detection.sfm_parameters.min_distance", params.min_distance);
    params.block_size = cfg.getInt("vision_config.detection.obstacle_detection.sfm_parameters.block_size", params.block_size);
    params.harris_k = cfg.getDouble("vision_config.detection.obstacle_detection.sfm_parameters.harris_k", params.harris_k);
    params.min_track_frames = cfg.getInt("vision_config.detection.obstacle_detection.sfm_parameters.min_track_frames", params.min_track_frames);
    params.max_optical_flow_error = cfg.getDouble("vision_config.detection.obstacle_detection.sfm_parameters.max_optical_flow_error", params.max_optical_flow_error);
    params.min_points_threshold = cfg.getInt("vision_config.detection.obstacle_detection.sfm_parameters.min_points_threshold", params.min_points_threshold);
    params.displacement_threshold = cfg.getDouble("vision_config.detection.obstacle_detection.sfm_parameters.displacement_threshold", params.displacement_threshold);

    // MQTT client
    mqtt::MqttClient client(broker, port, "vision_obstacle", username, password);
    if (!client.connect()) {
        std::cerr << "Failed to connect MQTT" << std::endl;
        return 1;
    }

    AppCtx app{ &client, outData, params, {}, cv::Mat(), {} };
    client.setMessageCallback(&onCameraMsg, &app);

    if (!client.subscribe(camData, 1)) {
        std::cerr << "Failed to subscribe camera data: " << camData << std::endl;
        return 1;
    }

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    auto last_status = std::chrono::steady_clock::now();
    while (running) {
        client.loop(50);
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_status).count() >= 5) {
            json st;
            st["module"] = "vision_obstacle";
            st["status"] = "running";
            st["ts"] = (uint64_t)std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            try {
                auto statusTopic = mqtt::topics::TopicManager::getInstance().status();
                client.publish(statusTopic, st.dump(), 0, false);
            } catch (...) { /* ignore if no status topic */ }
            last_status = now;
        }
    }

    client.disconnect();
    return 0;
}
