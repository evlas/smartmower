#include <iostream>
#include <csignal>
#include <chrono>
#include <thread>
#include <memory>
#include <unistd.h>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "config/config_manager.h"
#include "mqtt/mqtt_client.h"
#include "mqtt/mqtt_topics.h"

using json = nlohmann::json;

static bool running = true;
void handle_signal(int) { running = false; }

struct VOState {
    cv::Mat prevGray;
    std::vector<cv::Point2f> prevPts;
    cv::Mat K; // camera matrix if available
    bool hasK = false;
    // Config params
    int feat_max_corners = 500;
    double feat_quality_level = 0.01;
    double feat_min_distance = 8.0;
    int feat_block_size = 5;

    int of_win_size = 21;
    int of_max_level = 3;
    double of_term_eps = 0.03;

    double ransac_prob = 0.999;
    double ransac_thresh_px = 1.0;
    int ransac_min_inliers = 10;

    int track_min_features_refill = 200;
    int track_refill_corners = 500;

    int publish_every_n_frames = 1;
    double publish_min_confidence = 0.2;

    int frame_idx = 0;
};

struct CallbackCtx {
    VOState* state;
    mqtt::MqttClient* client;
    std::string outTopic;
};

static bool estimateMotion(const std::vector<cv::Point2f>& p0,
                           const std::vector<cv::Point2f>& p1,
                           const cv::Mat& K,
                           double ransac_prob, double ransac_thresh_px, int min_inliers,
                           cv::Mat& R, cv::Mat& t, double& conf) {
    if (p0.size() < 8 || p1.size() < 8) return false;
    cv::Mat E, mask;
    if (!K.empty()) {
        E = cv::findEssentialMat(p0, p1, K.at<double>(0,0), cv::Point2d(K.at<double>(0,2), K.at<double>(1,2)), cv::RANSAC, ransac_prob, ransac_thresh_px, mask);
    } else {
        E = cv::findEssentialMat(p0, p1, 1.0, cv::Point2d(0,0), cv::RANSAC, ransac_prob, ransac_thresh_px, mask);
    }
    if (E.empty()) return false;
    int inliers = 0;
    if (!K.empty()) {
        inliers = cv::recoverPose(E, p0, p1, K, R, t, mask);
    } else {
        // usa focale unitaria e centro (0,0)
        inliers = cv::recoverPose(E, p0, p1, R, t, 1.0, cv::Point2d(0,0), mask);
    }
    conf = (mask.empty() || p0.empty()) ? 0.0 : std::max(0.0, std::min(1.0, (double)inliers / (double)p0.size()));
    return inliers >= min_inliers;
}

static void onCameraFrameMsg(const std::string& topic, const std::vector<uint8_t>& payload, void* user) {
    (void)topic;
    auto* ctx = static_cast<CallbackCtx*>(user);
    VOState& st = *ctx->state;
    mqtt::MqttClient* c = ctx->client;
    const std::string& outTopic = ctx->outTopic;

    // Decode image
    cv::Mat buf(1, (int)payload.size(), CV_8UC1, const_cast<uint8_t*>(payload.data()));
    cv::Mat frame = cv::imdecode(buf, cv::IMREAD_GRAYSCALE);
    if (frame.empty()) return;

    std::vector<cv::Point2f> pts0, pts1;
    if (st.prevGray.empty()) {
        cv::goodFeaturesToTrack(frame, st.prevPts, st.feat_max_corners, st.feat_quality_level, st.feat_min_distance, cv::Mat(), st.feat_block_size);
        st.prevGray = frame;
        return;
    } else {
        std::vector<unsigned char> status;
        std::vector<float> err;
        std::vector<cv::Point2f> tracked;
        tracked = st.prevPts;
        cv::Size win(st.of_win_size, st.of_win_size);
        cv::TermCriteria term(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, st.of_term_eps);
        cv::calcOpticalFlowPyrLK(st.prevGray, frame, st.prevPts, tracked, status, err, win, st.of_max_level, term);
        pts0.reserve(st.prevPts.size());
        pts1.reserve(st.prevPts.size());
        for (size_t i = 0; i < st.prevPts.size(); ++i) {
            if (i < status.size() && status[i]) {
                pts0.push_back(st.prevPts[i]);
                pts1.push_back(tracked[i]);
            }
        }
        if ((int)pts0.size() < st.track_min_features_refill) {
            cv::goodFeaturesToTrack(frame, st.prevPts, st.track_refill_corners, st.feat_quality_level, st.feat_min_distance, cv::Mat(), st.feat_block_size);
            st.prevGray = frame;
            return;
        }
    }

    // Estimate motion
    cv::Mat R, t;
    double conf = 0.0;
    if (estimateMotion(pts0, pts1, st.hasK ? st.K : cv::Mat(), st.ransac_prob, st.ransac_thresh_px, st.ransac_min_inliers, R, t, conf)) {
        // Convert R to quaternion
        cv::Mat rvec;
        cv::Rodrigues(R, rvec);
        double angle = cv::norm(rvec);
        cv::Vec3d rv(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));
        cv::Vec3d axis = (angle > 1e-9) ? (rv / angle) : cv::Vec3d(0,0,1);
        double qw = std::cos(angle/2.0);
        double s = std::sin(angle/2.0);
        double qx = axis[0]*s;
        double qy = axis[1]*s;
        double qz = axis[2]*s;

        // Publish gating
        if ((st.frame_idx % std::max(1, st.publish_every_n_frames) == 0) && conf >= st.publish_min_confidence) {
            json j;
            j["timestamp_us"] = (uint64_t)std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            if (t.type() == CV_64F || (t.rows == 3 && t.cols == 1)) {
                j["translation"] = { {"x", t.at<double>(0)}, {"y", t.at<double>(1)}, {"z", t.at<double>(2)} };
            } else if (t.type() == CV_32F) {
                j["translation"] = { {"x", (double)t.at<float>(0)}, {"y", (double)t.at<float>(1)}, {"z", (double)t.at<float>(2)} };
            }
            j["rotation_quat"] = { {"w", qw}, {"x", qx}, {"y", qy}, {"z", qz} };
            j["confidence"] = conf;

            c->publish(outTopic, j.dump(), 1, false);
        }
    }

    // Prepare next
    st.prevGray = frame;
    st.prevPts = pts1;
    if ((int)st.prevPts.size() < st.track_min_features_refill) {
        cv::goodFeaturesToTrack(st.prevGray, st.prevPts, st.track_refill_corners, st.feat_quality_level, st.feat_min_distance, cv::Mat(), st.feat_block_size);
    }
    st.frame_idx++;
}

int main(int argc, char** argv) {
    std::string config_path = "/opt/smartmower/etc/config/robot_config.json";
    if (argc > 1) config_path = argv[1];

    config::ConfigManager cfg(config_path);
    if (!cfg.load()) {
        std::cerr << "Failed to load config: " << config_path << std::endl;
        return 1;
    }

    // MQTT setup
    std::string broker = cfg.getString("mqtt.broker", "localhost");
    int port = cfg.getInt("mqtt.port", 1883);
    std::string user = cfg.getString("mqtt.username", "");
    std::string pass = cfg.getString("mqtt.password", "");
    std::string clientId = std::string("vision_odometry_") + std::to_string(getpid());

    mqtt::MqttClient client(broker, port, clientId, user, pass);
    if (!client.connect()) {
        std::cerr << "Failed to connect MQTT" << std::endl;
        return 1;
    }

    // Topics
    std::string rootTopic = cfg.getString("mqtt.root_topic", "smartmower");
    auto camSubs = cfg.getMap("mqtt.topics.camera.subtopics");
    std::string camBase = cfg.getString("mqtt.topics.camera.base", "vision/camera");
    std::string camDataTopic = rootTopic + "/" + camBase + "/" + (camSubs.count("data") ? camSubs["data"] : std::string("data"));

    auto odoSubs = cfg.getMap("mqtt.topics.odometry.subtopics");
    std::string odoBase = cfg.getString("mqtt.topics.odometry.base", "vision/odometry");
    std::string odoDataTopic = rootTopic + "/" + odoBase + "/" + (odoSubs.count("data") ? odoSubs["data"] : std::string("data"));
    std::string odoStatusTopic = rootTopic + "/" + odoBase + "/" + (odoSubs.count("status") ? odoSubs["status"] : std::string("status"));

    // Camera intrinsics (optional)
    auto intr = cfg.getObject("vision_config.camera.intrinsics");
    VOState state;
    if (!intr.empty()) {
        double fx = intr.value("focal_length_x", 0.0);
        double fy = intr.value("focal_length_y", 0.0);
        double cx = intr.value("principal_point_x", 0.0);
        double cy = intr.value("principal_point_y", 0.0);
        if (fx > 0 && fy > 0) {
            state.K = (cv::Mat_<double>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
            state.hasK = true;
        }
    }

    // VO parameters from config
    state.feat_max_corners = cfg.getInt("vision_config.odometry.features.max_corners", state.feat_max_corners);
    state.feat_quality_level = cfg.getDouble("vision_config.odometry.features.quality_level", state.feat_quality_level);
    state.feat_min_distance = cfg.getDouble("vision_config.odometry.features.min_distance", state.feat_min_distance);
    state.feat_block_size = cfg.getInt("vision_config.odometry.features.block_size", state.feat_block_size);

    state.of_win_size = cfg.getInt("vision_config.odometry.optical_flow.win_size", state.of_win_size);
    state.of_max_level = cfg.getInt("vision_config.odometry.optical_flow.max_level", state.of_max_level);
    state.of_term_eps = cfg.getDouble("vision_config.odometry.optical_flow.term_eps", state.of_term_eps);

    state.ransac_prob = cfg.getDouble("vision_config.odometry.ransac.prob", state.ransac_prob);
    state.ransac_thresh_px = cfg.getDouble("vision_config.odometry.ransac.threshold_px", state.ransac_thresh_px);
    state.ransac_min_inliers = cfg.getInt("vision_config.odometry.ransac.min_inliers", state.ransac_min_inliers);

    state.track_min_features_refill = cfg.getInt("vision_config.odometry.tracking.min_features_refill", state.track_min_features_refill);
    state.track_refill_corners = cfg.getInt("vision_config.odometry.tracking.refill_corners", state.track_refill_corners);

    state.publish_every_n_frames = cfg.getInt("vision_config.odometry.publish.every_n_frames", state.publish_every_n_frames);
    state.publish_min_confidence = cfg.getDouble("vision_config.odometry.publish.min_confidence", state.publish_min_confidence);

    // Subscribe to camera frames
    if (!client.subscribe(camDataTopic, 1)) {
        std::cerr << "Failed to subscribe camera topic: " << camDataTopic << std::endl;
        return 1;
    }

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    auto last_status = std::chrono::steady_clock::now();

    // Bind callback C con contesto
    CallbackCtx ctx;
    ctx.state = &state;
    ctx.client = &client;
    ctx.outTopic = odoDataTopic;
    client.setMessageCallback(&onCameraFrameMsg, &ctx);

    // Rebind with proper context: we can't directly pass lambdas with capture to C callback, so we simulate via static storage
    // We'll instead poll in a loop and process via a static global; to keep it simple, process inside loop via a small queue is overkill,
    // so we rely on mosquitto's internal invoking of on_message and the lambda with captures above isn't directly usable.
    // To keep the implementation straightforward, we implement a small static forwarder below.

    // Implement a simple polling loop invoking mosquitto_loop
    while (running) {
        client.loop(10);
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_status).count() >= 1) {
            json st;
            st["online"] = true;
            st["has_intrinsics"] = state.hasK;
            st["features_prev"] = (int)state.prevPts.size();
            client.publish(odoStatusTopic, st.dump(), 0, false);
            last_status = now;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    client.disconnect();
    return 0;
}
