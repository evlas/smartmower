#include "slam/mqtt_slam_node.h"
#include "slam/types.h"
#include "slam/visual_odometry.h"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

using json = nlohmann::json;

namespace slam {

MqttSlamNode::MqttSlamNode(const std::string& config_file) 
    : config_file_(config_file) {
    // Crea prima il componente SensorFusion, usato durante il caricamento config
    sensor_fusion_ = std::make_unique<SensorFusion>();
    visual_odometry_ = std::make_unique<VisualOdometry>();

    // Poi carica la configurazione
    if (!loadConfig()) {
        throw std::runtime_error("Failed to load configuration");
    }
    // Inizializza ObstacleDetector se non creato in loadConfig
    if (!obstacle_detector_) {
        // Default sicuri
        size_t win = 5; double min_d = 0.3; double max_d = 4.0;
        obstacle_detector_ = std::make_unique<ObstacleDetector>(win, min_d, max_d);
    }
    // Infine inizializza MQTT
    initMqtt();

    std::cout << "MqttSlamNode initialized successfully" << std::endl;
}

void MqttSlamNode::onObstacleMessage(const std::vector<uint8_t>& payload) {
    if (!obstacle_detector_) return;
    try {
        auto j = json::parse(payload.begin(), payload.end());

        bool detected = j.value("obstacle_detected", false);
        if (!detected) return;

        DetectedObstacle o{};
        // Stima distanza approssimata se non fornita
        double dist = 1.0;
        if (j.contains("estimated_distance_m") && j["estimated_distance_m"].is_number()) {
            dist = j["estimated_distance_m"].get<double>();
        }
        o.distance = dist;
        o.position = Vector2d(dist, 0.0);
        // Confidenza
        if (j.contains("confidence") && j["confidence"].is_number()) {
            o.confidence = j["confidence"].get<double>();
        } else if (j.contains("area_ratio") && j["area_ratio"].is_number()) {
            double ar = j["area_ratio"].get<double>();
            o.confidence = std::min(1.0, std::max(0.0, ar));
        } else {
            o.confidence = 0.5;
        }
        // Timestamp (preferisci microsecondi se presenti)
        if (j.contains("timestamp_us") && j["timestamp_us"].is_number_unsigned()) {
            o.timestamp = j["timestamp_us"].get<uint64_t>();
        } else if (j.contains("timestamp") && j["timestamp"].is_number_unsigned()) {
            o.timestamp = j["timestamp"].get<uint64_t>() * 1000ULL;
        } else {
            o.timestamp = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
        }

        obstacle_detector_->updateObstacleMap({o});
    } catch (const std::exception& e) {
        std::cerr << "Error processing obstacle message: " << e.what() << std::endl;
    }
}
MqttSlamNode::~MqttSlamNode() {
    stop();
}

bool MqttSlamNode::loadConfig() {
    try {
        std::ifstream f(config_file_);
        if (!f.is_open()) {
            throw std::runtime_error(std::string("Cannot open config file: ") + config_file_);
        }
        json config = json::parse(f);
        
        // Carica configurazione MQTT
        auto mqtt = config["mqtt"];
        mqtt_config_.broker = mqtt["broker"];
        mqtt_config_.port = mqtt["port"];
        mqtt_config_.client_id = mqtt.value("client_id", "slam_node");
        mqtt_config_.username = mqtt.value("username", "mower");
        mqtt_config_.password = mqtt.value("password", "smart");
        std::string root_topic = mqtt.value("root_topic", std::string("smartmower"));
        
        // Carica configurazione SLAM
        auto slam_config = config["slam_config"];
        
        // Topic MQTT
        auto topics = mqtt["topics"];
        auto slam_topics = topics["slam"]["subtopics"];
        auto pico_topics = topics["pico"];

        // Costruisci il data topic del bridge pico (fallback), può essere sovrascritto da slam_config.subscriptions.pico
        std::string pico_base = pico_topics["base"].get<std::string>();
        mqtt_config_.data_topic = root_topic + "/" + pico_base + "/data";
        mqtt_config_.status_topic = root_topic + "/" + pico_base + "/status";

        // Topic di pubblicazione (prefissa root_topic)
        std::string base_topic = root_topic + "/" + topics["slam"]["base"].get<std::string>();
        mqtt_config_.slam_pose_topic = base_topic + "/" + slam_topics["pose"].get<std::string>();
        // Topic mappa (opzionale): presente solo se configurato
        if (slam_topics.contains("map") && slam_topics["map"].is_string()) {
            mqtt_config_.slam_map_topic = base_topic + "/" + slam_topics["map"].get<std::string>();
        } else {
            mqtt_config_.slam_map_topic.clear();
        }

        // Topic di sottoscrizione secondo le nuove chiavi: vodometry, vobstacle, gps, pico
        if (slam_config.contains("subscriptions")) {
            auto subs = slam_config["subscriptions"];
            if (subs.contains("vodometry") && subs["vodometry"].is_string()) {
                mqtt_config_.odometry_topic = subs["vodometry"].get<std::string>();
            }
            if (subs.contains("vobstacle") && subs["vobstacle"].is_string()) {
                mqtt_config_.obstacle_topic = subs["vobstacle"].get<std::string>();
            }
            if (subs.contains("gps") && subs["gps"].is_string()) {
                mqtt_config_.gps_topic = subs["gps"].get<std::string>();
            }
            if (subs.contains("pico") && subs["pico"].is_string()) {
                mqtt_config_.data_topic = subs["pico"].get<std::string>();
            }
            if (subs.contains("pico_status") && subs["pico_status"].is_string()) {
                mqtt_config_.status_topic = subs["pico_status"].get<std::string>();
            }
        }
        // Fallback se non configurati
        if (mqtt_config_.odometry_topic.empty()) {
            mqtt_config_.odometry_topic = root_topic + "/vision/odometry/data";
        }
        if (mqtt_config_.obstacle_topic.empty()) {
            mqtt_config_.obstacle_topic = root_topic + "/vision/obstacle/data";
        }
        
        // Configurazione del filtro di Kalman
        if (slam_config.contains("initial_position")) {
            auto pos = slam_config["initial_position"];
            sensor_fusion_->setInitialPosition({
                pos["x"].get<double>(),
                pos["y"].get<double>(),
                pos["z"].get<double>()
            });
        }
        
        if (slam_config.contains("initial_orientation")) {
            auto orient = slam_config["initial_orientation"];
            // Converti i quaternioni in angoli di Eulero (roll, pitch, yaw)
            double w = orient["w"].get<double>();
            double x = orient["x"].get<double>();
            double y = orient["y"].get<double>();
            double z = orient["z"].get<double>();
            
            // Converti quaternione in angoli di Eulero (semplificato)
            // Attenzione: questa è una conversione semplificata e potrebbe aver bisogno di aggiustamenti
            double roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
            double pitch = asin(2.0 * (w * y - z * x));
            double yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
            
            sensor_fusion_->setInitialOrientation({
                roll,
                pitch,
                yaw
            });
        }
        
        // Configurazione dei parametri SLAM
        if (slam_config.contains("mapping")) {
            auto mapping = slam_config["mapping"];

            // Risoluzione (default 0.1 m/cella)
            double resolution = 0.1;
            if (mapping.contains("resolution") && mapping["resolution"].is_number()) {
                resolution = mapping["resolution"].get<double>();
            }
            sensor_fusion_->setMapResolution(resolution);

            // Calcolo robusto di width/height in metri
            auto get_dim_m = [&](const char* plain_key,
                                 const char* meters_key,
                                 const char* cells_key) -> double {
                if (mapping.contains(plain_key) && mapping[plain_key].is_number()) {
                    return mapping[plain_key].get<double>();
                }
                if (mapping.contains(meters_key) && mapping[meters_key].is_number()) {
                    return mapping[meters_key].get<double>();
                }
                if (mapping.contains(cells_key) && mapping[cells_key].is_number_integer()) {
                    return mapping[cells_key].get<int>() * resolution;
                }
                return 0.0;
            };

            double width_m = get_dim_m("width", "width_meters", "width_cells");
            double height_m = get_dim_m("height", "height_meters", "height_cells");
            sensor_fusion_->setMapSize(width_m, height_m);

            // Origine mappa
            double origin_x = 0.0;
            double origin_y = 0.0;
            if (mapping.contains("origin_x") && mapping["origin_x"].is_number()) {
                origin_x = mapping["origin_x"].get<double>();
            }
            if (mapping.contains("origin_y") && mapping["origin_y"].is_number()) {
                origin_y = mapping["origin_y"].get<double>();
            }
            sensor_fusion_->setMapOrigin(origin_x, origin_y);
        }

        // Configurazione magnetometro
        // Preferisci: slam_config.sensor_fusion.sensor_weights.magnetometer
        // Fallback:  slam_config.magnetometer.weight
        {
            double weight = 0.1;
            if (slam_config.contains("sensor_fusion") && slam_config["sensor_fusion"].is_object()) {
                auto sf = slam_config["sensor_fusion"];
                if (sf.contains("sensor_weights") && sf["sensor_weights"].is_object()) {
                    auto sw = sf["sensor_weights"];
                    if (sw.contains("magnetometer") && sw["magnetometer"].is_number()) {
                        weight = sw["magnetometer"].get<double>();
                    }
                }
                if (sf.contains("mag_declination_deg") && sf["mag_declination_deg"].is_number()) {
                    sensor_fusion_->setMagDeclinationDeg(sf["mag_declination_deg"].get<double>());
                }

                // Calibrazione magnetometro
                if (sf.contains("mag_calibration") && sf["mag_calibration"].is_object()) {
                    auto mc = sf["mag_calibration"];
                    // offset
                    if (mc.contains("offset") && mc["offset"].is_object()) {
                        Eigen::Vector3d off = Eigen::Vector3d::Zero();
                        if (mc["offset"].contains("x") && mc["offset"]["x"].is_number()) off.x() = mc["offset"]["x"].get<double>();
                        if (mc["offset"].contains("y") && mc["offset"]["y"].is_number()) off.y() = mc["offset"]["y"].get<double>();
                        if (mc["offset"].contains("z") && mc["offset"]["z"].is_number()) off.z() = mc["offset"]["z"].get<double>();
                        sensor_fusion_->setMagCalibrationOffset(off);
                    }
                    // scale
                    if (mc.contains("scale") && mc["scale"].is_object()) {
                        Eigen::Vector3d sca = Eigen::Vector3d::Ones();
                        if (mc["scale"].contains("x") && mc["scale"]["x"].is_number()) sca.x() = mc["scale"]["x"].get<double>();
                        if (mc["scale"].contains("y") && mc["scale"]["y"].is_number()) sca.y() = mc["scale"]["y"].get<double>();
                        if (mc["scale"].contains("z") && mc["scale"]["z"].is_number()) sca.z() = mc["scale"]["z"].get<double>();
                        sensor_fusion_->setMagCalibrationScale(sca);
                    }
                    // align_matrix 3x3
                    if (mc.contains("align_matrix") && mc["align_matrix"].is_array() && mc["align_matrix"].size() == 3) {
                        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
                        bool ok = true;
                        for (size_t r = 0; r < 3; ++r) {
                            if (!mc["align_matrix"][r].is_array() || mc["align_matrix"][r].size() != 3) { ok = false; break; }
                            for (size_t c = 0; c < 3; ++c) {
                                if (!mc["align_matrix"][r][c].is_number()) { ok = false; break; }
                                R(r, c) = mc["align_matrix"][r][c].get<double>();
                            }
                            if (!ok) break;
                        }
                        if (ok) sensor_fusion_->setMagCalibrationAlignMatrix(R);
                    }
                }
            }
            // fallback legacy
            if (slam_config.contains("magnetometer") && slam_config["magnetometer"].is_object()) {
                auto magcfg = slam_config["magnetometer"];
                if (magcfg.contains("weight") && magcfg["weight"].is_number()) {
                    weight = magcfg["weight"].get<double>();
                }
            }
            if (weight < 0.0) weight = 0.0;
            if (weight > 1.0) weight = 1.0;
            sensor_fusion_->setMagnetometerWeight(weight);
        }

        if (slam_config.contains("localization")) {
            auto loc = slam_config["localization"];
            sensor_fusion_->setParticleCount(loc["particle_count"]);
            sensor_fusion_->setMotionModelNoise({
                loc["motion_model_noise"][0],
                loc["motion_model_noise"][1],
                loc["motion_model_noise"][2]
            });
            sensor_fusion_->setSensorModelNoise(loc["sensor_model_noise"]);
        }

        // Configurazione obstacle detection (sonar)
        {
            size_t win = 5; double min_d = 0.3; double max_d = 4.0;
            if (slam_config.contains("obstacle_detection") && slam_config["obstacle_detection"].is_object()) {
                auto obc = slam_config["obstacle_detection"];
                if (obc.contains("filter_window_size") && obc["filter_window_size"].is_number_unsigned())
                    win = obc["filter_window_size"].get<size_t>();
                if (obc.contains("min_distance") && obc["min_distance"].is_number())
                    min_d = obc["min_distance"].get<double>();
                if (obc.contains("max_distance") && obc["max_distance"].is_number())
                    max_d = obc["max_distance"].get<double>();
            }
            obstacle_detector_ = std::make_unique<ObstacleDetector>(win, min_d, max_d);
        }

        // Parametri hardware per odometria ruote (encoder su albero motore)
        if (config.contains("system") && config["system"].is_object()) {
            auto sys = config["system"];
            if (sys.contains("hardware") && sys["hardware"].is_object()) {
                auto hw = sys["hardware"];
                // dimensions: wheel_diameter, wheel_track
                if (hw.contains("dimensions") && hw["dimensions"].is_object()) {
                    auto dim = hw["dimensions"];
                    if (dim.contains("wheel_diameter") && dim["wheel_diameter"].is_number()) {
                        double d = dim["wheel_diameter"].get<double>();
                        if (d > 0.0) pico_odom_params_.wheel_radius_m = 0.5 * d;
                    }
                    // Carreggiata: usa solo wheel_track
                    if (dim.contains("wheel_track") && dim["wheel_track"].is_number()) {
                        double wt = dim["wheel_track"].get<double>();
                        if (wt > 0.0) pico_odom_params_.wheel_base_m = wt;
                    }
                }
                // motors: gear ratio e pulses per rev sul motore
                if (hw.contains("motors") && hw["motors"].is_object()) {
                    auto mot = hw["motors"];
                    int pulses = pico_odom_params_.encoder_cpr; // default fallback
                    int gear = 1;
                    if (mot.contains("motors_encoder_pulses_per_rev") && mot["motors_encoder_pulses_per_rev"].is_number_integer()) {
                        pulses = mot["motors_encoder_pulses_per_rev"].get<int>();
                    }
                    if (mot.contains("motors_gear_ratio") && mot["motors_gear_ratio"].is_number_integer()) {
                        gear = mot["motors_gear_ratio"].get<int>();
                    }
                    long long cpr = static_cast<long long>(pulses) * static_cast<long long>(gear);
                    if (cpr > 0 && cpr <= std::numeric_limits<int>::max()) {
                        pico_odom_params_.encoder_cpr = static_cast<int>(cpr);
                    }
                }
            }
        }

        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error loading config: " << e.what() << std::endl;
        return false;
    }
}

void MqttSlamNode::initMqtt() {
    mqtt_client_ = std::make_unique<mqtt::MqttClient>(
        mqtt_config_.broker,
        mqtt_config_.port,
        mqtt_config_.client_id,
        mqtt_config_.username,
        mqtt_config_.password
    );
    
    // Imposta il callback per i messaggi
    mqtt_client_->setMessageCallback([this](const std::string& topic, 
                                         const std::vector<uint8_t>& payload) {
        onMessage(topic, payload);
    });
    
    // Connetti al broker
    if (!mqtt_client_->connect()) {
        throw std::runtime_error("Failed to connect to MQTT broker");
    }
    
    // Sottoscrizioni ai topic necessari
    mqtt_client_->subscribe(mqtt_config_.data_topic);
    if (!mqtt_config_.status_topic.empty()) {
        mqtt_client_->subscribe(mqtt_config_.status_topic);
    }
    // VO è obbligatoria: senza VO si considera errore di configurazione
    if (mqtt_config_.odometry_topic.empty()) {
        throw std::runtime_error("Visual Odometry MQTT topic is required (slam_config.subscriptions.vodometry)");
    }
    mqtt_client_->subscribe(mqtt_config_.odometry_topic);
    // Sottoscrizione al topic ostacoli da vision (opzionale)
    if (!mqtt_config_.obstacle_topic.empty()) {
        mqtt_client_->subscribe(mqtt_config_.obstacle_topic);
    }
    // Sottoscrizione al topic GPS (opzionale)
    if (!mqtt_config_.gps_topic.empty()) {
        mqtt_client_->subscribe(mqtt_config_.gps_topic);
    }
    
    std::cout << "MQTT initialized and subscribed to topics" << std::endl;
}

void MqttSlamNode::onMessage(const std::string& topic, 
                            const std::vector<uint8_t>& payload) {
    try {
        if (topic == mqtt_config_.data_topic) {
            onDataMessage(payload);
        } else if (topic == mqtt_config_.odometry_topic) {
            onOdometryMessage(payload);
        } else if (!mqtt_config_.status_topic.empty() && topic == mqtt_config_.status_topic) {
            onPicoStatusMessage(payload);
        } else if (!mqtt_config_.obstacle_topic.empty() && topic == mqtt_config_.obstacle_topic) {
            onObstacleMessage(payload);
        } else if (!mqtt_config_.gps_topic.empty() && topic == mqtt_config_.gps_topic) {
            onGpsMessage(payload);
        }
    } catch (const std::exception& e) {
        std::cerr << "Error processing message: " << e.what() << std::endl;
    }
}

void MqttSlamNode::onDataMessage(const std::vector<uint8_t>& payload) {
    // Il payload è JSON come pubblicato da pico .../data
    try {
        auto j = json::parse(payload.begin(), payload.end());

        // Estrai IMU e timestamp
        IMUData imu;
        imu.accel = Eigen::Vector3d(
            j["accel"]["x"].get<double>(),
            j["accel"]["y"].get<double>(),
            j["accel"]["z"].get<double>()
        );
        imu.gyro = Eigen::Vector3d(
            j["gyro"]["x"].get<double>(),
            j["gyro"]["y"].get<double>(),
            j["gyro"]["z"].get<double>()
        );
        // timestamp in ms -> µs
        imu.timestamp = static_cast<uint64_t>(j["timestamp"].get<uint64_t>()) * 1000ULL;

        // Magnetometro (opzionale)
        if (j.contains("mag") && j["mag"].is_object() &&
            j["mag"].contains("x") && j["mag"].contains("y") && j["mag"].contains("z")) {
            imu.mag = Eigen::Vector3d(
                j["mag"]["x"].get<double>(),
                j["mag"]["y"].get<double>(),
                j["mag"]["z"].get<double>()
            );
            imu.has_mag = true;
        }

        // Passa l'IMU alla fusione sensori
        sensor_fusion_->processIMU(imu);

        // Se disponibile, passa anche il magnetometro per l'heading
        if (imu.has_mag) {
            sensor_fusion_->processMagnetometer(imu.mag, imu.timestamp);
        }

        // Se necessario, in futuro: usare anche ultrasonic/power/safety per gating dei dati
        // Sonar (ultrasonic) dal bridge Pico
        if (obstacle_detector_ && j.contains("ultrasonic") && j["ultrasonic"].is_object()) {
            pico::SensorData sd{}; // zero-init
            auto us = j["ultrasonic"];
            // Mappatura: left, center, right
            if (us.contains("left") && us["left"].is_number())   sd.us_distances[0] = us["left"].get<float>();
            if (us.contains("center") && us["center"].is_number()) sd.us_distances[1] = us["center"].get<float>();
            if (us.contains("right") && us["right"].is_number()) sd.us_distances[2] = us["right"].get<float>();
            // timestamp in ms
            if (j.contains("timestamp") && j["timestamp"].is_number_unsigned()) {
                sd.timestamp = j["timestamp"].get<uint64_t>();
            }

            // Posa corrente del robot (x,y,theta)
            Vector3d pose = Vector3d::Zero();
            if (sensor_fusion_) {
                const auto& st = sensor_fusion_->getCurrentState();
                // Estrai yaw dal quaternione
                auto q = st.orientation;
                double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
                double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
                double yaw = std::atan2(siny_cosp, cosy_cosp);
                pose.x() = st.position.x();
                pose.y() = st.position.y();
                pose.z() = yaw;
            }
            obstacle_detector_->processUltrasonicData(sd, pose);
        }

        // Bumper -> aggiungi ostacolo immediato davanti come evento di safety
        if (obstacle_detector_ && j.contains("safety") && j["safety"].is_object()) {
            auto sf = j["safety"];
            bool bumper = sf.value("bumper", false);
            if (bumper) {
                DetectedObstacle o{};
                o.distance = 0.2; // 20 cm come contatto
                o.position = Vector2d(o.distance, 0.0);
                o.confidence = 1.0;
                o.timestamp = (j.contains("timestamp") && j["timestamp"].is_number_unsigned())
                                ? j["timestamp"].get<uint64_t>() * 1000ULL
                                : static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                      std::chrono::system_clock::now().time_since_epoch()).count());
                obstacle_detector_->updateObstacleMap({o});
            }
        }

        // Pubblica periodicamente la posa (ad esempio ogni N messaggi)
        static int count = 0;
        if (++count % 20 == 0) {
            std::cout << "[SLAM] onDataMessage: triggering publishPose()" << std::endl;
            publishPose(sensor_fusion_->getCurrentState());
        }
    } catch (const std::exception& e) {
        std::cerr << "Error processing data message: " << e.what() << std::endl;
    }
}

void MqttSlamNode::onImuMessage(const std::vector<uint8_t>& payload) {
    // Verifica che i dati ricevuti abbiano la dimensione corretta
    if (payload.size() < sizeof(pico::SensorData)) {
        std::cerr << "Invalid IMU message size: " << payload.size() 
                 << " bytes, expected at least " << sizeof(pico::SensorData) << std::endl;
        return;
    }
    
    try {
        // Interpreta il payload come struttura SensorData
        const auto* sensor_data = reinterpret_cast<const pico::SensorData*>(payload.data());
        
        // Estrai i dati IMU
        IMUData imu;
        imu.accel = Eigen::Vector3d(
            sensor_data->accel[0],
            sensor_data->accel[1],
            sensor_data->accel[2]
        );
        
        imu.gyro = Eigen::Vector3d(
            sensor_data->gyro[0],
            sensor_data->gyro[1],
            sensor_data->gyro[2]
        );
        
        // Converti il timestamp da ms a µs
        imu.timestamp = static_cast<uint64_t>(sensor_data->timestamp) * 1000;
        
        // Elabora i dati IMU
        sensor_fusion_->processIMU(imu);
        
        // Log di debug ogni 100 messaggi
        static int count = 0;
        if (++count % 100 == 0) {
            std::cout << "Processed IMU data - accel: (" 
                     << imu.accel.x() << ", " << imu.accel.y() << ", " << imu.accel.z() 
                     << "), gyro: ("
                     << imu.gyro.x() << ", " << imu.gyro.y() << ", " << imu.gyro.z()
                     << "), ts: " << imu.timestamp << "µs" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error processing IMU data: " << e.what() << std::endl;
    }
}

void MqttSlamNode::onGpsMessage(const std::vector<uint8_t>& payload) {
    auto j = json::parse(payload.begin(), payload.end());
    
    GPSData gps;
    gps.position.x() = j["lat"];
    gps.position.y() = j["lon"];
    gps.position.z() = j["alt"];
    
    // Inizializza la matrice di covarianza (esempio)
    gps.covariance = Eigen::Matrix3d::Identity() * 0.1;
    gps.timestamp = j["timestamp"];
    
    sensor_fusion_->processGPS(gps);
    
    // Pubblica la posizione aggiornata
    publishPose(sensor_fusion_->getCurrentState());
}

void MqttSlamNode::onCameraMessage(const std::vector<uint8_t>& payload) {
    // Decodifica l'immagine
    cv::Mat img = cv::imdecode(payload, cv::IMREAD_COLOR);
    if (img.empty()) {
        std::cerr << "Failed to decode image from MQTT payload" << std::endl;
        return;
    }
    
    // Elabora il frame per l'odometria visiva
    uint64_t ts_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());

    if (visual_odometry_) {
        auto vo = visual_odometry_->processFrame(img, ts_us);
        sensor_fusion_->processVisualOdometry(vo);
    }

    // Pubblica la traiettoria aggiornata periodicamente
    static int frame_count = 0;
    if (++frame_count % 10 == 0 && !mqtt_config_.slam_map_topic.empty()) {
        publishMap(sensor_fusion_->getTrajectory());
    }
}

void MqttSlamNode::onOdometryMessage(const std::vector<uint8_t>& payload) {
    try {
        auto j = json::parse(payload.begin(), payload.end());
        VisualOdometryData vo;
        // timestamp
        if (j.contains("timestamp_us") && j["timestamp_us"].is_number_unsigned()) {
            vo.timestamp = j["timestamp_us"].get<uint64_t>();
        } else if (j.contains("timestamp") && j["timestamp"].is_number_unsigned()) {
            // fallback millisecondi -> microsecondi
            vo.timestamp = j["timestamp"].get<uint64_t>() * 1000ULL;
        } else {
            vo.timestamp = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
        }

        // translation
        vo.translation = Vector3d::Zero();
        if (j.contains("translation") && j["translation"].is_object()) {
            auto t = j["translation"];
            if (t.contains("x")) vo.translation.x() = t["x"].get<double>();
            if (t.contains("y")) vo.translation.y() = t["y"].get<double>();
            if (t.contains("z")) vo.translation.z() = t["z"].get<double>();
        }

        // rotation quaternion
        if (j.contains("rotation_quat") && j["rotation_quat"].is_object()) {
            auto q = j["rotation_quat"];
            double w = q.value("w", 1.0);
            double x = q.value("x", 0.0);
            double y = q.value("y", 0.0);
            double z = q.value("z", 0.0);
            vo.rotation = Quaterniond(w, x, y, z);
        } else {
            vo.rotation = Quaterniond::Identity();
        }

        vo.confidence = j.value("confidence", 0.0);

        // Inoltra alla fusione sensori
        sensor_fusion_->processVisualOdometry(vo);

        // Pubblica anche la traiettoria periodicamente
        static int count = 0;
        if (++count % 10 == 0 && !mqtt_config_.slam_map_topic.empty()) {
            publishMap(sensor_fusion_->getTrajectory());
        }
    } catch (const std::exception& e) {
        std::cerr << "Error processing VO message: " << e.what() << std::endl;
    }
}

void MqttSlamNode::publishPose(const State& state) {
    json j;
    // Sanitizza posizione
    Vector3d pos = state.position;
    if (!std::isfinite(pos.x()) || !std::isfinite(pos.y()) || !std::isfinite(pos.z())) {
        pos = Vector3d::Zero();
    }
    // Clamp per evitare valori assurdi in pubblicazione
    auto clamp = [](double v) { return std::max(-1e3, std::min(1e3, v)); };
    j["position"] = {
        {"x", clamp(pos.x())},
        {"y", clamp(pos.y())},
        {"z", clamp(pos.z())}
    };
    
    // Normalizza quaternione e sanitizza
    auto q = state.orientation;
    if (!std::isfinite(q.w()) || !std::isfinite(q.x()) || !std::isfinite(q.y()) || !std::isfinite(q.z())) {
        q = Quaterniond::Identity();
    } else {
        q.normalize();
    }
    j["orientation"] = {
        {"w", q.w()},
        {"x", q.x()},
        {"y", q.y()},
        {"z", q.z()}
    };
    
    j["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    std::string payload = j.dump();
    bool ok = mqtt_client_->publish(
        mqtt_config_.slam_pose_topic,
        std::vector<uint8_t>(payload.begin(), payload.end())
    );
    std::cout << "[SLAM] publishPose -> topic: " << mqtt_config_.slam_pose_topic
              << ", bytes: " << payload.size()
              << ", ok: " << std::boolalpha << ok << std::endl;
}

void MqttSlamNode::publishMap(const std::vector<Vector3d>& trajectory) {
    json j;
    j["type"] = "trajectory";
    
    for (const auto& point : trajectory) {
        j["points"].push_back({
            {"x", point.x()},
            {"y", point.y()},
            {"z", point.z()}
        });
    }
    
    j["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    if (mqtt_config_.slam_map_topic.empty()) {
        return; // pubblicazione mappa disabilitata
    }
    std::string payload = j.dump();
    mqtt_client_->publish(mqtt_config_.slam_map_topic,
                         std::vector<uint8_t>(payload.begin(), payload.end()));
}

void MqttSlamNode::run() {
    if (running_) return;
    
    running_ = true;
    processing_thread_ = std::thread(&MqttSlamNode::processingLoop, this);
    
    std::cout << "SLAM node started" << std::endl;
    
    // Loop principale per gestire i messaggi MQTT
    while (running_) {
        mqtt_client_->loop(100);  // Timeout di 100ms
        
        // Qui puoi aggiungere altre operazioni periodiche
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void MqttSlamNode::stop() {
    if (!running_) return;
    
    running_ = false;
    if (processing_thread_.joinable()) {
        processing_thread_.join();
    }
    
    if (mqtt_client_) {
        mqtt_client_->disconnect();
    }
    
    std::cout << "SLAM node stopped" << std::endl;
}

void MqttSlamNode::processingLoop() {
    while (running_) {
        // Qui puoi eseguire l'elaborazione pesante in un thread separato
        // Ad esempio: loop closure, ottimizzazione della mappa, ecc.
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void MqttSlamNode::onPicoStatusMessage(const std::vector<uint8_t>& payload) {
    try {
        auto j = json::parse(payload.begin(), payload.end());
        // Attesi: { timestamp: <ms>, motors: [ {encoder:int, rpm:float, speed:float}, ... ] }
        if (!j.contains("motors") || !j["motors"].is_array()) return;

        uint64_t ts_ms = 0;
        if (j.contains("timestamp") && j["timestamp"].is_number_unsigned()) {
            ts_ms = j["timestamp"].get<uint64_t>();
        } else {
            ts_ms = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count());
        }

        int lidx = pico_odom_params_.left_motor_index;
        int ridx = pico_odom_params_.right_motor_index;
        if (static_cast<size_t>(lidx) >= j["motors"].size() || static_cast<size_t>(ridx) >= j["motors"].size()) return;

        auto mL = j["motors"][lidx];
        auto mR = j["motors"][ridx];
        if (!mL.contains("encoder") || !mR.contains("encoder")) return;

        int64_t encL = mL["encoder"].get<int64_t>() * pico_odom_params_.encoder_polarity_left;
        int64_t encR = mR["encoder"].get<int64_t>() * pico_odom_params_.encoder_polarity_right;

        if (!pico_enc_state_.has_prev) {
            pico_enc_state_.prev_left = encL;
            pico_enc_state_.prev_right = encR;
            pico_enc_state_.prev_ts_ms = ts_ms;
            pico_enc_state_.has_prev = true;
            return;
        }

        int64_t dL_counts = encL - pico_enc_state_.prev_left;
        int64_t dR_counts = encR - pico_enc_state_.prev_right;
        uint64_t dt_ms = (ts_ms > pico_enc_state_.prev_ts_ms) ? (ts_ms - pico_enc_state_.prev_ts_ms) : 0;
        if (dt_ms == 0) return;
        // Guardrail su dt troppo grande (clock jump) per evitare outlier
        if (dt_ms > 2000) { // >2s è sospetto per il nostro stream di test
            return;
        }

        pico_enc_state_.prev_left = encL;
        pico_enc_state_.prev_right = encR;
        pico_enc_state_.prev_ts_ms = ts_ms;

        // Validazione parametri odometrici
        if (pico_odom_params_.encoder_cpr <= 0 || pico_odom_params_.wheel_base_m <= 1e-6 ||
            !std::isfinite(pico_odom_params_.wheel_radius_m)) {
            return; // parametri non validi
        }
        // Conversione counts -> metri
        const double two_pi = 6.283185307179586;
        double revL = static_cast<double>(dL_counts) / static_cast<double>(pico_odom_params_.encoder_cpr);
        double revR = static_cast<double>(dR_counts) / static_cast<double>(pico_odom_params_.encoder_cpr);
        double dL = revL * two_pi * pico_odom_params_.wheel_radius_m;
        double dR = revR * two_pi * pico_odom_params_.wheel_radius_m;

        double d_center = 0.5 * (dL + dR);
        double d_theta = (dR - dL) / pico_odom_params_.wheel_base_m; // rotazione attorno a Z [rad]
        double dt_s = static_cast<double>(dt_ms) * 1e-3;

        // Estrai stato corrente per yaw
        const auto st = sensor_fusion_->getCurrentState();
        auto q = st.orientation;
        double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
        double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        // Integrazione pose odometrica (assumi movimento a velocità costante durante dt)
        double theta_mid = yaw + 0.5 * d_theta;
        double dx = d_center * std::cos(theta_mid);
        double dy = d_center * std::sin(theta_mid);

        PicoOdometryData po{};
        po.position.x() = st.position.x() + dx;
        po.position.y() = st.position.y() + dy;
        po.position.z() = yaw + d_theta; // salva theta in z
        po.velocity.x() = d_center / dt_s; // v
        po.velocity.y() = 0.0;
        po.velocity.z() = d_theta / dt_s;  // omega
        po.covariance = Eigen::Matrix3d::Identity();
        // Stima semplice covarianza: proporzionale ai moduli degli spostamenti
        double pos_var = std::max(1e-4, std::abs(d_center) * 0.01);
        double yaw_var = std::max(1e-4, std::abs(d_theta) * 0.01);
        po.covariance(0,0) = pos_var;
        po.covariance(1,1) = pos_var;
        po.covariance(2,2) = yaw_var;
        po.timestamp = ts_ms * 1000ULL; // µs

        // Scarta se non finito o valori assurdi
        if (!std::isfinite(po.position.x()) || !std::isfinite(po.position.y()) || !std::isfinite(po.position.z()) ||
            !std::isfinite(po.velocity.x()) || !std::isfinite(po.velocity.z()) ||
            std::abs(po.position.x()) > 1e3 || std::abs(po.position.y()) > 1e3) {
            return;
        }

        sensor_fusion_->processPicoOdometry(po);

        // Pubblica periodicamente la posa dopo integrazione encoder
        static int count = 0;
        if (++count % 10 == 0) {
            publishPose(sensor_fusion_->getCurrentState());
        }
    } catch (const std::exception& e) {
        std::cerr << "Error processing pico status message: " << e.what() << std::endl;
    }
}

} // namespace slam
