#include "fusion/state.h"
#include <chrono>

namespace fusion {

State::State() {
    reset();
}

void State::reset() {
    timestamp = std::chrono::system_clock::now();
    position = Eigen::Vector3d::Zero();
    velocity_body = Eigen::Vector3d::Zero();
    orientation = Eigen::Quaterniond::Identity();
    accel_bias = Eigen::Vector3d::Zero();
    gyro_bias = Eigen::Vector3d::Zero();
    odom_scale = Eigen::Vector3d::Ones();
}

} // namespace fusion
