#include "pico_control_hardware/pico_system_hardware.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <cmath>
#include <chrono>

namespace pico_control_hardware
{

hardware_interface::CallbackReturn PicoSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  // Use new non-deprecated API from Jazzy and still leverage base parsing
  if (
    hardware_interface::HardwareComponentInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & hp = info_.hardware_parameters;

  // Accept both new and legacy parameter names
  if (hp.count("serial_port"))
    serial_port_ = hp.at("serial_port");
  else if (hp.count("port"))
    serial_port_ = hp.at("port");

  if (hp.count("baudrate"))
    baudrate_ = std::stoi(hp.at("baudrate"));
  else if (hp.count("baud"))
    baudrate_ = std::stoi(hp.at("baud"));

  if (hp.count("wheel_radius"))
    wheel_radius_ = std::stod(hp.at("wheel_radius"));
  if (hp.count("wheel_separation"))
    wheel_separation_ = std::stod(hp.at("wheel_separation"));
  if (hp.count("max_wheel_speed"))
    max_wheel_speed_ = std::stod(hp.at("max_wheel_speed"));
  if (hp.count("accel_limit"))
    accel_limit_ = std::stod(hp.at("accel_limit"));
  if (hp.count("ticks_per_rev_motor"))
    ticks_per_rev_motor_ = std::stoi(hp.at("ticks_per_rev_motor"));
  if (hp.count("gear_ratio"))
    gear_ratio_ = std::stod(hp.at("gear_ratio"));
  if (ticks_per_rev_motor_ <= 0 || gear_ratio_ <= 0.0) {
    m_per_tick_ = 0.0;
  } else {
    m_per_tick_ = (2.0 * M_PI * wheel_radius_) / (static_cast<double>(ticks_per_rev_motor_) * gear_ratio_);
  }

  // Expect exactly 2 joints (left, right)
  if (info_.joints.size() != 2)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("PicoSystemHardware"), "Expected 2 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  pos_.assign(2, 0.0);
  vel_.assign(2, 0.0);
  cmd_.assign(2, 0.0);

  // Validate interfaces
  for (const auto & joint : info_.joints)
  {
    bool has_cmd_vel = false;
    bool has_state_pos = false, has_state_vel = false;
    for (const auto & ci : joint.command_interfaces)
      has_cmd_vel |= (ci.name == hardware_interface::HW_IF_VELOCITY);
    for (const auto & si : joint.state_interfaces)
    {
      has_state_pos |= (si.name == hardware_interface::HW_IF_POSITION);
      has_state_vel |= (si.name == hardware_interface::HW_IF_VELOCITY);
    }
    if (!has_cmd_vel || !has_state_pos || !has_state_vel)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("PicoSystemHardware"),
        "Joint %s must expose cmd velocity and state position+velocity", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PicoSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  // Avoid calling deprecated base on_init(info). Manually set info_ and parse.
  info_ = info;

  // Parse parameters (support legacy and new names)
  if (info_.hardware_parameters.count("serial_port"))
    serial_port_ = info_.hardware_parameters.at("serial_port");
  else if (info_.hardware_parameters.count("port"))
    serial_port_ = info_.hardware_parameters.at("port");

  if (info_.hardware_parameters.count("baudrate"))
    baudrate_ = std::stoi(info_.hardware_parameters.at("baudrate"));
  else if (info_.hardware_parameters.count("baud"))
    baudrate_ = std::stoi(info_.hardware_parameters.at("baud"));

  if (info_.hardware_parameters.count("wheel_radius"))
    wheel_radius_ = std::stod(info_.hardware_parameters.at("wheel_radius"));
  if (info_.hardware_parameters.count("wheel_separation"))
    wheel_separation_ = std::stod(info_.hardware_parameters.at("wheel_separation"));
  if (info_.hardware_parameters.count("max_wheel_speed"))
    max_wheel_speed_ = std::stod(info_.hardware_parameters.at("max_wheel_speed"));
  if (info_.hardware_parameters.count("accel_limit"))
    accel_limit_ = std::stod(info_.hardware_parameters.at("accel_limit"));
  if (info_.hardware_parameters.count("ticks_per_rev_motor"))
    ticks_per_rev_motor_ = std::stoi(info_.hardware_parameters.at("ticks_per_rev_motor"));
  if (info_.hardware_parameters.count("gear_ratio"))
    gear_ratio_ = std::stod(info_.hardware_parameters.at("gear_ratio"));
  if (ticks_per_rev_motor_ <= 0 || gear_ratio_ <= 0.0) {
    m_per_tick_ = 0.0;
  } else {
    m_per_tick_ = (2.0 * M_PI * wheel_radius_) / (static_cast<double>(ticks_per_rev_motor_) * gear_ratio_);
  }

  // Expect exactly 2 joints (left, right)
  if (info_.joints.size() != 2)
  {
    RCLCPP_ERROR(rclcpp::get_logger("PicoSystemHardware"), "Expected 2 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  pos_.assign(2, 0.0);
  vel_.assign(2, 0.0);
  cmd_.assign(2, 0.0);

  // Validate interfaces
  for (const auto & joint : info_.joints)
  {
    bool has_cmd_vel = false;
    bool has_state_pos = false, has_state_vel = false;
    for (const auto & ci : joint.command_interfaces)
      has_cmd_vel |= (ci.name == hardware_interface::HW_IF_VELOCITY);
    for (const auto & si : joint.state_interfaces)
    {
      has_state_pos |= (si.name == hardware_interface::HW_IF_POSITION);
      has_state_vel |= (si.name == hardware_interface::HW_IF_VELOCITY);
    }
    if (!has_cmd_vel || !has_state_pos || !has_state_vel)
    {
      RCLCPP_ERROR(rclcpp::get_logger("PicoSystemHardware"), "Joint %s must expose cmd velocity and state position+velocity", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PicoSystemHardware::on_configure(const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PicoSystemHardware::on_activate(const rclcpp_lifecycle::State &)
{
  if (!open_serial(serial_port_, baudrate_))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  std::fill(pos_.begin(), pos_.end(), 0.0);
  std::fill(vel_.begin(), vel_.end(), 0.0);
  std::fill(cmd_.begin(), cmd_.end(), 0.0);

  // Create topic subscriptions for blades and relay commands
  if (auto node = hardware_interface::HardwareComponentInterface::get_node())
  {
    // Publishers (created on first activate)
    if (!pub_imu_)          pub_imu_ = node->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
    if (!pub_wheel_twist_)  pub_wheel_twist_ = node->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/wheel_odom/twist", 10);
    if (!pub_sonar_left_)   pub_sonar_left_ = node->create_publisher<sensor_msgs::msg::Range>("/sonar/left/scan", 10);
    if (!pub_sonar_center_) pub_sonar_center_ = node->create_publisher<sensor_msgs::msg::Range>("/sonar/center/scan", 10);
    if (!pub_sonar_right_)  pub_sonar_right_ = node->create_publisher<sensor_msgs::msg::Range>("/sonar/right/scan", 10);
    if (!pub_batt_)         pub_batt_ = node->create_publisher<sensor_msgs::msg::BatteryState>("/battery", 10);
    if (!pub_event_)        pub_event_ = node->create_publisher<std_msgs::msg::UInt16>("/events/pcf8574", 10);
    if (!pub_blades_rpm_)   pub_blades_rpm_ = node->create_publisher<std_msgs::msg::Float32MultiArray>("/blades/rpm", 10);

    sub_blades_ = node->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/blades/cmd", rclcpp::QoS(10),
      [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr msg)
      {
        if (msg->data.size() >= 2)
        {
          std::lock_guard<std::mutex> lk(cmd_mutex_);
          blade_cmd_[0] = std::clamp(msg->data[0], 0.0f, 1.0f);
          blade_cmd_[1] = std::clamp(msg->data[1], 0.0f, 1.0f);
        }
      });

    sub_relay_ = node->create_subscription<std_msgs::msg::Bool>(
      "/relay/cmd", rclcpp::QoS(10),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg)
      {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        relay_state_ = msg->data ? 1 : 0;
      });
  }

  // Start RX thread
  stop_rx_.store(false);
  rx_thread_ = std::thread(&PicoSystemHardware::run_rx_loop, this);
  // Send limits to firmware (optional)
  {
    std::vector<uint8_t> p(sizeof(float) * 2);
    float max_abs_speed = static_cast<float>(max_wheel_speed_);
    float accel = static_cast<float>(accel_limit_);
    std::memcpy(p.data(), &max_abs_speed, sizeof(float));
    std::memcpy(p.data() + 4, &accel, sizeof(float));
    write_frame(MSG_CMD_LIMITS, p);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PicoSystemHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  stop_rx_.store(true);
  if (rx_thread_.joinable()) rx_thread_.join();
  close_serial();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> PicoSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> states;
  // two joints: index 0 -> left, 1 -> right
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    states.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_[i]));
    states.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_[i]));
  }
  return states;
}

std::vector<hardware_interface::CommandInterface> PicoSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> cmds;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    cmds.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &cmd_[i]));
  }
  return cmds;
}

hardware_interface::return_type PicoSystemHardware::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  // TODO: read odometry/encoders from serial and set vel_ and pos_
  // For initial bringup, echo commands as measured velocities and integrate position
  const double dt = period.seconds();
  for (size_t i = 0; i < vel_.size(); ++i)
  {
    vel_[i] = cmd_[i];
    pos_[i] += vel_[i] * dt;
  }
  return hardware_interface::return_type::OK;
}

// --- RX loop: read COBS-framed packets terminated by 0x00, decode and publish ---
void PicoSystemHardware::run_rx_loop()
{
  std::vector<uint8_t> buf;
  buf.reserve(1024);
  std::vector<uint8_t> frame;
  frame.reserve(256);
  std::vector<uint8_t> decoded;
  decoded.reserve(256);

  auto node = hardware_interface::HardwareComponentInterface::get_node();
  const std::string odom_frame = "odom";
  const std::string base_frame = "base_link";

  while (!stop_rx_.load())
  {
    uint8_t tmp[256];
    ssize_t n = ::read(fd_, tmp, sizeof(tmp));
    if (n > 0)
    {
      buf.insert(buf.end(), tmp, tmp + n);
      // Extract frames split by 0x00
      for (;;)
      {
        auto it = std::find(buf.begin(), buf.end(), 0x00);
        if (it == buf.end()) break;
        frame.assign(buf.begin(), it);
        buf.erase(buf.begin(), it + 1);
        if (frame.empty()) continue;
        // COBS decode
        if (!cobs_decode(frame, decoded)) continue;
        if (decoded.size() < 1 + 1 + 1 + 4 + 2) continue;
        const uint8_t msg_id = decoded[0];
        const uint8_t len = decoded[1];
        // verify CRC
        const size_t payload_offset = 1 + 1 + 1 + 4;
        const size_t expected_total = payload_offset + len + 2;
        if (decoded.size() != expected_total) continue;
        uint16_t crc_calc = crc16_ccitt(decoded.data(), decoded.size() - 2);
        uint16_t crc_rx = static_cast<uint16_t>(decoded[decoded.size()-2]) |
                          (static_cast<uint16_t>(decoded.back()) << 8);
        if (crc_calc != crc_rx) continue;

        const uint8_t *payload = decoded.data() + payload_offset;
        const uint8_t seq = decoded[2];
        const uint32_t ts_ms = static_cast<uint32_t>(decoded[3]) |
                               (static_cast<uint32_t>(decoded[4])<<8) |
                               (static_cast<uint32_t>(decoded[5])<<16) |
                               (static_cast<uint32_t>(decoded[6])<<24);

        switch (msg_id)
        {
          case MSG_TLM_IMU:
          {
            if (len < 10 * 4) break;
            const float *f = reinterpret_cast<const float*>(payload);
            bool ori_valid = !(std::isnan(f[0]) || std::isnan(f[1]) || std::isnan(f[2]) || std::isnan(f[3]));
            // Log throttled every 5 seconds for debugging
            static auto last_log = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log).count() >= 5) {
              RCLCPP_INFO(rclcpp::get_logger("PicoSystemHardware"),
                "IMU frame: len=%d ts_ms=%u ori_valid=%d f[0]=%.3f f[1]=%.3f f[2]=%.3f f[3]=%.3f f[4]=%.3f f[5]=%.3f f[6]=%.3f f[7]=%.3f f[8]=%.3f f[9]=%.3f",
                len, ts_ms, ori_valid, f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7], f[8], f[9]);
              last_log = now;
            }
            sensor_msgs::msg::Imu msg;
            msg.header.stamp = rclcpp::Time(ts_ms * 1000000ULL);
            msg.header.frame_id = "imu_link";
            if (ori_valid)
            {
              msg.orientation.w = f[0]; msg.orientation.x = f[1]; msg.orientation.y = f[2]; msg.orientation.z = f[3];
              for (int i=0;i<9;++i) msg.orientation_covariance[i] = 0.0;
              msg.orientation_covariance[0]=0.02; msg.orientation_covariance[4]=0.02; msg.orientation_covariance[8]=0.05;
            }
            else
            {
              msg.orientation.w = 1.0; msg.orientation.x = 0.0; msg.orientation.y = 0.0; msg.orientation.z = 0.0;
              msg.orientation_covariance[0] = -1.0; msg.orientation_covariance[1]=0; msg.orientation_covariance[2]=0;
              msg.orientation_covariance[3] = 0;    msg.orientation_covariance[4] = 0;  msg.orientation_covariance[5]=0;
              msg.orientation_covariance[6] = 0;    msg.orientation_covariance[7] = 0;  msg.orientation_covariance[8]=0;
            }
            msg.linear_acceleration.x = std::isnan(f[4]) ? 0.0 : f[4];
            msg.linear_acceleration.y = std::isnan(f[5]) ? 0.0 : f[5];
            msg.linear_acceleration.z = std::isnan(f[6]) ? 0.0 : f[6];
            msg.angular_velocity.x = std::isnan(f[7]) ? 0.0 : f[7];
            msg.angular_velocity.y = std::isnan(f[8]) ? 0.0 : f[8];
            msg.angular_velocity.z = std::isnan(f[9]) ? 0.0 : f[9];
            for (int i=0;i<9;++i) msg.angular_velocity_covariance[i] = 0.0;
            msg.angular_velocity_covariance[0]=0.01; msg.angular_velocity_covariance[4]=0.01; msg.angular_velocity_covariance[8]=0.02;
            for (int i=0;i<9;++i) msg.linear_acceleration_covariance[i] = 0.0;
            msg.linear_acceleration_covariance[0]=0.1; msg.linear_acceleration_covariance[4]=0.1; msg.linear_acceleration_covariance[8]=0.2;
            auto pub = std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::Imu>>(pub_imu_);
            if (pub) pub->publish(msg);
            break;
          }
          case MSG_TLM_ODOM:
          {
            // New payload: <iif> => int32 dl, int32 dr, float dt (seconds)
            if (len < 4 + 4 + 4) break;
            int32_t dl_ticks = 0;
            int32_t dr_ticks = 0;
            float dt = 0.0f;
            std::memcpy(&dl_ticks, payload + 0, 4);
            std::memcpy(&dr_ticks, payload + 4, 4);
            std::memcpy(&dt,       payload + 8, 4);
            if (dt <= 0.0f || m_per_tick_ <= 0.0) break;
            const double dL = static_cast<double>(dl_ticks) * m_per_tick_;
            const double dR = static_cast<double>(dr_ticks) * m_per_tick_;
            const double v  = (dL + dR) * 0.5 / static_cast<double>(dt);
            const double vth = (dR - dL) / wheel_separation_ / static_cast<double>(dt);
            geometry_msgs::msg::TwistWithCovarianceStamped tw;
            tw.header.stamp = node->get_clock()->now();
            tw.header.frame_id = base_frame;
            tw.twist.twist.linear.x = v;
            tw.twist.twist.linear.y = 0.0;
            tw.twist.twist.angular.z = vth;
            // Basic covariances (tune in config): low vy, moderate vth
            tw.twist.covariance.fill(0.0);
            tw.twist.covariance[0] = 0.02;   // vx var
            tw.twist.covariance[7] = 0.10;   // vy var (unused)
            tw.twist.covariance[35]= 0.05;   // vth var
            auto pub = std::static_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>>(pub_wheel_twist_);
            if (pub) pub->publish(tw);
            break;
          }
          case MSG_TLM_SONAR:
          {
            if (len < 3 * 4) break;
            const float *f = reinterpret_cast<const float*>(payload);
            // Pubblica sempre 3 messaggi Range; mappa -1.0 (timeout) a 5.0 m (campo libero)
            sensor_msgs::msg::Range msgs[3];
            for (int i = 0; i < 3; ++i)
            {
              const float raw = f[i];
              const float max_r = 5.0f;
              const float v = (raw >= 0.0f) ? raw : max_r;
              msgs[i].header.stamp = node->get_clock()->now();
              msgs[i].header.frame_id = (i==0?"sonar_left_frame": i==1?"sonar_center_frame":"sonar_right_frame");
              msgs[i].radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
              msgs[i].field_of_view = 0.26f;
              msgs[i].min_range = 0.02f;
              msgs[i].max_range = max_r;
              msgs[i].range = std::max(msgs[i].min_range, std::min(v, msgs[i].max_range));
            }
            auto pl = std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::Range>>(pub_sonar_left_);
            auto pc = std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::Range>>(pub_sonar_center_);
            auto pr = std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::Range>>(pub_sonar_right_);
            if (pl) pl->publish(msgs[0]);
            if (pc) pc->publish(msgs[1]);
            if (pr) pr->publish(msgs[2]);
            break;
          }
          case MSG_TLM_BATT:
          {
            if (len < 2 * 4) break;
            const float *f = reinterpret_cast<const float*>(payload);
            // Sentinel: voltage -1.0 → skip publish
            if (f[0] >= 0.0f)
            {
              sensor_msgs::msg::BatteryState b;
              b.header.stamp = node->get_clock()->now();
              b.voltage = f[0];
              b.current = f[1];
              auto pub = std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::BatteryState>>(pub_batt_);
              if (pub) pub->publish(b);
            }
            break;
          }
          case MSG_TLM_BLADES_RPM:
          {
            if (len < 2 * 4) break;
            const float *f = reinterpret_cast<const float*>(payload);
            // Sentinel: NaN → skip publish
            if (!std::isnan(f[0]) && !std::isnan(f[1]))
            {
              std_msgs::msg::Float32MultiArray arr;
              arr.data.resize(2);
              arr.data[0] = f[0];
              arr.data[1] = f[1];
              auto pub = std::static_pointer_cast<rclcpp::Publisher<std_msgs::msg::Float32MultiArray>>(pub_blades_rpm_);
              if (pub) pub->publish(arr);
            }
            break;
          }
          case MSG_TLM_EVENT:
          {
            if (len < 2) break;
            uint16_t v = static_cast<uint16_t>(payload[0]) | (static_cast<uint16_t>(payload[1])<<8);
            std_msgs::msg::UInt16 e; e.data = v;
            auto pub = std::static_pointer_cast<rclcpp::Publisher<std_msgs::msg::UInt16>>(pub_event_);
            if (pub) pub->publish(e);
            break;
          }
          default: break;
        }
      }
    }
    else
    {
      // Sleep a bit to avoid busy loop when no data
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
}

// Basic COBS decoder (returns false on invalid input)
bool PicoSystemHardware::cobs_decode(const std::vector<uint8_t> &in, std::vector<uint8_t> &out)
{
  out.clear();
  size_t i = 0;
  while (i < in.size())
  {
    uint8_t code = in[i++];
    if (code == 0) return false;
    for (uint8_t j = 1; j < code; ++j)
    {
      if (i >= in.size()) return false;
      out.push_back(in[i++]);
    }
    if (code < 0xFF && i < in.size())
    {
      out.push_back(0x00);
    }
  }
  return true;
}

hardware_interface::return_type PicoSystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Map wheel velocities [rad/s] to normalized [-1, 1] for Pico firmware
  float left_norm = 0.0f;
  float right_norm = 0.0f;
  if (max_wheel_speed_ > 0.0)
  {
    left_norm = static_cast<float>(std::clamp(cmd_[0] / max_wheel_speed_, -1.0, 1.0));
    right_norm = static_cast<float>(std::clamp(cmd_[1] / max_wheel_speed_, -1.0, 1.0));
  }
  std::vector<uint8_t> payload(sizeof(float) * 2);
  std::memcpy(payload.data(), &left_norm, sizeof(float));
  std::memcpy(payload.data() + 4, &right_norm, sizeof(float));
  write_frame(MSG_CMD_DRIVE, payload);

   // Send blades command (2 floats [0..1]) and relay state (1 byte)
  {
    std::array<float, 2> blades_local;
    uint8_t relay_local;
    {
      std::lock_guard<std::mutex> lk(cmd_mutex_);
      blades_local = blade_cmd_;
      relay_local = relay_state_;
    }
    std::vector<uint8_t> p_blades(sizeof(float) * 2);
    std::memcpy(p_blades.data(), &blades_local[0], sizeof(float));
    std::memcpy(p_blades.data() + 4, &blades_local[1], sizeof(float));
    write_frame(MSG_CMD_BLADES, p_blades);

    std::vector<uint8_t> p_relay(1);
    p_relay[0] = relay_local;
    write_frame(MSG_CMD_RELAY, p_relay);
  }
  return hardware_interface::return_type::OK;
}

// ========== Serial helpers copied/adapted from pico_bridge ==========

uint16_t PicoSystemHardware::crc16_ccitt(const uint8_t *data, size_t len, uint16_t init)
{
  uint16_t crc = init;
  for (size_t i = 0; i < len; ++i)
  {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b)
    {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021; else crc <<= 1;
    }
  }
  return crc;
}

void PicoSystemHardware::cobs_encode(const std::vector<uint8_t> &in, std::vector<uint8_t> &out)
{
  out.clear();
  out.reserve(in.size() + 2);
  size_t code_index = 0;
  uint8_t code = 1;
  out.push_back(0);
  for (uint8_t byte : in)
  {
    if (byte == 0)
    {
      out[code_index] = code;
      code_index = out.size();
      out.push_back(0);
      code = 1;
    }
    else
    {
      out.push_back(byte);
      code++;
      if (code == 0xFF)
      {
        out[code_index] = code;
        code_index = out.size();
        out.push_back(0);
        code = 1;
      }
    }
  }
  out[code_index] = code;
  out.push_back(0x00);
}

bool PicoSystemHardware::open_serial(const std::string & port, int baudrate)
{
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("PicoSystemHardware"), "Failed to open serial %s", port.c_str());
    return false;
  }
  struct termios tio{};
  if (tcgetattr(fd_, &tio) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("PicoSystemHardware"), "tcgetattr failed");
    ::close(fd_); fd_ = -1; return false;
  }
  cfmakeraw(&tio);
  speed_t spd = B115200;
  switch (baudrate)
  {
    case 9600: spd = B9600; break;
    case 19200: spd = B19200; break;
    case 38400: spd = B38400; break;
    case 57600: spd = B57600; break;
    case 115200: spd = B115200; break;
    case 230400: spd = B230400; break;
    case 460800: spd = B460800; break;
    case 921600: spd = B921600; break;
    default: spd = B115200; break;
  }
  cfsetispeed(&tio, spd);
  cfsetospeed(&tio, spd);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~PARENB;
  tio.c_cflag &= ~CSTOPB;
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;
  tio.c_cc[VTIME] = 0;
  tio.c_cc[VMIN] = 0;
  if (tcsetattr(fd_, TCSANOW, &tio) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("PicoSystemHardware"), "tcsetattr failed");
    ::close(fd_); fd_ = -1; return false;
  }
  int flags = fcntl(fd_, F_GETFL, 0);
  fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);
  RCLCPP_INFO(rclcpp::get_logger("PicoSystemHardware"), "Serial opened %s @ %d", port.c_str(), baudrate);
  return true;
}

void PicoSystemHardware::close_serial()
{
  if (fd_ >= 0)
  {
    ::close(fd_);
    fd_ = -1;
  }
}

void PicoSystemHardware::write_frame(uint8_t msg_id, const std::vector<uint8_t> &payload)
{
  if (fd_ < 0) return;
  std::vector<uint8_t> buf;
  buf.reserve(7 + payload.size() + 2);
  static uint8_t seq_local = 0;
  buf.push_back(msg_id);
  buf.push_back(static_cast<uint8_t>(payload.size()));
  buf.push_back(seq_local++);
  // timestamp (ms since steady clock start). Not required to match bridge.
  uint32_t ts_ms = 0; // optional, set to 0
  buf.push_back(static_cast<uint8_t>(ts_ms & 0xFF));
  buf.push_back(static_cast<uint8_t>((ts_ms >> 8) & 0xFF));
  buf.push_back(static_cast<uint8_t>((ts_ms >> 16) & 0xFF));
  buf.push_back(static_cast<uint8_t>((ts_ms >> 24) & 0xFF));
  buf.insert(buf.end(), payload.begin(), payload.end());
  uint16_t crc = crc16_ccitt(buf.data(), buf.size());
  buf.push_back(static_cast<uint8_t>(crc & 0xFF));
  buf.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
  std::vector<uint8_t> encoded;
  cobs_encode(buf, encoded);
  (void)::write(fd_, encoded.data(), encoded.size());
}

} // namespace pico_control_hardware

PLUGINLIB_EXPORT_CLASS(pico_control_hardware::PicoSystemHardware, hardware_interface::SystemInterface)
