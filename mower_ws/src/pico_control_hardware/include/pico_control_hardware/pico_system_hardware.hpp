#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/publisher_base.hpp>
#include <rclcpp_lifecycle/state.hpp>
 #include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <string>
#include <vector>
#include <memory>
#include <cstdint>
#include <array>
#include <mutex>
#include <thread>
#include <atomic>

namespace pico_control_hardware
{

class PicoSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PicoSystemHardware)

  // New API (Jazzy): prefer this to avoid deprecation warnings
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // serial helpers
  bool open_serial(const std::string & port, int baudrate);
  void close_serial();
  void write_frame(uint8_t msg_id, const std::vector<uint8_t> &payload);

  // helpers
  static uint16_t crc16_ccitt(const uint8_t *data, size_t len, uint16_t init = 0xFFFF);
  static void cobs_encode(const std::vector<uint8_t> &in, std::vector<uint8_t> &out);

  // params
  std::string serial_port_ = "/dev/ttyAMA0";
  int baudrate_ = 230400;
  double wheel_radius_ = 0.30;      // m
  double wheel_separation_ = 0.55;  // m
  double max_wheel_speed_ = 10.0;   // rad/s (normalize command)
  double accel_limit_ = 0.0;        // rad/s^2 (optional)
  int ticks_per_rev_motor_ = 12;    // ticks per motor revolution
  double gear_ratio_ = 185.0;       // motor:wheel ratio
  double m_per_tick_ = 0.0;         // computed from wheel_radius_ and ticks_per_rev_motor_*gear_ratio_

  // joints: left=0, right=1
  std::vector<double> pos_{0.0, 0.0};
  std::vector<double> vel_{0.0, 0.0};
  std::vector<double> cmd_{0.0, 0.0};

  // IO
  int fd_ = -1;
  uint8_t seq_ = 0;

  // IDs (keep aligned with pico firmware)
  static constexpr uint8_t MSG_CMD_DRIVE  = 0x10;
  static constexpr uint8_t MSG_CMD_BLADES = 0x11;
  static constexpr uint8_t MSG_CMD_RELAY  = 0x12;
  static constexpr uint8_t MSG_CMD_LIMITS = 0x13;

  // Telemetry IDs from Pico
  static constexpr uint8_t MSG_TLM_IMU   = 0x01;
  static constexpr uint8_t MSG_TLM_ODOM  = 0x02;
  static constexpr uint8_t MSG_TLM_SONAR = 0x03;
  static constexpr uint8_t MSG_TLM_BATT  = 0x04;
  static constexpr uint8_t MSG_TLM_EVENT = 0x05;
  static constexpr uint8_t MSG_TLM_BLADES_RPM = 0x06;

  // Topic interfaces for blades and relay
  rclcpp::SubscriptionBase::SharedPtr sub_blades_;
  rclcpp::SubscriptionBase::SharedPtr sub_relay_;
  std::mutex cmd_mutex_;
  std::array<float, 2> blade_cmd_{0.0f, 0.0f}; // normalized [0..1]
  uint8_t relay_state_ = 0; // 0=off, 1=on

  // Publishers for telemetry
  std::shared_ptr<rclcpp::PublisherBase> pub_imu_;
  std::shared_ptr<rclcpp::PublisherBase> pub_wheel_twist_;
  std::shared_ptr<rclcpp::PublisherBase> pub_sonar_left_;
  std::shared_ptr<rclcpp::PublisherBase> pub_sonar_center_;
  std::shared_ptr<rclcpp::PublisherBase> pub_sonar_right_;
  std::shared_ptr<rclcpp::PublisherBase> pub_batt_;
  std::shared_ptr<rclcpp::PublisherBase> pub_event_;
  std::shared_ptr<rclcpp::PublisherBase> pub_blades_rpm_;

  // RX thread and helpers
  std::thread rx_thread_;
  std::atomic<bool> stop_rx_{false};
  void run_rx_loop();

  // COBS decode helper
  static bool cobs_decode(const std::vector<uint8_t> &in, std::vector<uint8_t> &out);
};

} // namespace pico_control_hardware
