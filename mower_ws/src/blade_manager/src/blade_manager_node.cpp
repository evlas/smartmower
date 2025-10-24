/**
 * @file blade_manager_node.cpp
 * @brief Nodo ROS2 per gestione velocità lame in base allo stato del mower.
 */
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <chrono>
#include <string>
#include <vector>
#include <algorithm>

using std::placeholders::_1;
using namespace std::chrono_literals;

/**
 * @class BladeManager
 * @brief Decide e pubblica il comando lame in funzione dello stato corrente.
 */
class BladeManager : public rclcpp::Node {
public:
  /** @brief Costruttore: dichiara parametri, crea ROS pub/sub e timer. */
  BladeManager() : Node("blade_manager") {
    // Parameters
    state_topic_ = this->declare_parameter<std::string>("state_topic", "/mower/state");
    rpm_topic_ = this->declare_parameter<std::string>("rpm_topic", "/blades/rpm");
    blades_cmd_topic_ = this->declare_parameter<std::string>("blades_cmd_topic", "/blades/cmd");  // Compatibile con pico_control_hardware
    enable_states_ = this->declare_parameter<std::vector<std::string>>("enable_states", {"MOWING"});
    target_speed_ = this->declare_parameter<double>("target_speed", 0.8);
    min_command_threshold_ = this->declare_parameter<double>("min_command_threshold", 0.1);
    stall_rpm_threshold_ = this->declare_parameter<double>("stall_rpm_threshold", 300.0); // RPM
    stall_window_sec_ = this->declare_parameter<double>("stall_window_sec", 1.0); // seconds below threshold
    retry_delay_sec_ = this->declare_parameter<double>("retry_delay_sec", 2.0);
    max_retries_ = this->declare_parameter<int>("max_retries", 3);

    // Publishers / Subscribers
    pub_blades_cmd_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(blades_cmd_topic_, 10);
    pub_diag_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    pub_safety_error_ = this->create_publisher<std_msgs::msg::String>("/safety/error", 10);

    sub_state_ = this->create_subscription<std_msgs::msg::String>(state_topic_, 10,
      std::bind(&BladeManager::on_state, this, _1));
    sub_rpm_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(rpm_topic_, 10,
      std::bind(&BladeManager::on_rpm, this, _1));

    last_rpm_time_ = this->now();

    // Control loop
    timer_ = this->create_wall_timer(50ms, std::bind(&BladeManager::control_loop, this));

    RCLCPP_INFO(get_logger(), "blade_manager avviato. Sub: state=%s, rpm=%s. Pub: blades_cmd=%s (pico_control_hardware)",
      state_topic_.c_str(), rpm_topic_.c_str(), blades_cmd_topic_.c_str());
  }

private:
  /** @brief Callback stato macchina a stati. */
  void on_state(const std_msgs::msg::String::SharedPtr msg) {
    current_state_ = msg->data;
  }

  /** @brief Callback RPM lame [left,right]. */
  void on_rpm(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    // Expect [blade1_rpm, blade2_rpm]
    if (msg->data.size() >= 1) last_rpm_left_ = msg->data[0];
    if (msg->data.size() >= 2) last_rpm_right_ = msg->data[1];
    last_rpm_time_ = this->now();
  }

  /** @brief Ritorna true se lo stato corrente richiede lame ON. */
  bool state_requires_blades_on() const {
    return std::find(enable_states_.begin(), enable_states_.end(), current_state_) != enable_states_.end();
  }

  /** @brief Loop di controllo periodico: decide e pubblica il comando. */
  void control_loop() {
    const auto now = this->now();

    // Determine desired command based on state
    double desired = state_requires_blades_on() ? target_speed_ : 0.0;

    // TODO: Implementare rilevazione stall quando pico_control_hardware fornirà dati RPM
    // Per ora, assumiamo che le lame funzionino correttamente quando commanded ON
    // In futuro: utilizzare dati RPM da pico_control_hardware per rilevare stall

    // Publish command - formato compatibile con pico_control_hardware
    publish_blades(desired);
  }

  /** @brief Gestione stall (TODO: da implementare quando disponibili RPM affidabili). */
  void handle_stall(const rclcpp::Time & now) {
    // TODO: Implementare gestione stall quando avremo dati RPM da pico_control_hardware
    RCLCPP_WARN(get_logger(), "Rilevazione stall non ancora implementata - necessita dati RPM da pico_control_hardware");
  }

  /** @brief Pubblica comando lame come Float32MultiArray [left,right]. */
  void publish_blades(double speed) {
    std_msgs::msg::Float32MultiArray msg;
    msg.data = {static_cast<float>(speed), static_cast<float>(speed)};
    pub_blades_cmd_->publish(msg);
  }

  // Parameters
  std::string state_topic_;
  std::string rpm_topic_;
  std::string blades_cmd_topic_;
  std::vector<std::string> enable_states_;
  double target_speed_;
  double min_command_threshold_;
  double stall_rpm_threshold_;
  double stall_window_sec_;
  double retry_delay_sec_;
  int max_retries_;

  // State
  std::string current_state_;
  float last_rpm_left_{0.0f};
  float last_rpm_right_{0.0f};
  rclcpp::Time last_rpm_time_;
  bool below_threshold_{false};
  rclcpp::Time below_since_;
  bool in_retry_delay_{false};
  rclcpp::Time retry_start_time_;
  int stall_count_{0};

  // ROS interfaces
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_blades_cmd_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_safety_error_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_state_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_rpm_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BladeManager>());
  rclcpp::shutdown();
  return 0;
}
