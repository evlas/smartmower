/**
 * @file relay_manager.cpp
 * @brief Nodo ROS2 per la gestione del relè di sicurezza in base allo stato.
 */
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <chrono>

using std::placeholders::_1;

/**
 * @class RelayManager
 * @brief Pubblica il comando relè in funzione dello stato della macchina e verifica allineamento.
 */
class RelayManager : public rclcpp::Node {
public:
  /** @brief Costruttore: parametri, pub/sub e log iniziale. */
  RelayManager() : Node("relay_manager") {
    // Parametri configurabili
    state_topic_ = this->declare_parameter<std::string>("state_topic", "/mower/state");
    relay_cmd_topic_ = this->declare_parameter<std::string>("relay_cmd_topic", "/relay/cmd");  // Compatibile con pico_control_hardware

    // Publisher per comandi relay (bool ON/OFF) - formato compatibile con pico_control_hardware
    relay_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      relay_cmd_topic_, 10);

    // Publisher per errori relay
    error_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/errors/relay", 10);

    // Subscriber to state machine state
    state_sub_ = this->create_subscription<std_msgs::msg::String>(
      state_topic_, 10,
      std::bind(&RelayManager::state_callback, this, _1));

    // Subscriber per stato attuale relay
    relay_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/relay/state", 10,
      std::bind(&RelayManager::relay_state_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Relay Manager started. Sub: %s, Pub: %s (pico_control_hardware), Error: /errors/relay, Relay State: /relay/state",
                state_topic_.c_str(), relay_cmd_topic_.c_str());
  }

private:
  /** @brief Callback stato macchina: decide ON/OFF del relè. */
  void state_callback(const std_msgs::msg::String::ConstSharedPtr msg) {
    std::string state = msg->data;
    bool should_relay_be_on = false;

    // Check if relay should be on for the current state
    if (state == "UNDOCKING" || state == "MOWING" ||
        state == "DOCKING" || state == "MANUAL_CONTROL") {
      should_relay_be_on = true;
    }
    // For IDLE, CHARGING, EMERGENCY_STOP, ERROR, PAUSED - relay should be off

    // Only publish if state changed
    if (should_relay_be_on != relay_state_) {
      relay_state_ = should_relay_be_on;
      publish_relay_state(should_relay_be_on);
      RCLCPP_INFO(this->get_logger(), "State: %s - Relay %s",
                 state.c_str(),
                 should_relay_be_on ? "ON" : "OFF");
    }
  }

  /** @brief Pubblica il comando relè e avvia controllo di allineamento. */
  void publish_relay_state(bool on) {
    std_msgs::msg::Bool msg;
    msg.data = on;
    relay_pub_->publish(msg);

    // Avvia il controllo di allineamento
    expected_relay_state_ = on;
    current_relay_state_ = !on;  // Forza disallineamento iniziale
    waiting_for_alignment_ = true;

    // Crea o riavvia timer per controllo timeout (500ms)
    if (alignment_timer_) {
      alignment_timer_->cancel();
    }
    alignment_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&RelayManager::check_alignment_timeout, this));
  }

  /** @brief Callback stato reale del relè, verifica allineamento. */
  void relay_state_callback(const std_msgs::msg::Bool::ConstSharedPtr msg) {
    current_relay_state_ = msg->data;

    // Se abbiamo un comando pending, controlla se si è allineato
    if (waiting_for_alignment_) {
      if (current_relay_state_ == expected_relay_state_) {
        // Relay si è allineato correttamente
        waiting_for_alignment_ = false;
        alignment_timer_->cancel();  // Cancella il timer
        publish_relay_error(false);  // Nessun errore

        RCLCPP_INFO(this->get_logger(), "Relay allineato correttamente a stato %s",
                   expected_relay_state_ ? "ON" : "OFF");
      }
    }
  }

  /** @brief Gestisce timeout di allineamento del relè. */
  void check_alignment_timeout() {
    if (waiting_for_alignment_) {
      // Timeout scaduto - relay non si è allineato
      waiting_for_alignment_ = false;
      publish_relay_error(true);  // Segnala errore

      RCLCPP_WARN(this->get_logger(), "ERRORE: Relay non si è allineato entro 500ms. Stato attuale: %s, Atteso: %s",
                 current_relay_state_ ? "ON" : "OFF",
                 expected_relay_state_ ? "ON" : "OFF");
    }
  }

  /** @brief Pubblica un flag di errore sul relè. */
  void publish_relay_error(bool error) {
    std_msgs::msg::Bool msg;
    msg.data = error;
    error_pub_->publish(msg);
  }

  std::string state_topic_;
  std::string relay_cmd_topic_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr relay_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr error_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr relay_state_sub_;
  rclcpp::TimerBase::SharedPtr alignment_timer_;

  bool relay_state_{false};
  bool current_relay_state_{false};
  bool expected_relay_state_{false};
  bool waiting_for_alignment_{false};
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RelayManager>());
  rclcpp::shutdown();
  return 0;
}
