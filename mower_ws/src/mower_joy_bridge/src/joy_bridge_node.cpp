/**
 * @file joy_bridge_node.cpp
 * @brief Nodo ROS2 C++ che collega i pulsanti del joystick (DS4) con la macchina a stati e gli attuatori.
 *
 * Funzionalità:
 * - Pulsante X: toggle modalità manuale inviando eventi `MANUAL_ON`/`MANUAL_OFF` su `/mower/event`.
 * - Solo in modalità manuale (`/mower/state == MANUAL_CONTROL`):
 *   - Pulsante Cerchio: toggle lame ON al 100% (1.0) / OFF pubblicando su `/blades/cmd` (`Float32MultiArray`).
 *   - Pulsante Quadrato: toggle relè pubblicando su `/relay/cmd` (`Bool`).
 *
 * Parametri:
 * - `joy_topic` (string, default `/joy`)
 * - `state_topic` (string, default `/mower/state`)
 * - `event_topic` (string, default `/mower/event`)
 * - `blades_cmd_topic` (string, default `/blades/cmd`)
 * - `relay_cmd_topic` (string, default `/relay/cmd`)
 * - `button_x_index` (int, default 0)
 * - `button_circle_index` (int, default 1)
 * - `button_square_index` (int, default 3)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <string>
#include <vector>

/**
 * @class MowerJoyBridge
 * @brief Implementazione del nodo di bridge joystick.
 */
class MowerJoyBridge : public rclcpp::Node {
public:
  /**
   * @brief Costruttore: dichiara parametri, crea publisher/subscriber.
   */
  MowerJoyBridge() : Node("mower_joy_bridge") {
    // Parametri
    joy_topic_ = declare_parameter<std::string>("joy_topic", "/joy");
    state_topic_ = declare_parameter<std::string>("state_topic", "/mower/state");
    event_topic_ = declare_parameter<std::string>("event_topic", "/mower/event");
    blades_cmd_topic_ = declare_parameter<std::string>("blades_cmd_topic", "/blades/cmd");
    relay_cmd_topic_ = declare_parameter<std::string>("relay_cmd_topic", "/relay/cmd");

    button_x_index_ = declare_parameter<int>("button_x_index", 0);
    button_circle_index_ = declare_parameter<int>("button_circle_index", 1);
    button_square_index_ = declare_parameter<int>("button_square_index", 3);

    // Publisher
    pub_event_ = create_publisher<std_msgs::msg::String>(event_topic_, 10);
    pub_blades_cmd_ = create_publisher<std_msgs::msg::Float32MultiArray>(blades_cmd_topic_, 10);
    pub_relay_cmd_ = create_publisher<std_msgs::msg::Bool>(relay_cmd_topic_, 10);

    // Subscriber
    sub_joy_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, 10, std::bind(&MowerJoyBridge::on_joy, this, std::placeholders::_1));

    sub_state_ = create_subscription<std_msgs::msg::String>(
      state_topic_, 10, std::bind(&MowerJoyBridge::on_state, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "mower_joy_bridge avviato. Sub: joy=%s state=%s -> Pub: event=%s blades=%s relay=%s",
                joy_topic_.c_str(), state_topic_.c_str(), event_topic_.c_str(),
                blades_cmd_topic_.c_str(), relay_cmd_topic_.c_str());
  }

private:
  /**
   * @brief Callback dello stato macchina a stati.
   * @param msg Messaggio di stato corrente.
   */
  void on_state(const std_msgs::msg::String::SharedPtr msg) {
    current_state_ = msg->data;
    in_manual_ = (current_state_ == "MANUAL_CONTROL");
  }

  /**
   * @brief Invia un evento alla macchina a stati.
   * @param e Nome dell'evento.
   */
  void publish_event(const std::string &e) {
    std_msgs::msg::String msg;
    msg.data = e;
    pub_event_->publish(msg);
    RCLCPP_INFO(get_logger(), "Evento pubblicato: %s", e.c_str());
  }

  /**
   * @brief Pubblica comando lame.
   * @param on Se true, imposta entrambe le lame a 1.0; altrimenti 0.0.
   */
  void publish_blades(bool on) {
    std_msgs::msg::Float32MultiArray msg;
    const float v = on ? 1.0f : 0.0f;
    msg.data = {v, v};
    pub_blades_cmd_->publish(msg);
    RCLCPP_INFO(get_logger(), "Lame %s (%.1f)", on ? "ON" : "OFF", v);
  }

  /**
   * @brief Pubblica comando relè.
   * @param on Stato relè desiderato.
   */
  void publish_relay(bool on) {
    std_msgs::msg::Bool msg;
    msg.data = on;
    pub_relay_cmd_->publish(msg);
    RCLCPP_INFO(get_logger(), "Relay %s", on ? "ON" : "OFF");
  }

  /**
   * @brief Callback del joystick: gestisce edge-detection e toggle funzioni.
   * @param joy Messaggio Joy.
   */
  void on_joy(const sensor_msgs::msg::Joy::SharedPtr joy) {
    // Verifica dimensioni
    const auto &buttons = joy->buttons;

    auto pressed = [&](int idx) -> bool {
      return idx >= 0 && idx < static_cast<int>(buttons.size()) && buttons[idx] != 0;
    };

    // Edge detection per X
    bool x_now = pressed(button_x_index_);
    if (x_now && !last_x_) {
      // Toggle modalità manuale via eventi
      if (!in_manual_) {
        publish_event("MANUAL_ON");
      } else {
        publish_event("MANUAL_OFF");
      }
    }

    // Solo in manuale: gestisci Cerchio (lame) e Quadrato (relè)
    if (in_manual_) {
      bool circle_now = pressed(button_circle_index_);
      if (circle_now && !last_circle_) {
        blades_on_ = !blades_on_;
        publish_blades(blades_on_);
      }

      bool square_now = pressed(button_square_index_);
      if (square_now && !last_square_) {
        relay_on_ = !relay_on_;
        publish_relay(relay_on_);
      }
    }

    // Aggiorna ultimi stati
    last_x_ = x_now;
    last_circle_ = pressed(button_circle_index_);
    last_square_ = pressed(button_square_index_);
  }

  // Parametri
  std::string joy_topic_;
  std::string state_topic_;
  std::string event_topic_;
  std::string blades_cmd_topic_;
  std::string relay_cmd_topic_;
  int button_x_index_{0};
  int button_circle_index_{1};
  int button_square_index_{3};

  // Stato locale
  std::string current_state_{"IDLE"};
  bool in_manual_{false};
  bool blades_on_{false};
  bool relay_on_{false};

  // Edge detection
  bool last_x_{false};
  bool last_circle_{false};
  bool last_square_{false};

  // Interfacce ROS
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_state_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_event_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_blades_cmd_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_relay_cmd_;
};

/**
 * @brief Entry point del nodo.
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MowerJoyBridge>());
  rclcpp::shutdown();
  return 0;
}
