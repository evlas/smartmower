#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>

using std::placeholders::_1;

class StateMachineNode : public rclcpp::Node {
public:
  enum class State {
    IDLE,
    UNDOCKING,
    MOWING,
    DOCKING,
    CHARGING,
    EMERGENCY_STOP,
    MANUAL_CONTROL,
    ERROR,
    PAUSED
  };

  StateMachineNode() : Node("mower_state_machine") {
    // Parameters
    cmd_timeout_s_ = declare_parameter<double>("cmd_timeout", 1.0);
    auto_resume_ = declare_parameter<bool>("auto_resume", false);
    start_on_boot_ = declare_parameter<bool>("start_on_boot", false);

    // Topic names
    const std::string event_topic = "/mower/event";
    const std::string state_topic = "/mower/state";
    const std::string estop_topic = "/safety/estop";
    const std::string error_topic = "/safety/error";

    // State publisher
    state_pub_ = create_publisher<std_msgs::msg::String>(state_topic, 10);

    // Subscribers
    event_sub_ = create_subscription<std_msgs::msg::String>(
      event_topic, 10, std::bind(&StateMachineNode::on_event, this, _1));

    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      estop_topic, 10, std::bind(&StateMachineNode::on_estop, this, _1));

    error_sub_ = create_subscription<std_msgs::msg::String>(
      error_topic, 10, std::bind(&StateMachineNode::on_error, this, _1));

    // Timer for state machine updates
    timer_ = create_wall_timer(
      std::chrono::milliseconds(200), 
      std::bind(&StateMachineNode::on_timer, this)
    );

    // Initial state
    set_state(State::IDLE);
    RCLCPP_INFO(get_logger(), "State machine started. Initial state: %s", state_to_cstr(state_));
  }

  // Gestione degli errori
  void on_error(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_ERROR(get_logger(), "Safety error received: %s", msg->data.c_str());
    set_state(State::ERROR);
  }

private:
  // Pubblicatori
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

  // Sottoscrittori
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr event_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr error_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Parametri
  double cmd_timeout_s_{1.0};
  bool auto_resume_{false};
  bool start_on_boot_{false};

  // Stato
  State state_{State::IDLE};
  State previous_state_{State::IDLE};
  
  // Helper functions
  bool is_valid_paused_state(State s) const {
    return (s == State::UNDOCKING || s == State::MOWING || s == State::DOCKING);
  }

  // Converti stato in stringa
  static const char* state_to_cstr(State s) {
    switch (s) {
      case State::IDLE: return "IDLE";
      case State::UNDOCKING: return "UNDOCKING";
      case State::MOWING: return "MOWING";
      case State::DOCKING: return "DOCKING";
      case State::CHARGING: return "CHARGING";
      case State::EMERGENCY_STOP: return "EMERGENCY_STOP";
      case State::MANUAL_CONTROL: return "MANUAL_CONTROL";
      case State::ERROR: return "ERROR";
      case State::PAUSED: return "PAUSED";
      default: return "UNKNOWN";
    }
  }

  // Pubblica lo stato attuale
  void publish_state() {
    auto msg = std_msgs::msg::String();
    msg.data = state_to_cstr(state_);
    state_pub_->publish(msg);
  }

  // Imposta un nuovo stato
  void set_state(State s) {
    if (s == state_) return;
    previous_state_ = state_;
    state_ = s;
    RCLCPP_INFO(get_logger(), "State changed to: %s", state_to_cstr(state_));
    publish_state();
  }

  // Gestione eventi
  void on_event(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "Received event: %s", msg->data.c_str());
    handle_event(msg->data);
  }

  void on_estop(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
      RCLCPP_WARN(get_logger(), "Emergency stop activated!");
      set_state(State::EMERGENCY_STOP);
    }
  }

  void on_timer() {
    // Qui puoi implementare timeout o logica di auto-ripresa
    (void)auto_resume_;
    (void)cmd_timeout_s_;
  }

  void handle_event(const std::string &e) {
    RCLCPP_DEBUG(get_logger(), "Processing event: %s", e.c_str());

    // Transizioni globali (valide da qualsiasi stato)
    if (e == "ESTOP") {
      set_state(State::EMERGENCY_STOP);
      return;
    }

    // Transizioni specifiche per stato
    switch (state_) {
      case State::IDLE:
        if (e == "START_MOWING") set_state(State::MOWING);
        else if (e == "UNDOCK") set_state(State::UNDOCKING);
        else if (e == "MANUAL_ON") set_state(State::MANUAL_CONTROL);
        else if (e == "CHARGE") set_state(State::CHARGING);
        else if (e == "ERROR") set_state(State::ERROR);
        break;

      case State::UNDOCKING:
        if (e == "UNDOCK_DONE") set_state(State::MOWING);
        else if (e == "PAUSE") {
          previous_state_ = state_;
          set_state(State::PAUSED);
        } else if (e == "ERROR") set_state(State::ERROR);
        break;

      case State::MOWING:
        if (e == "PAUSE") {
          previous_state_ = state_;
          set_state(State::PAUSED);
        } else if (e == "DOCK" || e == "LOW_BATTERY") {
          set_state(State::DOCKING);
        } else if (e == "ERROR") set_state(State::ERROR);
        break;

      case State::DOCKING:
        if (e == "DOCKED") set_state(State::CHARGING);
        else if (e == "STOP") set_state(State::IDLE);
        else if (e == "PAUSE") {
          previous_state_ = state_;
          set_state(State::PAUSED);
        } else if (e == "ERROR") set_state(State::ERROR);
        break;

      case State::CHARGING:
        if (e == "CHARGED") set_state(State::IDLE);
        else if (e == "START_MOWING") set_state(State::UNDOCKING);
        else if (e == "ERROR") set_state(State::ERROR);
        break;

      case State::EMERGENCY_STOP:
        if (e == "RESET" || e == "RESET_ESTOP" || e == "CLEAR_ERROR") {
          set_state(State::IDLE);
        }
        break;

      case State::MANUAL_CONTROL:
        if (e == "MANUAL_OFF" || e == "STOP") {
          set_state(State::IDLE);
        } else if (e == "ERROR") {
          set_state(State::ERROR);
        }
        break;

      case State::ERROR:
        if (e == "RESET" || e == "CLEAR_ERROR") {
          set_state(State::IDLE);
        }
        break;

      case State::PAUSED:
        if (e == "RESUME") {
          if (is_valid_paused_state(previous_state_)) {
            set_state(previous_state_);
          } else {
            RCLCPP_WARN(get_logger(), "Cannot resume from PAUSED: invalid previous state");
          }
        } else if (e == "STOP") {
          set_state(State::IDLE);
        } else if (e == "ERROR") {
          set_state(State::ERROR);
        }
        break;

      default:
        RCLCPP_WARN(get_logger(), "Unhandled state: %s", state_to_cstr(state_));
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StateMachineNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
