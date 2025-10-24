/**
 * @file rpi_gpio_node.cpp
 * @brief Nodo ROS2 per pubblicare lo stato dei GPIO Raspberry Pi come topic booleani.
 */
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <gpiod.h>
#include <chrono>
#include <unordered_map>
#include <string>
#include <memory>

using namespace std::chrono_literals;

struct LineCfg {
  int pin{-1};
  bool active_low{true};
  bool last_state{false};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub;
  std::string topic;
  std::string name;
};

/**
 * @class RpiGpioNode
 * @brief Legge linee GPIO tramite libgpiod e pubblica messaggi Bool con debounce.
 */
class RpiGpioNode : public rclcpp::Node {
public:
  /** @brief Costruttore: parametri, apertura chip, configurazione linee e timer. */
  RpiGpioNode() : Node("rpi_gpio") {
    // Parameters
    chip_name_ = declare_parameter<std::string>("chip_name", "gpiochip0");
    poll_rate_hz_ = declare_parameter<int>("poll_rate_hz", 200);
    debounce_ms_ = declare_parameter<int>("debounce_ms", 50);

    // E-Stop
    estop_pin_ = declare_parameter<int>("estop_pin", 4);
    estop_active_low_ = declare_parameter<bool>("estop_active_low", true);
    estop_topic_ = declare_parameter<std::string>("estop_topic", "/buttons/estop");
    estop_pull_ = declare_parameter<std::string>("estop_pull", "up");

    // Buttons
    btn_active_low_ = declare_parameter<bool>("button_active_low", true);
    btn_ns_ = declare_parameter<std::string>("buttons_namespace", "/buttons");
    ok_pin_ = declare_parameter<int>("ok_pin", 7);
    back_pin_ = declare_parameter<int>("back_pin", 8);
    up_pin_ = declare_parameter<int>("up_pin", 24);
    down_pin_ = declare_parameter<int>("down_pin", 25);

    // Open chip
    chip_ = gpiod_chip_open_by_name(chip_name_.c_str());
    if (!chip_) {
      RCLCPP_FATAL(get_logger(), "Failed to open GPIO chip: %s", chip_name_.c_str());
      throw std::runtime_error("gpiod_chip_open_by_name failed");
    }

    // Configure lines
    add_input_line("estop", estop_pin_, estop_active_low_, estop_topic_);
    add_input_line("ok", ok_pin_, btn_active_low_, ns_topic(btn_ns_, "ok"));
    add_input_line("back", back_pin_, btn_active_low_, ns_topic(btn_ns_, "back"));
    add_input_line("up", up_pin_, btn_active_low_, ns_topic(btn_ns_, "up"));
    add_input_line("down", down_pin_, btn_active_low_, ns_topic(btn_ns_, "down"));

    // Timer for polling
    const double period_ms = 1000.0 / std::max(1, poll_rate_hz_);
    timer_ = create_wall_timer(std::chrono::milliseconds((int)period_ms),
      std::bind(&RpiGpioNode::poll_lines, this));

    RCLCPP_INFO(get_logger(), "rpi_gpio started on %s. E-Stop GPIO=%d (active_%s, pull_%s), debounce=%dms",
                chip_name_.c_str(), estop_pin_, 
                estop_active_low_?"low":"high", 
                estop_pull_.empty()?"none":estop_pull_.c_str(), 
                debounce_ms_);
  }

  /** @brief Distruttore: rilascia linee e chiude il chip. */
  ~RpiGpioNode() override {
    for (auto & kv : lines_) {
      if (kv.second.line) gpiod_line_release(kv.second.line);
    }
    if (chip_) gpiod_chip_close(chip_);
  }

private:
  struct LineHandle { std::string name; int pin; bool active_low; bool last; int64_t last_change_ms; gpiod_line* line; rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub; };

  static std::string ns_topic(const std::string &ns, const std::string &leaf) {
    if (ns.empty() || ns == "/") return "/" + leaf;
    if (ns.back() == '/') return ns + leaf;
    return ns + "/" + leaf;
  }

  /** @brief Configura una linea di input e crea relativo publisher.
   *  @param name Nome logico della linea.
   *  @param pin GPIO number.
   *  @param active_low True se la linea è attiva-bassa.
   *  @param topic Topic su cui pubblicare.
   */
  void add_input_line(const std::string &name, int pin, bool active_low, const std::string &topic) {
    if (pin < 0) return; // disabled
    gpiod_line* line = gpiod_chip_get_line(chip_, pin);
    if (!line) {
      RCLCPP_ERROR(get_logger(), "Failed to get GPIO line %d for %s", pin, name.c_str());
      return;
    }
    
    // Configura il pull-up/pull-down in base al parametro specifico per l'E-Stop o al default
    int flags = 0;
    if (name == "estop") {
      if (estop_pull_ == "up") {
        flags = GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP;
      } else if (estop_pull_ == "down") {
        flags = GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN;
      } // else: no pull
    } else {
      // Per i pulsanti usa il comportamento basato su active_low
      flags = active_low ? GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP : GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN;
    }
    
    int ret = gpiod_line_request_input_flags(line, ("rpi_gpio_"+name).c_str(), flags);
    if (ret < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to request input for GPIO %d (%s)", pin, name.c_str());
      return;
    }
    auto pub = create_publisher<std_msgs::msg::Bool>(topic, 10);

    LineHandle lh{ name, pin, active_low, false, now_ms(), line, pub };
    // Initialize state
    bool state = read_line(lh);
    lh.last = state;
    publish(lh, state, true);

    lines_.emplace(name, std::move(lh));
    RCLCPP_INFO(get_logger(), "Line configured: %s on GPIO %d -> %s (active_%s)",
                name.c_str(), pin, topic.c_str(), active_low?"low":"high");
  }

  /** @brief Legge lo stato logico della linea considerando la polarità.
   *  @return true se linea attiva, false altrimenti.
   */
  bool read_line(const LineHandle &lh) {
    int val = gpiod_line_get_value(lh.line);
    if (val < 0) return lh.last;
    bool logical = lh.active_low ? (val == 0) : (val != 0);
    return logical;
  }

  /** @brief Pubblica su ROS lo stato della linea e logga variazioni.
   *  @param initial Se true, log informativo iniziale.
   */
  void publish(LineHandle &lh, bool state, bool initial=false) {
    std_msgs::msg::Bool m; m.data = state;
    lh.pub->publish(m);
    if (!initial) {
      RCLCPP_WARN(get_logger(), "GPIO %s changed -> %s", lh.name.c_str(), state?"TRUE":"FALSE");
    } else {
      RCLCPP_INFO(get_logger(), "GPIO %s initial -> %s", lh.name.c_str(), state?"TRUE":"FALSE");
    }
  }

  static int64_t now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  /** @brief Loop di polling con debounce in millisecondi.
   */
  void poll_lines() {
    for (auto & kv : lines_) {
      auto & lh = kv.second;
      bool cur = read_line(lh);
      if (cur != lh.last) {
        int64_t t = now_ms();
        if (t - lh.last_change_ms >= debounce_ms_) {
          lh.last = cur;
          lh.last_change_ms = t;
          publish(lh, cur);
        }
      }
    }
  }

  // Params
  std::string chip_name_;
  int poll_rate_hz_;
  int debounce_ms_;
  int estop_pin_;
  bool estop_active_low_;
  std::string estop_topic_;
  std::string estop_pull_;
  bool btn_active_low_;
  std::string btn_ns_;
  int ok_pin_;
  int back_pin_;
  int up_pin_;
  int down_pin_;

  // GPIO
  gpiod_chip* chip_{nullptr};
  std::unordered_map<std::string, LineHandle> lines_;

  // ROS
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RpiGpioNode>());
  rclcpp::shutdown();
  return 0;
}
