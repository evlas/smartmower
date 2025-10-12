#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>

class EventsBridge : public rclcpp::Node {
public:
  EventsBridge() : Node("events_bridge") {
    // Parameters
    events_topic_ = this->declare_parameter<std::string>("events_topic", "/events/pcf8574");
    tilt_topic_ = this->declare_parameter<std::string>("tilt_topic", "/safety/tilt");
    lift_topic_ = this->declare_parameter<std::string>("lift_topic", "/safety/lift");
    relay_state_topic_ = this->declare_parameter<std::string>("relay_state_topic", "/relay/state");
    bumper_left_topic_  = this->declare_parameter<std::string>("bumper_left_topic", "/sensors/bumper_left");
    bumper_right_topic_ = this->declare_parameter<std::string>("bumper_right_topic", "/sensors/bumper_right");
    rain_topic_         = this->declare_parameter<std::string>("rain_topic", "/sensors/rain");
    aux1_topic_         = this->declare_parameter<std::string>("aux1_topic", "/sensors/aux1");
    aux2_topic_         = this->declare_parameter<std::string>("aux2_topic", "/sensors/aux2");
    aux3_topic_         = this->declare_parameter<std::string>("aux3_topic", "/sensors/aux3");
    aux4_topic_         = this->declare_parameter<std::string>("aux4_topic", "/sensors/aux4");
    // Error/diagnostic topics (nuovi)
    err_imu_topic_      = this->declare_parameter<std::string>("err_imu_topic", "/errors/imu");
    err_batt_topic_     = this->declare_parameter<std::string>("err_batt_topic", "/errors/batt");
    err_sonar_topic_    = this->declare_parameter<std::string>("err_sonar_topic", "/errors/sonar");
    err_odom_topic_     = this->declare_parameter<std::string>("err_odom_topic", "/errors/odom");
    err_pcf_topic_      = this->declare_parameter<std::string>("err_pcf_topic", "/errors/pcf");
    perimeter_left_topic_  = this->declare_parameter<std::string>("perimeter_left_topic", "/sensors/perimeter_left");
    perimeter_right_topic_ = this->declare_parameter<std::string>("perimeter_right_topic", "/sensors/perimeter_right");

    // Bit mapping per README di pico_bridge
    bit_relay_enabled_ = this->declare_parameter<int>("bit_relay_enabled", 0);     // 1<<0
    bit_bumper_left_   = this->declare_parameter<int>("bit_bumper_left", 1);       // 1<<1
    bit_bumper_right_  = this->declare_parameter<int>("bit_bumper_right", 2);      // 1<<2
    bit_lift_          = this->declare_parameter<int>("bit_lift", 3);              // 1<<3
    bit_rain_          = this->declare_parameter<int>("bit_rain", 4);              // 1<<4
    // AUX mapping per firmware main.py: AUX1=5, AUX2=6, AUX3=7, AUX4 mappato a TILT (bit 11)
    bit_aux1_          = this->declare_parameter<int>("bit_aux1", 5);              // 1<<5
    bit_aux2_          = this->declare_parameter<int>("bit_aux2", 6);              // 1<<6
    bit_aux3_          = this->declare_parameter<int>("bit_aux3", 7);              // 1<<7
    bit_aux4_          = this->declare_parameter<int>("bit_aux4", 11);             // 1<<11 (tilt)
    bit_tilt_          = this->declare_parameter<int>("bit_tilt", 11);             // 1<<11
    bit_perimeter_left_  = this->declare_parameter<int>("bit_perimeter_left", 8);  // 1<<8
    bit_perimeter_right_ = this->declare_parameter<int>("bit_perimeter_right", 9); // 1<<9
    // Error bits per main.py firmware (confermata sorgente di verità)
    bit_err_pcf_       = this->declare_parameter<int>("bit_err_pcf", 10);          // 1<<10
    bit_err_imu_       = this->declare_parameter<int>("bit_err_imu", 12);          // 1<<12
    bit_err_batt_      = this->declare_parameter<int>("bit_err_batt", 13);         // 1<<13
    bit_err_sonar_     = this->declare_parameter<int>("bit_err_sonar", 14);        // 1<<14
    bit_err_odom_      = this->declare_parameter<int>("bit_err_odom", 15);         // 1<<15

    pub_tilt_   = this->create_publisher<std_msgs::msg::Bool>(tilt_topic_, 10);
    pub_lift_   = this->create_publisher<std_msgs::msg::Bool>(lift_topic_, 10);
    pub_relay_state_ = this->create_publisher<std_msgs::msg::Bool>(relay_state_topic_, 10);
    pub_bump_left_  = this->create_publisher<std_msgs::msg::Bool>(bumper_left_topic_, 10);
    pub_bump_right_ = this->create_publisher<std_msgs::msg::Bool>(bumper_right_topic_, 10);
    pub_rain_   = this->create_publisher<std_msgs::msg::Bool>(rain_topic_, 10);
    pub_aux1_   = this->create_publisher<std_msgs::msg::Bool>(aux1_topic_, 10);
    pub_aux2_   = this->create_publisher<std_msgs::msg::Bool>(aux2_topic_, 10);
    pub_aux3_   = this->create_publisher<std_msgs::msg::Bool>(aux3_topic_, 10);
    pub_aux4_   = this->create_publisher<std_msgs::msg::Bool>(aux4_topic_, 10);
    pub_perimeter_left_  = this->create_publisher<std_msgs::msg::Bool>(perimeter_left_topic_, 10);
    pub_perimeter_right_ = this->create_publisher<std_msgs::msg::Bool>(perimeter_right_topic_, 10);
    // Error publishers
    pub_err_imu_   = this->create_publisher<std_msgs::msg::Bool>(err_imu_topic_, 10);
    pub_err_batt_  = this->create_publisher<std_msgs::msg::Bool>(err_batt_topic_, 10);
    pub_err_sonar_ = this->create_publisher<std_msgs::msg::Bool>(err_sonar_topic_, 10);
    pub_err_odom_  = this->create_publisher<std_msgs::msg::Bool>(err_odom_topic_, 10);
    pub_err_pcf_   = this->create_publisher<std_msgs::msg::Bool>(err_pcf_topic_, 10);

    sub_events_ = this->create_subscription<std_msgs::msg::UInt16>(
      events_topic_, 10, std::bind(&EventsBridge::on_events, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "events_bridge avviato. Sub: %s -> Pub: relay=%s tilt=%s lift=%s bump_left=%s bump_right=%s rain=%s perimeter_l=%s perimeter_r=%s aux1=%s aux2=%s aux3=%s aux4=%s",
      events_topic_.c_str(), relay_state_topic_.c_str(), tilt_topic_.c_str(), lift_topic_.c_str(), bumper_left_topic_.c_str(), bumper_right_topic_.c_str(), rain_topic_.c_str(), perimeter_left_topic_.c_str(), perimeter_right_topic_.c_str(), aux1_topic_.c_str(), aux2_topic_.c_str(), aux3_topic_.c_str(), aux4_topic_.c_str());
  }

private:
  inline bool is_set(uint16_t v, int bit) const { return bit >= 0 && bit < 16 && ((v >> bit) & 0x1); }

  void on_events(const std_msgs::msg::UInt16::SharedPtr msg) {
    const uint16_t e = msg->data;
    std_msgs::msg::Bool b;
    // Relay state
    b.data = is_set(e, bit_relay_enabled_);
    pub_relay_state_->publish(b);
    // Safety
    b.data = is_set(e, bit_tilt_);
    pub_tilt_->publish(b);
    b.data = is_set(e, bit_lift_);
    pub_lift_->publish(b);
    // Bumpers
    b.data = is_set(e, bit_bumper_left_);
    pub_bump_left_->publish(b);
    b.data = is_set(e, bit_bumper_right_);
    pub_bump_right_->publish(b);
    // Environment
    b.data = is_set(e, bit_rain_);
    pub_rain_->publish(b);
    // Aux
    b.data = is_set(e, bit_aux1_);
    pub_aux1_->publish(b);
    b.data = is_set(e, bit_aux2_);
    pub_aux2_->publish(b);
    b.data = is_set(e, bit_aux3_);
    pub_aux3_->publish(b);
    b.data = is_set(e, bit_aux4_);
    pub_aux4_->publish(b);
    // Perimeter
    b.data = is_set(e, bit_perimeter_left_);
    pub_perimeter_left_->publish(b);
    b.data = is_set(e, bit_perimeter_right_);
    pub_perimeter_right_->publish(b);
    // Errors
    b.data = is_set(e, bit_err_imu_);
    pub_err_imu_->publish(b);
    b.data = is_set(e, bit_err_batt_);
    pub_err_batt_->publish(b);
    b.data = is_set(e, bit_err_sonar_);
    pub_err_sonar_->publish(b);
    b.data = is_set(e, bit_err_odom_);
    pub_err_odom_->publish(b);
    b.data = is_set(e, bit_err_pcf_);
    pub_err_pcf_->publish(b);
  }

  // Params
  std::string events_topic_;
  std::string tilt_topic_;
  std::string lift_topic_;
  std::string relay_state_topic_;
  std::string bumper_left_topic_;
  std::string bumper_right_topic_;
  std::string rain_topic_;
  std::string aux1_topic_;
  std::string aux2_topic_;
  std::string aux3_topic_;
  std::string aux4_topic_;
  std::string perimeter_left_topic_;
  std::string perimeter_right_topic_;
  std::string err_imu_topic_;
  std::string err_batt_topic_;
  std::string err_sonar_topic_;
  std::string err_odom_topic_;
  std::string err_pcf_topic_;
  int bit_tilt_;
  int bit_relay_enabled_;
  int bit_bumper_left_;
  int bit_bumper_right_;
  int bit_lift_;
  int bit_rain_;
  int bit_aux1_;
  int bit_aux2_;
  int bit_aux3_;
  int bit_aux4_;
  int bit_perimeter_left_;
  int bit_perimeter_right_;
  int bit_err_imu_;
  int bit_err_batt_;
  int bit_err_sonar_;
  int bit_err_odom_;
  int bit_err_pcf_;

  // ROS
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr sub_events_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_tilt_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_lift_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_relay_state_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_bump_left_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_bump_right_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_rain_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_aux1_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_aux2_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_aux3_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_aux4_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_perimeter_left_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_perimeter_right_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_err_imu_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_err_batt_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_err_sonar_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_err_odom_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_err_pcf_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EventsBridge>());
  rclcpp::shutdown();
  return 0;
}
