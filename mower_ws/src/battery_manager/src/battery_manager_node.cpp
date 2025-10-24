/**
 * @file battery_manager_node.cpp
 * @brief Nodo ROS2 per gestione batteria: stima percentuale, eventi e diagnostica.
 *
 * @details Supporta input alternativi (battery_raw, battery_state, voltage),
 * pubblica `/diagnostics` e genera eventi come `LOW_BATTERY` su `/mower/event`.
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <cmath>
#include <chrono>
#include <string>

using std::placeholders::_1;

/**
 * @class BatteryManagerNode
 * @brief Gestisce la valutazione stato batteria e la segnalazione diagnostica/eventi.
 */
class BatteryManagerNode : public rclcpp::Node {
public:
  /**
   * @brief Costruttore: dichiara parametri, crea pub/sub e log iniziale.
   */
  BatteryManagerNode()
  : Node("battery_manager")
  {
    // Parameters
    event_topic_ = this->declare_parameter<std::string>("event_topic", "/mower/event");
    // Pubblica diagnostica su /diagnostics per integrazione con safety_supervisor
    error_topic_ = this->declare_parameter<std::string>("error_topic", "/diagnostics");
    battery_topic_ = this->declare_parameter<std::string>("battery_topic", "/sensors/battery");
    // Modalità input: 'battery_raw' (default), 'battery_state', 'voltage'
    input_type_ = this->declare_parameter<std::string>("battery_input_type", "battery_raw");
    battery_raw_topic_ = this->declare_parameter<std::string>("battery_raw_topic", "/sensors/battery_raw");
    voltage_topic_ = this->declare_parameter<std::string>("voltage_topic", "/sensors/bus_voltage");
    voltage_max_v_ = this->declare_parameter<double>("voltage_max_v", 12.6); // 3S LiPo full
    voltage_min_v_ = this->declare_parameter<double>("voltage_min_v", 9.0);  // 3S LiPo critical

    low_batt_thresh_pct_ = this->declare_parameter<double>("low_battery_threshold_pct", 30.0);
    low_batt_clear_pct_  = this->declare_parameter<double>("low_battery_clear_pct", 35.0);
    critical_batt_thresh_pct_ = this->declare_parameter<double>("critical_battery_threshold_pct", 5.0);
    critical_batt_clear_pct_  = this->declare_parameter<double>("critical_battery_clear_pct", 8.0);

    // Publishers
    pub_event_ = this->create_publisher<std_msgs::msg::String>(event_topic_, 10);
    pub_error_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(error_topic_, 10);
    pub_battery_state_ = this->create_publisher<sensor_msgs::msg::BatteryState>(battery_topic_, 10);

    // Subscriber in base alla modalità
    if (input_type_ == "battery_raw") {
      sub_battery_raw_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        battery_raw_topic_, 10, std::bind(&BatteryManagerNode::onBatteryRaw, this, _1));
    } else if (input_type_ == "battery_state") {
      sub_battery_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
        battery_topic_, 10, std::bind(&BatteryManagerNode::onBattery, this, _1));
    } else {
      sub_battery_voltage_ = this->create_subscription<std_msgs::msg::Float32>(
        voltage_topic_, 10, std::bind(&BatteryManagerNode::onBatteryVoltage, this, _1));
    }

    RCLCPP_INFO(this->get_logger(), "battery_manager avviato. Mode: %s. Sub a: %s, pub event: %s, error: %s",
      input_type_.c_str(),
      (input_type_=="battery_raw" ? battery_raw_topic_.c_str() : (input_type_=="battery_state" ? battery_topic_.c_str() : voltage_topic_.c_str())),
      event_topic_.c_str(), error_topic_.c_str());
  }

private:
  // Parameters
  std::string event_topic_;
  std::string error_topic_;
  std::string battery_topic_;
  std::string input_type_;
  std::string battery_raw_topic_;
  std::string voltage_topic_;
  double low_batt_thresh_pct_;
  double low_batt_clear_pct_;
  double critical_batt_thresh_pct_;
  double critical_batt_clear_pct_;
  double voltage_max_v_;
  double voltage_min_v_;

  // Publishers / Subscribers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_event_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_error_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_battery_state_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr sub_battery_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_battery_voltage_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_battery_raw_;

  // State flags to avoid spamming
  bool low_battery_sent_{false};
  bool critical_error_sent_{false};

  static bool isFinite(double v) {
    return std::isfinite(v);
  }

  // Callback per BatteryState: estrae percentuale e stato e delega alla logica comune
  /**
   * @brief Callback per `sensor_msgs::BatteryState`.
   * @param msg Messaggio di stato batteria.
   */
  void onBattery(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
    double pct = NAN;
    uint8_t power_status = msg->power_supply_status;

    // Se percentage non è disponibile ma abbiamo tensione, calcolala
    if (!isFinite(msg->percentage) && isFinite(msg->voltage)) {
      double range = voltage_max_v_ - voltage_min_v_;
      if (range > 0.0) {
        pct = (msg->voltage - voltage_min_v_) * 100.0 / range;
        if (pct < 0.0) pct = 0.0;
        if (pct > 100.0) pct = 100.0;
        RCLCPP_DEBUG(get_logger(), "Battery percentage calcolata da tensione: %.1f%% (V=%.2f)", pct, msg->voltage);
      }
    } else if (isFinite(msg->percentage)) {
      pct = msg->percentage * 100.0; // [0..1] -> [0..100]
    }

    // Determina stato alimentazione dalla corrente se disponibile
    if (isFinite(msg->current)) {
      if (msg->current < -0.1) {
        power_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
      } else if (msg->current > 0.1) {
        power_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
      } else {
        power_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
      }
    }

    processBatteryLogic(pct, power_status);
  }

  // Callback per batteria grezza: [voltage, current]
  /**
   * @brief Callback per batteria grezza `[voltage, current]`.
   * @param msg Array con V e A.
   */
  void onBatteryRaw(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    double voltage = NAN;
    double current = NAN;
    if (msg->data.size() >= 1) voltage = static_cast<double>(msg->data[0]);
    if (msg->data.size() >= 2) current = static_cast<double>(msg->data[1]);

    sensor_msgs::msg::BatteryState out;
    out.header.stamp = this->now();
    if (isFinite(voltage)) out.voltage = voltage; // [V]
    if (isFinite(current)) out.current = current; // [A], segno: >0 scarica, <0 carica (convenzionalmente)
    // percentage non stimata né letta dal raw: lasciata NaN

    // Stato alimentazione
    if (isFinite(current)) {
      if (current < -0.1) out.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
      else if (current > 0.1) out.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
      else out.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
    } else {
      out.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    }

    pub_battery_state_->publish(out);
    // percentuale non disponibile -> logica basata esclusivamente sullo status
    processBatteryLogic(NAN, out.power_supply_status);
  }

  // Callback per sola tensione: calcola percentuale lineare tra voltage_min_v_ e voltage_max_v_
  /**
   * @brief Callback per sola tensione del bus.
   * @param msg Tensione [V].
   */
  void onBatteryVoltage(const std_msgs::msg::Float32::SharedPtr msg) {
    double v = static_cast<double>(msg->data);
    double pct = NAN;
    if (isFinite(v)) {
      double range = voltage_max_v_ - voltage_min_v_;
      if (range > 0.0) {
        pct = (v - voltage_min_v_) * 100.0 / range;
        if (pct < 0.0) pct = 0.0;
        if (pct > 100.0) pct = 100.0;
      }
    }
    // Senza informazione di corrente assumiamo scarica (comportamento conservativo)
    processBatteryLogic(pct, sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING);
  }

  // Logica comune per generare eventi/diagnostica
  /**
   * @brief Logica comune per soglie, isteresi e generazione diagnostica/eventi.
   * @param pct Percentuale batteria [0..100] o NaN se non disponibile.
   * @param power_supply_status Stato alimentazione (charging/discharging/...).
   */
  void processBatteryLogic(double pct, uint8_t power_supply_status) {
    const bool discharging = (power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING);
    const bool charging    = (power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING);
    const bool full        = (power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL);

    // Reset dei latch quando in carica o piena
    if (charging || full) {
      if (low_battery_sent_) {
        RCLCPP_INFO(get_logger(), "Reset low_battery latch (charging/full)");
        low_battery_sent_ = false;
      }
      if (critical_error_sent_) {
        RCLCPP_INFO(get_logger(), "Reset critical battery error latch (charging/full)");
        critical_error_sent_ = false;
      }
      return;
    }

    if (!discharging) {
      // Stato sconosciuto: applica solo isteresi di reset
      if (isFinite(pct) && pct > low_batt_clear_pct_) low_battery_sent_ = false;
      if (isFinite(pct) && pct > critical_batt_clear_pct_) critical_error_sent_ = false;
      return;
    }

    if (!isFinite(pct)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000, "Battery percentage non valida (NaN). Impossibile valutare LOW_BATTERY/CRITICAL");
      return;
    }

    // 1) Errore critico
    if (pct < critical_batt_thresh_pct_) {
      if (!critical_error_sent_) {
        diagnostic_msgs::msg::DiagnosticArray arr;
        arr.header.stamp = this->now();

        diagnostic_msgs::msg::DiagnosticStatus st;
        st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        st.name = "battery";
        char msgbuf[256];
        std::snprintf(msgbuf, sizeof(msgbuf), "Batteria CRITICA: %.1f%%. Arresto necessario e docking immediato.", pct);
        st.message = msgbuf;
        st.hardware_id = "battery";

        diagnostic_msgs::msg::KeyValue kv;
        kv.key = "percentage";
        char pbuf[32];
        std::snprintf(pbuf, sizeof(pbuf), "%.1f", pct);
        kv.value = pbuf;
        st.values.push_back(kv);

        arr.status.push_back(st);
        pub_error_->publish(arr);
        RCLCPP_ERROR(get_logger(), "%s", st.message.c_str());
        critical_error_sent_ = true;
      }
      return;
    } else if (pct > critical_batt_clear_pct_) {
      if (critical_error_sent_) {
        RCLCPP_INFO(get_logger(), "Batteria uscita dalla condizione CRITICA (%.1f%% > %.1f%%)", pct, critical_batt_clear_pct_);
      }
      critical_error_sent_ = false;
    }

    // 2) Evento LOW_BATTERY
    if (pct < low_batt_thresh_pct_) {
      if (!low_battery_sent_) {
        std_msgs::msg::String ev;
        ev.data = "LOW_BATTERY";
        pub_event_->publish(ev);
        RCLCPP_WARN(get_logger(), "LOW_BATTERY emesso (%.1f%% < %.1f%%)", pct, low_batt_thresh_pct_);
        low_battery_sent_ = true;
      }
    } else if (pct > low_batt_clear_pct_) {
      if (low_battery_sent_) {
        RCLCPP_INFO(get_logger(), "Batteria sopra soglia clear (%.1f%% > %.1f%%): reset LOW_BATTERY", pct, low_batt_clear_pct_);
      }
      low_battery_sent_ = false;
    }
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BatteryManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
