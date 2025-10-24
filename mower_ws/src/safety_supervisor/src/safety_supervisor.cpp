/**
 * @file safety_supervisor.cpp
 * @brief Nodo ROS2 che valuta condizioni di sicurezza e pubblica E-Stop.
 *
 * @details Ascolta tilt, lift, diagnostica e genera condizioni di arresto e messaggi
 * diagnostici derivati. Parametri configurabili per timeout e livelli di errore.
 */
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <map>
#include <string>

using namespace std::chrono_literals;

/**
 * @class SafetySupervisor
 * @brief Supervisiona segnali di sicurezza e pubblica lo stato di E-Stop.
 */
class SafetySupervisor : public rclcpp::Node
{
public:
    /** @brief Costruttore: inizializza pub/sub, parametri e timer. */
    SafetySupervisor() : Node("safety_supervisor"), last_error_update_(this->now())
    {
        // Inizializzazione publisher
        estop_pub_ = this->create_publisher<std_msgs::msg::Bool>("/safety/estop", 10);
        
        // Inizializzazione subscribers
        estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/buttons/estop", 10, 
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data) {
                    estop_ = true;
                    publish_estop();
                }
            });

        tilt_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/safety/tilt", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                tilt_detected_ = msg->data;
                check_safety_conditions();
            });

        lift_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/safety/lift", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                lift_detected_ = msg->data;
                check_safety_conditions();
            });

        // Sottoscrizione ai messaggi diagnostici
        diagnostics_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics", 10,
            [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
                bool has_errors = false;
                for (const auto& status : msg->status) {
                    if (status.level >= min_error_level_) {
                        active_errors_[status.name] = status;
                        has_errors = true;
                    } else {
                        active_errors_.erase(status.name);
                    }
                }
                
                if (has_errors) {
                    last_error_update_ = this->now();
                } else if (active_errors_.empty()) {
                    last_error_update_ = rclcpp::Time(0);
                }
                
                check_safety_conditions();
            });
            
        // Publisher per i messaggi diagnostici
        diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics_out", 10);

        // Parametri
        this->declare_parameter("error_timeout_sec", 120.0);
        this->declare_parameter("error_level", "ERROR");
        this->declare_parameter("hardware_id", "mower");
        
        error_timeout_sec_ = this->get_parameter("error_timeout_sec").as_double();
        hardware_id_ = this->get_parameter("hardware_id").as_string();
        
        std::string error_level = this->get_parameter("error_level").as_string();
        if (error_level == "OK") min_error_level_ = 0;
        else if (error_level == "WARN") min_error_level_ = 1;
        else if (error_level == "ERROR") min_error_level_ = 2;
        else if (error_level == "STALE") min_error_level_ = 3;
        
        RCLCPP_INFO(this->get_logger(), "Configurazione: timeout=%.1fs, livello_minimo=%d (%s)",
                   error_timeout_sec_, min_error_level_, error_level.c_str());
        
        // Inizializzazione servizi
        estop_reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "estop_reset",
            [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                  std_srvs::srv::Trigger::Response::SharedPtr response) {
                bool safe_to_reset = estop_ && !tilt_detected_ && !lift_detected_ && active_errors_.empty();
                
                if (safe_to_reset) {
                    estop_ = false;
                    response->success = true;
                    response->message = "E-Stop disattivato con successo";
                    RCLCPP_INFO(this->get_logger(), "E-Stop disattivato manualmente");
                    publish_diagnostics();
                } else {
                    response->success = false;
                    response->message = "Impossibile disattivare E-Stop: condizioni di sicurezza non soddisfatte";
                    RCLCPP_WARN(this->get_logger(), "Tentativo di disattivazione E-Stop non riuscito");
                }
                return;
            });

        error_reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "error_reset",
            [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                  std_srvs::srv::Trigger::Response::SharedPtr response) {
                bool had_errors = !active_errors_.empty();
                active_errors_.clear();
                last_error_update_ = rclcpp::Time(0);
                
                response->success = true;
                response->message = "Stato errore resettato";
                
                if (had_errors) {
                    RCLCPP_INFO(this->get_logger(), "Stato errore resettato manualmente");
                    check_safety_conditions();
                    publish_diagnostics();
                }
                return;
            });

        // Timer per controllare le condizioni di sicurezza
        timer_ = this->create_wall_timer(
            100ms, std::bind(&SafetySupervisor::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Nodo Safety Supervisor avviato");
    }

private:
    void timer_callback()
    {
        check_safety_conditions();
    }

    void check_safety_conditions()
    {
        bool has_critical_error = false;
        auto now = this->now();
        
        // Controlla se ci sono errori diagnostici attivi
        if (!active_errors_.empty()) {
            auto error_duration = (now - last_error_update_).seconds();
            if (error_duration > error_timeout_sec_) {
                RCLCPP_ERROR(this->get_logger(), "Timeout errore diagnostico (%.1fs), attivazione E-Stop", error_duration);
                has_critical_error = true;
            }
        }

        // Attiva E-Stop se una delle condizioni è vera
        if (tilt_detected_ || lift_detected_ || has_critical_error) {
            estop_ = true;
            if (tilt_detected_) {
                RCLCPP_ERROR(this->get_logger(), "Rilevato tilt, attivazione E-Stop");
            }
            if (lift_detected_) {
                RCLCPP_ERROR(this->get_logger(), "Rilevato sollevamento, attivazione E-Stop");
            }
        }

        // Pubblica lo stato E-Stop e diagnostica
        if (estop_ != last_estop_state_ || !active_errors_.empty()) {
            publish_estop();
            publish_diagnostics();
            last_estop_state_ = estop_;
        }
    }

    void publish_estop()
    {
        auto msg = std_msgs::msg::Bool();
        msg.data = estop_;
        estop_pub_->publish(msg);
        
        if (estop_) {
            RCLCPP_WARN(this->get_logger(), "E-Stop ATTIVATO");
        } else {
            // Rimosso log non essenziale
        }
    }
    
    void publish_diagnostics()
    {
        diagnostic_msgs::msg::DiagnosticArray msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "safety";
        
        // Aggiungi stato E-Stop
        diagnostic_msgs::msg::DiagnosticStatus estop_status;
        estop_status.level = estop_ ? diagnostic_msgs::msg::DiagnosticStatus::ERROR : 
                                    diagnostic_msgs::msg::DiagnosticStatus::OK;
        estop_status.name = "safety_estop";
        estop_status.message = estop_ ? "Emergency Stop attivato" : "Sistema OK";
        estop_status.hardware_id = hardware_id_;
        
        // Aggiungi informazioni aggiuntive
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = "tilt_detected";
        kv.value = tilt_detected_ ? "true" : "false";
        estop_status.values.push_back(kv);
        
        kv.key = "lift_detected";
        kv.value = lift_detected_ ? "true" : "false";
        estop_status.values.push_back(kv);
        
        kv.key = "active_errors";
        kv.value = std::to_string(active_errors_.size());
        estop_status.values.push_back(kv);

        // Accoda lo stato e pubblica
        msg.status.push_back(estop_status);
        diagnostics_pub_->publish(msg);
    }

    // Variabili di stato
    bool estop_ = false;
    bool last_estop_state_ = false;
    bool tilt_detected_ = false;
    bool lift_detected_ = false;
    std::map<std::string, diagnostic_msgs::msg::DiagnosticStatus> active_errors_;
    rclcpp::Time last_error_update_;
    
    // Parametri
    double error_timeout_sec_ = 120.0;
    int min_error_level_ = 2;  // Livello minimo per attivare E-Stop (2=ERROR)
    std::string hardware_id_ = "mower";

    // ROS2 Subscribers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tilt_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lift_sub_;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
    
    // ROS2 Publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
    
    // ROS2 Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estop_reset_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr error_reset_srv_;
    
    // ROS2 Timer
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SafetySupervisor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
