// Header
#include "mqtt/mqtt_client.h"

// Std & deps
#include <mosquitto.h>
#include <cstring>
#include <mutex>
#include <memory>
#include <string>
#include <vector>

namespace smartmower {
namespace mqtt {

namespace {
struct MosqLibInit {
    MosqLibInit() { mosquitto_lib_init(); }
    ~MosqLibInit() { mosquitto_lib_cleanup(); }
};
MosqLibInit& ensureMosqInit() {
    static MosqLibInit init;
    return init;
}
}

class MqttImpl {
public:
    explicit MqttImpl(const std::string& clientId)
        : clientId_(clientId) { (void)ensureMosqInit(); }
    ~MqttImpl() {
        if (mosq_) {
            mosquitto_disconnect(mosq_);
            mosquitto_destroy(mosq_);
            mosq_ = nullptr;
        }
    }

    bool connect(const std::string& broker, int port,
                 const std::string& username, const std::string& password,
                 MqttClient::MessageCallback* cb_ptr) {
        std::lock_guard<std::mutex> lk(mu_);
        if (mosq_) return true;
        mosq_ = mosquitto_new(clientId_.c_str(), true, this);
        if (!mosq_) return false;
        if (!username.empty()) {
            if (mosquitto_username_pw_set(mosq_, username.c_str(), password.c_str()) != MOSQ_ERR_SUCCESS)
                return false;
        }
        msg_cb_ptr_ = cb_ptr;
        mosquitto_message_callback_set(mosq_, &MqttImpl::on_message);
        int rc = mosquitto_connect(mosq_, broker.c_str(), port, 60);
        return rc == MOSQ_ERR_SUCCESS;
    }

    void disconnect() {
        std::lock_guard<std::mutex> lk(mu_);
        if (!mosq_) return;
        mosquitto_disconnect(mosq_);
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
    }

    bool publish(const std::string& topic, const void* data, size_t len, int qos, bool retain) {
        std::lock_guard<std::mutex> lk(mu_);
        if (!mosq_) return false;
        return mosquitto_publish(mosq_, nullptr, topic.c_str(), (int)len, data, qos, retain) == MOSQ_ERR_SUCCESS;
    }

    bool subscribe(const std::string& topic, int qos) {
        std::lock_guard<std::mutex> lk(mu_);
        if (!mosq_) return false;
        return mosquitto_subscribe(mosq_, nullptr, topic.c_str(), qos) == MOSQ_ERR_SUCCESS;
    }

    void loop(int timeout_ms) {
        // Non detenere il lock mentre chiamiamo mosquitto_loop, perché i callback
        // possono richiamare publish/subscribe che acquisiscono lo stesso mutex → deadlock.
        struct mosquitto* local = nullptr;
        {
            std::lock_guard<std::mutex> lk(mu_);
            local = mosq_;
        }
        if (!local) return;
        mosquitto_loop(local, timeout_ms, 100);
    }

private:
    static void on_message(struct mosquitto* /*mosq*/, void* userdata, const struct mosquitto_message* msg) {
        if (!userdata || !msg) return;
        auto* self = reinterpret_cast<MqttImpl*>(userdata);
        if (!self->msg_cb_ptr_ || !(*self->msg_cb_ptr_)) return;
        std::string topic = msg->topic ? msg->topic : "";
        std::vector<uint8_t> payload;
        if (msg->payload && msg->payloadlen > 0) {
            payload.resize((size_t)msg->payloadlen);
            std::memcpy(payload.data(), msg->payload, (size_t)msg->payloadlen);
        }
        (*self->msg_cb_ptr_)(topic, payload);
    }

    std::mutex mu_;
    std::string clientId_;
    struct mosquitto* mosq_ = nullptr;
    MqttClient::MessageCallback* msg_cb_ptr_ = nullptr;
};

// ---- Implementazione MqttClient (pImpl via void* impl_) ----

MqttClient::MqttClient(const std::string& broker, int port,
                       const std::string& clientId,
                       const std::string& username,
                       const std::string& password)
    : broker_(broker), port_(port), clientId_(clientId), username_(username), password_(password) {
    impl_ = new MqttImpl(clientId_);
}

MqttClient::~MqttClient() {
    if (impl_) {
        delete reinterpret_cast<MqttImpl*>(impl_);
        impl_ = nullptr;
    }
}

bool MqttClient::connect() {
    auto* impl = reinterpret_cast<MqttImpl*>(impl_);
    if (!impl) return false;
    return impl->connect(broker_, port_, username_, password_, &msg_cb_);
}

void MqttClient::disconnect() {
    auto* impl = reinterpret_cast<MqttImpl*>(impl_);
    if (!impl) return;
    impl->disconnect();
}

bool MqttClient::publish(const std::string& topic, const std::string& payload, int qos, bool retain) {
    auto* impl = reinterpret_cast<MqttImpl*>(impl_);
    if (!impl) return false;
    return impl->publish(topic, payload.data(), payload.size(), qos, retain);
}

bool MqttClient::publish(const std::string& topic, const std::vector<uint8_t>& payload, int qos, bool retain) {
    auto* impl = reinterpret_cast<MqttImpl*>(impl_);
    if (!impl) return false;
    const void* data = payload.empty() ? nullptr : payload.data();
    return impl->publish(topic, data, payload.size(), qos, retain);
}

bool MqttClient::subscribe(const std::string& topic, int qos) {
    auto* impl = reinterpret_cast<MqttImpl*>(impl_);
    if (!impl) return false;
    return impl->subscribe(topic, qos);
}

void MqttClient::loop(int timeout_ms) {
    auto* impl = reinterpret_cast<MqttImpl*>(impl_);
    if (!impl) return;
    impl->loop(timeout_ms);
}

} // namespace mqtt
} // namespace smartmower
