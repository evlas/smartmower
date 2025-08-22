// SmartMower Web Admin - clean server
#include <microhttpd.h>
#include <string>
#include <cstring>
#include <fstream>
#include <sstream>
#include <vector>
#include <optional>
#include <thread>
#include <chrono>
#include <cstdlib>

#include <nlohmann/json.hpp>

#include "config/config.h"
#include "mqtt/mqtt_client.h"

using json = nlohmann::json;

struct HttpResponse {
    int status{200};
    std::string contentType{"application/json"};
    std::string body{"{}"};
    std::vector<std::pair<std::string,std::string>> headers;
};

// -------- util --------
static std::string readFile(const std::string& path){
    std::ifstream ifs(path, std::ios::binary);
    if(!ifs.is_open()) return {};
    std::ostringstream ss; ss << ifs.rdbuf();
    return ss.str();
}

// ---- Manual control APIs (area utente) ----
static void loadMqttConfig(std::string& broker, int& port, std::string& username, std::string& password) {
    smartmower::config::ConfigLoader cfg;
    const auto& j = cfg.json();
    broker = j.value("mqtt", json{}).value("broker", std::string("localhost"));
    port = j.value("mqtt", json{}).value("port", 1883);
    username = j.value("mqtt", json{}).value("username", std::string());
    password = j.value("mqtt", json{}).value("password", std::string());
}

[[maybe_unused]] static std::string rootTopic() {
    smartmower::config::ConfigLoader cfg; return cfg.rootTopic();
}

static std::string fsmCommandsTopicFromConfig(){
    smartmower::config::ConfigLoader cfg;
    const auto& j = cfg.json();
    const std::string root = cfg.rootTopic();
    // override diretto
    std::string override_cmd = j.value("state_machine", json{}).value("command_topic", std::string());
    if(!override_cmd.empty()) return override_cmd;
    // costruzione da mqtt.topics.state_machine
    std::string base = j.value("mqtt", json{}).value("topics", json{}).value("state_machine", json{}).value("base", std::string("state_machine"));
    std::string sub  = j.value("mqtt", json{}).value("topics", json{}).value("state_machine", json{}).value("subtopics", json{}).value("commands", std::string("commands"));
    return root + "/" + base + "/" + sub; // es. smartmower/state_machine/commands
}

static HttpResponse apiManualEnter(){
    try{
        std::string broker, username, password; int port=1883;
        loadMqttConfig(broker, port, username, password);
        const std::string t_cmd = fsmCommandsTopicFromConfig();
        smartmower::mqtt::MqttClient cli(broker, port, "smartmower_webadmin", username, password);
        if(!cli.connect()){ json err; err["error"] = "mqtt connect failed"; return {500, "application/json", err.dump(), {}}; }
        // Evento per entrare in manuale: stringa semplice 'manual' o JSON equivalente
        cli.publish(t_cmd, std::string("manual"), 1, false);
        return {200, "application/json", std::string("{\"status\":\"ok\"}"), {}};
    }catch(const std::exception& e){ json err; err["error"] = std::string(e.what()); return {400, "application/json", err.dump(), {}}; }
}

static HttpResponse apiManualExit(){
    try{
        std::string broker, username, password; int port=1883;
        loadMqttConfig(broker, port, username, password);
        const std::string t_cmd = fsmCommandsTopicFromConfig();
        smartmower::mqtt::MqttClient cli(broker, port, "smartmower_webadmin", username, password);
        if(!cli.connect()){ json err; err["error"] = "mqtt connect failed"; return {500, "application/json", err.dump(), {}}; }
        json out; out["manual"]["exit"] = true;
        cli.publish(t_cmd, out.dump(), 1, false);
        return {200, "application/json", std::string("{\"status\":\"ok\"}"), {}};
    }catch(const std::exception& e){ json err; err["error"] = std::string(e.what()); return {400, "application/json", err.dump(), {}}; }
}

static HttpResponse apiManualCmdVel(const std::string& body){
    try{
        double lin = 0.0, ang = 0.0;
        bool parsed = false;
        // Prova JSON
        try{
            auto js = json::parse(body);
            if(js.is_object()){
                lin = js.value("linear", 0.0);
                ang = js.value("angular", 0.0);
            }
            // se js non è oggetto (es. null/stringa/numero), usiamo i default 0,0
            parsed = true;
        }catch(...){
            // Prova form url-encoded semplice: linear=..&angular=..
            if(!body.empty()){
                auto decode = [](std::string s){
                    std::replace(s.begin(), s.end(), '+', ' ');
                    return s; };
                auto getkv = [&](const std::string& key)->std::optional<std::string>{
                    size_t pos = 0; while(pos < body.size()){
                        size_t amp = body.find('&', pos); if(amp==std::string::npos) amp = body.size();
                        size_t eq = body.find('=', pos); if(eq!=std::string::npos && eq < amp){
                            auto k = body.substr(pos, eq-pos); auto v = body.substr(eq+1, amp-(eq+1));
                            if(k == key) return std::optional<std::string>(decode(v));
                        }
                        pos = amp + 1;
                    }
                    return std::nullopt; };
                if(auto sv = getkv("linear")) { try{ lin = std::stod(*sv); }catch(...){} }
                if(auto sv = getkv("angular")) { try{ ang = std::stod(*sv); }catch(...){} }
                parsed = true; // consideriamo valido anche se mancano i campi
            } else {
                // body vuoto: default 0,0
                parsed = true;
            }
        }
        (void)parsed; // evitare warning: variabile impostata ma non usata
        // invio come comando unificato FSM
        json out; out["manual"]["cmd_vel"]["linear"] = lin; out["manual"]["cmd_vel"]["angular"] = ang;
        std::string broker, username, password; int port=1883; loadMqttConfig(broker, port, username, password);
        const std::string topic = fsmCommandsTopicFromConfig();
        smartmower::mqtt::MqttClient cli(broker, port, "smartmower_webadmin", username, password);
        if(!cli.connect()){ json err; err["error"] = "mqtt connect failed"; return {500, "application/json", err.dump(), {}}; }
        cli.publish(topic, out.dump(), 0, false);
        return {200, "application/json", std::string("{\"status\":\"ok\"}"), {}};
    }catch(const std::exception& e){ json err; err["error"] = std::string("invalid cmd_vel payload: ") + e.what(); return {400, "application/json", err.dump(), {}}; }
}

static HttpResponse apiManualBlade(const std::string& body){
    try{
        bool on = false; bool parsed = false;
        try{
            auto js = json::parse(body);
            if(js.is_object()){
                on = js.value("on", false);
            }
            // se js non è oggetto (es. null), manteniamo default false
            parsed = true;
        }catch(...){
            // Prova form url-encoded: on=true|1
            if(!body.empty()){
                auto getkv = [&](const std::string& key)->std::optional<std::string>{
                    size_t pos = 0; while(pos < body.size()){
                        size_t amp = body.find('&', pos); if(amp==std::string::npos) amp = body.size();
                        size_t eq = body.find('=', pos); if(eq!=std::string::npos && eq < amp){
                            auto k = body.substr(pos, eq-pos); auto v = body.substr(eq+1, amp-(eq+1));
                            if(k == key) return std::optional<std::string>(v);
                        }
                        pos = amp + 1;
                    }
                    return std::nullopt; };
                if(auto sv = getkv("on")){
                    std::string v = *sv; std::transform(v.begin(), v.end(), v.begin(), ::tolower);
                    on = (v=="1" || v=="true" || v=="on");
                }
                parsed = true;
            } else {
                // body vuoto: default off
                parsed = true;
            }
        }
        (void)parsed; // evitare warning: variabile impostata ma non usata
        json out; out["manual"]["blade"]["on"] = on;
        std::string broker, username, password; int port=1883; loadMqttConfig(broker, port, username, password);
        const std::string topic = fsmCommandsTopicFromConfig();
        smartmower::mqtt::MqttClient cli(broker, port, "smartmower_webadmin", username, password);
        if(!cli.connect()){ json err; err["error"] = "mqtt connect failed"; return {500, "application/json", err.dump(), {}}; }
        cli.publish(topic, out.dump(), 0, false);
        return {200, "application/json", std::string("{\"status\":\"ok\"}"), {}};
    }catch(const std::exception& e){ json err; err["error"] = std::string("invalid blade payload: ") + e.what(); return {400, "application/json", err.dump(), {}}; }
}

static std::string guessContentType(const std::string& path){
    if(path.rfind(".js") != std::string::npos) return "application/javascript";
    if(path.rfind(".css") != std::string::npos) return "text/css";
    if(path.rfind(".json") != std::string::npos) return "application/json";
    if(path.rfind(".svg") != std::string::npos) return "image/svg+xml";
    if(path.rfind(".png") != std::string::npos) return "image/png";
    if(path.rfind(".jpg") != std::string::npos || path.rfind(".jpeg") != std::string::npos) return "image/jpeg";
    if(path.rfind(".ico") != std::string::npos) return "image/x-icon";
    return "text/html";
}

static std::string getCookie(struct MHD_Connection* connection, const std::string& name){
    const char* ck = MHD_lookup_connection_value(connection, MHD_HEADER_KIND, "Cookie");
    if(!ck) return {};
    std::string s(ck);
    size_t pos = 0;
    while(pos < s.size()){
        while(pos < s.size() && (s[pos]==' ' || s[pos]==';')) pos++;
        size_t eq = s.find('=', pos);
        if(eq == std::string::npos) break;
        std::string k = s.substr(pos, eq-pos);
        size_t end = s.find(';', eq+1);
        std::string v = s.substr(eq+1, (end==std::string::npos? s.size():end) - (eq+1));
        if(k == name) return v;
        if(end == std::string::npos) break;
        pos = end+1;
    }
    return {};
}

static bool checkAdminPin(const std::string& pin){
    try{
        smartmower::config::ConfigLoader cfg;
        auto j = cfg.json();
        std::string expected = j.value("system", json{}).value("admin_pin", std::string("123456"));
        return !pin.empty() && pin == expected;
    }catch(...){ return pin == "123456"; }
}

static bool isAuthorizedAdmin(struct MHD_Connection* connection){
    std::string pin = getCookie(connection, "admin_pin");
    return checkAdminPin(pin);
}

// -------- APIs --------
static HttpResponse apiGetState(){
    try{
        smartmower::config::ConfigLoader cfg;
        const std::string root = cfg.rootTopic();
        const auto& j = cfg.json();
        std::string broker = j.value("mqtt", json{}).value("broker", std::string("localhost"));
        int port = j.value("mqtt", json{}).value("port", 1883);
        std::string username = j.value("mqtt", json{}).value("username", std::string());
        std::string password = j.value("mqtt", json{}).value("password", std::string());

        smartmower::mqtt::MqttClient cli(broker, port, "smartmower_webadmin", username, password);
        if(!cli.connect()){ json err; err["error"] = "mqtt connect failed"; return {500, "application/json", err.dump(), {}}; }
        const std::string t_state = root + "/status/state";
        std::optional<std::string> resp;
        cli.subscribe(t_state, 1);
        cli.setMessageCallback([&](const std::string& topic, const std::vector<uint8_t>& payload){ if(topic==t_state && !resp.has_value()) resp = std::string(payload.begin(), payload.end()); });
        auto start = std::chrono::steady_clock::now();
        while(!resp.has_value()){
            cli.loop(50);
            if(std::chrono::steady_clock::now() - start > std::chrono::seconds(2)) break;
        }
        if(!resp.has_value()){ json err; err["error"] = "timeout"; err["wait_ms"] = 2000; return {504, "application/json", err.dump(), {}}; }
        return {200, "application/json", *resp, {}};
    }catch(const std::exception& e){ json err; err["error"] = std::string(e.what()); return {500, "application/json", err.dump(), {}}; }
}

static HttpResponse apiGetBattery(){
    try{
        smartmower::config::ConfigLoader cfg;
        const std::string root = cfg.rootTopic();
        const auto& j = cfg.json();
        std::string broker = j.value("mqtt", json{}).value("broker", std::string("localhost"));
        int port = j.value("mqtt", json{}).value("port", 1883);
        std::string username = j.value("mqtt", json{}).value("username", std::string());
        std::string password = j.value("mqtt", json{}).value("password", std::string());

        smartmower::mqtt::MqttClient cli(broker, port, "smartmower_webadmin", username, password);
        if(!cli.connect()){ json err; err["error"] = "mqtt connect failed"; return {500, "application/json", err.dump(), {}}; }
        const std::string t_batt = root + "/status/battery";
        std::optional<std::string> resp;
        cli.subscribe(t_batt, 1);
        cli.setMessageCallback([&](const std::string& topic, const std::vector<uint8_t>& payload){ if(topic==t_batt && !resp.has_value()) resp = std::string(payload.begin(), payload.end()); });
        auto start = std::chrono::steady_clock::now();
        while(!resp.has_value()){
            cli.loop(50);
            if(std::chrono::steady_clock::now() - start > std::chrono::seconds(2)) break;
        }
        if(!resp.has_value()) return {204, "application/json", "{}", {}};
        return {200, "application/json", *resp, {}};
    }catch(const std::exception& e){ json err; err["error"] = std::string(e.what()); return {500, "application/json", err.dump(), {}}; }
}

static HttpResponse apiPostStateCmd(const std::string& body){
    try{
        auto js = json::parse(body);
        std::string action = js.value("action", std::string());
        if(action.empty()){ json err; err["error"] = "missing action"; return {400, "application/json", err.dump(), {}}; }

        smartmower::config::ConfigLoader cfg;
        const std::string root = cfg.rootTopic();
        const auto& j = cfg.json();
        std::string broker = j.value("mqtt", json{}).value("broker", std::string("localhost"));
        int port = j.value("mqtt", json{}).value("port", 1883);
        std::string username = j.value("mqtt", json{}).value("username", std::string());
        std::string password = j.value("mqtt", json{}).value("password", std::string());

        smartmower::mqtt::MqttClient cli(broker, port, "smartmower_webadmin", username, password);
        if(!cli.connect()){ json err; err["error"] = "mqtt connect failed"; return {500, "application/json", err.dump(), {}}; }
        const std::string t_cmd = root + "/state_machine/cmd";
        json out; out["action"] = action;
        cli.publish(t_cmd, out.dump(), 1, false);
        json ok; ok["status"] = "ok"; return {200, "application/json", ok.dump(), {}};
    }catch(const std::exception& e){ json err; err["error"] = std::string(e.what()); return {400, "application/json", err.dump(), {}}; }
}

static HttpResponse apiAdminLogin(const std::string& body){
    try{
        auto js = json::parse(body);
        std::string pin = js.value("pin", std::string());
        if(checkAdminPin(pin)){
            json ok; ok["status"] = "ok";
            HttpResponse r{200, "application/json", ok.dump(), {}};
            r.headers.push_back({"Set-Cookie", std::string("admin_pin=")+pin+"; Path=/; HttpOnly; SameSite=Lax"});
            return r;
        }
        json err; err["error"] = "invalid_pin"; return {401, "application/json", err.dump(), {}};
    }catch(const std::exception& e){ json err; err["error"] = std::string(e.what()); return {400, "application/json", err.dump(), {}}; }
}

static HttpResponse apiGetConfig(){
    try{
        smartmower::config::ConfigLoader cfg;
        const std::string root = cfg.rootTopic();
        const auto& j = cfg.json();
        std::string broker = j.value("mqtt", json{}).value("broker", std::string("localhost"));
        int port = j.value("mqtt", json{}).value("port", 1883);
        std::string username = j.value("mqtt", json{}).value("username", std::string());
        std::string password = j.value("mqtt", json{}).value("password", std::string());

        smartmower::mqtt::MqttClient cli(broker, port, "smartmower_webadmin", username, password);
        if(!cli.connect()){ json err; err["error"] = "mqtt connect failed"; return {500, "application/json", err.dump(), {}}; }
        const std::string t_get = root + "/supervisor/config/get";
        const std::string t_state = root + "/supervisor/config/state";
        std::optional<std::string> resp;
        cli.subscribe(t_state, 1);
        cli.setMessageCallback([&](const std::string& topic, const std::vector<uint8_t>& payload){ if(topic==t_state && !resp.has_value()) resp = std::string(payload.begin(), payload.end()); });
        cli.publish(t_get, "{}", 1, false);
        auto start = std::chrono::steady_clock::now();
        while(!resp.has_value()){
            cli.loop(50);
            if(std::chrono::steady_clock::now() - start > std::chrono::seconds(5)) break;
        }
        if(!resp.has_value()){ json err; err["error"] = "timeout"; err["wait_ms"] = 5000; return {504, "application/json", err.dump(), {}}; }
        return {200, "application/json", *resp, {}};
    }catch(const std::exception& e){ json err; err["error"] = std::string(e.what()); return {500, "application/json", err.dump(), {}}; }
}

static HttpResponse apiPostConfig(const std::string& body){
    try{
        auto js = json::parse(body);
        smartmower::config::ConfigLoader cfg;
        const std::string root = cfg.rootTopic();
        const auto& j = cfg.json();
        std::string broker = j.value("mqtt", json{}).value("broker", std::string("localhost"));
        int port = j.value("mqtt", json{}).value("port", 1883);
        std::string username = j.value("mqtt", json{}).value("username", std::string());
        std::string password = j.value("mqtt", json{}).value("password", std::string());

        smartmower::mqtt::MqttClient cli(broker, port, "smartmower_webadmin", username, password);
        if(!cli.connect()){ json err; err["error"] = "mqtt connect failed"; return {500, "application/json", err.dump(), {}}; }
        const std::string t_set = root + "/supervisor/config/set";
        const std::string t_res = root + "/supervisor/config/result";
        std::optional<std::string> resp;
        cli.subscribe(t_res, 1);
        cli.setMessageCallback([&](const std::string& topic, const std::vector<uint8_t>& payload){ if(topic==t_res && !resp.has_value()) resp = std::string(payload.begin(), payload.end()); });
        cli.publish(t_set, js.dump(), 1, false);
        auto start = std::chrono::steady_clock::now();
        while(!resp.has_value()){
            cli.loop(50);
            if(std::chrono::steady_clock::now() - start > std::chrono::seconds(5)) break;
        }
        if(!resp.has_value()){ json err; err["error"] = "timeout"; err["wait_ms"] = 5000; return {504, "application/json", err.dump(), {}}; }
        return {200, "application/json", *resp, {}};
    }catch(const std::exception& e){ json err; err["error"] = std::string(e.what()); return {400, "application/json", err.dump(), {}}; }
}

static HttpResponse apiPostSupervisorCmd(const std::string& body){
    try{
        auto js = json::parse(body);
        std::string action = js.value("action", std::string());
        std::string service = js.value("service", std::string("costmap_node"));
        if(action != "start" && action != "stop" && action != "restart"){
            json err; err["error"] = "invalid action"; return {400, "application/json", err.dump(), {}};
        }
        smartmower::config::ConfigLoader cfg;
        const std::string root = cfg.rootTopic();
        const auto& j = cfg.json();
        std::string broker = j.value("mqtt", json{}).value("broker", std::string("localhost"));
        int port = j.value("mqtt", json{}).value("port", 1883);
        std::string username = j.value("mqtt", json{}).value("username", std::string());
        std::string password = j.value("mqtt", json{}).value("password", std::string());
        smartmower::mqtt::MqttClient cli(broker, port, "smartmower_webadmin", username, password);
        if(!cli.connect()){ json err; err["error"] = "mqtt connect failed"; return {500, "application/json", err.dump(), {}}; }
        const std::string t_cmd = root + "/supervisor/cmd";
        json cmd; cmd["action"] = action; cmd["service"] = service;
        cli.publish(t_cmd, cmd.dump(), 1, false);
        json ok; ok["status"] = "ok"; return {200, "application/json", ok.dump(), {}};
    }catch(const std::exception& e){ json err; err["error"] = std::string(e.what()); return {400, "application/json", err.dump(), {}}; }
}

// -------- static serving --------
static HttpResponse serveUserStatic(const std::string& urlPath){
    const std::vector<std::string> bases = { "/opt/smartmower/web", ".", ".." };
    auto resolve = [&](const std::string& base){
        // alias retrocompatibile: /user_static/* -> /static/*
        std::string p = urlPath;
        if(p.rfind("/user_static/",0) == 0){
            p = std::string("/static/") + p.substr(std::string("/user_static/").size());
        }
        if(p == "/" || p == "/index.html") return base + "/static/index.html";
        if(p.rfind("/static/",0) == 0) return base + p;
        return base + "/static/index.html";
    };
    std::string chosenPath, data;
    for(const auto& b: bases){ chosenPath = resolve(b); data = readFile(chosenPath); if(!data.empty()) break; }
    if(data.empty()) return {404, "text/plain", "Not found", {}};
    return {200, guessContentType(chosenPath), data, {}};
}

static HttpResponse serveAdminStatic(const std::string& urlPath){
    const std::vector<std::string> bases = { "/opt/smartmower/web", ".", ".." };
    auto resolve = [&](const std::string& base){
        if(urlPath == "/admin" || urlPath == "/admin/") return base + "/static/admin/index.html";
        if(urlPath.rfind("/static/admin/",0) == 0) return base + urlPath;
        return base + "/static/admin/index.html";
    };
    std::string chosenPath, data;
    for(const auto& b: bases){ chosenPath = resolve(b); data = readFile(chosenPath); if(!data.empty()) break; }
    if(data.empty()) return {404, "text/plain", "Not found", {}};
    return {200, guessContentType(chosenPath), data, {}};
}

// -------- HTTP handler --------
struct RequestContext { std::string body; };

static MHD_Result handler(void* /*cls*/, struct MHD_Connection* connection,
                   const char* url, const char* method,
                   const char* /*version*/, const char* upload_data,
                   size_t* upload_data_size, void** con_cls){
    auto* ctx = reinterpret_cast<RequestContext*>(*con_cls);
    if(!ctx){ ctx = new RequestContext(); *con_cls = ctx; return MHD_YES; }
    if(std::string(method) == "POST"){
        if(*upload_data_size != 0){ ctx->body.append(upload_data, upload_data + *upload_data_size); *upload_data_size = 0; return MHD_YES; }
    }

    HttpResponse res;
    std::string path(url);
    if(path == "/api/state" && std::string(method)=="GET"){
        res = apiGetState();
    } else if(path == "/api/battery" && std::string(method)=="GET"){
        res = apiGetBattery();
    } else if(path == "/api/state/cmd" && std::string(method)=="POST"){
        res = apiPostStateCmd(ctx->body);
    } else if(path == "/api/admin/login" && std::string(method)=="POST"){
        res = apiAdminLogin(ctx->body);
    } else if(path == "/api/config" && std::string(method)=="GET"){
        if(!isAuthorizedAdmin(connection)){
            res = {401, "application/json", "{\"error\":\"unauthorized\"}", {}};
        } else {
            res = apiGetConfig();
        }
    } else if(path == "/api/config" && std::string(method)=="POST"){
        if(!isAuthorizedAdmin(connection)){
            res = {401, "application/json", "{\"error\":\"unauthorized\"}", {}};
        } else {
            res = apiPostConfig(ctx->body);
        }
    } else if(path == "/api/supervisor/cmd" && std::string(method)=="POST"){
        if(!isAuthorizedAdmin(connection)){
            res = {401, "application/json", "{\"error\":\"unauthorized\"}", {}};
        } else {
            res = apiPostSupervisorCmd(ctx->body);
        }
    } else if(path == "/api/manual/enter" && std::string(method)=="POST"){
        res = apiManualEnter();
    } else if(path == "/api/manual/exit" && std::string(method)=="POST"){
        res = apiManualExit();
    } else if(path == "/api/manual/cmd_vel" && std::string(method)=="POST"){
        res = apiManualCmdVel(ctx->body);
    } else if(path == "/api/manual/blade" && std::string(method)=="POST"){
        res = apiManualBlade(ctx->body);
    } else {
        bool adminArea = (path == "/admin" || path == "/admin/" || path.rfind("/static/admin/",0)==0 || path.rfind("/admin_static/",0)==0);
        if(adminArea){
            // Consenti sempre il caricamento delle risorse statiche e della pagina /admin
            // L'autenticazione è gestita dalle API e dal client.
            res = serveAdminStatic(path);
        } else {
            res = serveUserStatic(path);
        }
    }

    struct MHD_Response* response = MHD_create_response_from_buffer(res.body.size(), (void*)res.body.data(), MHD_RESPMEM_MUST_COPY);
    if(!response){ delete ctx; *con_cls = nullptr; return MHD_NO; }
    MHD_add_response_header(response, "Content-Type", res.contentType.c_str());
    for(const auto& h: res.headers){ MHD_add_response_header(response, h.first.c_str(), h.second.c_str()); }
    MHD_Result ret = MHD_queue_response(connection, res.status, response);
    MHD_destroy_response(response);
    delete ctx; *con_cls = nullptr;
    return ret;
}

int main(){
    const char* env_port = std::getenv("SMARTMOWER_WEB_PORT");
    unsigned short port = env_port ? static_cast<unsigned short>(std::stoi(env_port)) : 8080;
    struct MHD_Daemon* d = MHD_start_daemon(MHD_USE_SELECT_INTERNALLY, port, nullptr, nullptr, &handler, nullptr, MHD_OPTION_END);
    if(!d) return 1;
    while(true){ std::this_thread::sleep_for(std::chrono::seconds(60)); }
    MHD_stop_daemon(d);
    return 0;
}
// --- END OF SERVER IMPLEMENTATION ---
#if 0  // duplicate block disabled
    int status{200};
    std::string contentType{"application/json"};
    std::string body{"{}"};
    std::vector<std::pair<std::string,std::string>> headers; // es. Set-Cookie
};

static std::string readFile(const std::string& path) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs.is_open()) return {};
    std::ostringstream ss; ss << ifs.rdbuf();
    return ss.str();
}

static HttpResponse apiGetState(){
    try{
        smartmower::config::ConfigLoader cfg;
        const std::string root = cfg.rootTopic();
        const auto& j = cfg.json();
        std::string broker = j.value("mqtt", json{}).value("broker", std::string("localhost"));
        int port = j.value("mqtt", json{}).value("port", 1883);
        std::string username = j.value("mqtt", json{}).value("username", std::string());
        std::string password = j.value("mqtt", json{}).value("password", std::string());

        smartmower::mqtt::MqttClient cli(broker, port, "smartmower_webadmin", username, password);
        if(!cli.connect()) return {500, "application/json", "{\"error\":\"mqtt connect failed\"}", {}};
        const std::string t_state = root + "/status/state";
        std::optional<std::string> resp;
        cli.subscribe(t_state, 1);
        cli.setMessageCallback([&](const std::string& topic, const std::vector<uint8_t>& payload){
            if(topic == t_state && !resp.has_value()) resp = std::string(payload.begin(), payload.end());
        });
        auto start = std::chrono::steady_clock::now();
        while(!resp.has_value()){
            cli.loop(50);
            if (std::chrono::steady_clock::now() - start > std::chrono::seconds(2)) break;
        }
        if(!resp.has_value()) return {504, "application/json", "{\"error\":\"timeout\",\"wait_ms\":2000}", {}};
        return {200, "application/json", *resp, {}};
    }catch(const std::exception& e){
        json err; err["error"] = std::string(e.what());
        return {500, "application/json", err.dump(), {}};
    }
}

static HttpResponse apiPostStateCmd(const std::string& body){
    try{
        auto js = json::parse(body);
        std::string action = js.value("action", std::string());
        if(action.empty()) return {400, "application/json", "{\\"error\\":\\"missing action\\"}", {}};
        smartmower::config::ConfigLoader cfg;
        const std::string root = cfg.rootTopic();
        const auto& j = cfg.json();
        std::string broker = j.value("mqtt", json{}).value("broker", std::string("localhost"));
        int port = j.value("mqtt", json{}).value("port", 1883);
        std::string username = j.value("mqtt", json{}).value("username", std::string());
        std::string password = j.value("mqtt", json{}).value("password", std::string());

        smartmower::mqtt::MqttClient cli(broker, port, "smartmower_webadmin", username, password);
        if(!cli.connect()) return {500, "application/json", "{\"error\":\"mqtt connect failed\"}", {}};
        const std::string t_cmd = root + "/state_machine/cmd";
        json out; out["action"] = action;
        cli.publish(t_cmd, out.dump(), 1, false);
        return {200, "application/json", "{\\"status\\":\\"ok\\"}", {}};
    }catch(const std::exception& e){
        json err; err["error"] = std::string(e.what());
        return {400, "application/json", err.dump(), {}};
    }
}

static HttpResponse apiGetBattery(){
    try{
        smartmower::config::ConfigLoader cfg;
        const std::string root = cfg.rootTopic();
        const auto& j = cfg.json();
        std::string broker = j.value("mqtt", json{}).value("broker", std::string("localhost"));
        int port = j.value("mqtt", json{}).value("port", 1883);
        std::string username = j.value("mqtt", json{}).value("username", std::string());
        std::string password = j.value("mqtt", json{}).value("password", std::string());
        smartmower::mqtt::MqttClient cli(broker, port, "smartmower_webadmin", username, password);
        if(!cli.connect()) return {500, "application/json", "{\"error\":\"mqtt connect failed\"}", {}};
        const std::string t_batt = root + "/status/battery";
        std::optional<std::string> resp;
        cli.subscribe(t_batt, 1);
        cli.setMessageCallback([&](const std::string& topic, const std::vector<uint8_t>& payload){ if(topic==t_batt && !resp.has_value()) resp = std::string(payload.begin(), payload.end()); });
        auto start = std::chrono::steady_clock::now();
        while(!resp.has_value()){
            cli.loop(50);
            if (std::chrono::steady_clock::now() - start > std::chrono::seconds(2)) break;
        }
        if(!resp.has_value()) return {204, "application/json", "{}", {}};
        return {200, "application/json", *resp, {}};
    }catch(const std::exception& e){
        json err; err["error"] = std::string(e.what());
        return {500, "application/json", err.dump(), {}};
    }
}

static std::string getCookie(struct MHD_Connection* connection, const std::string& name){
    const char* ck = MHD_lookup_connection_value(connection, MHD_HEADER_KIND, "Cookie");
    if(!ck) return {};
    std::string s(ck);
    // parsing semplice: "k1=v1; k2=v2"
    size_t pos = 0;
    while(pos < s.size()){
        // skip spazi e ';'
        while(pos < s.size() && (s[pos]==' ' || s[pos]==';')) pos++;
        size_t eq = s.find('=', pos);
        if(eq == std::string::npos) break;
        std::string k = s.substr(pos, eq-pos);
        size_t end = s.find(';', eq+1);
        std::string v = s.substr(eq+1, (end==std::string::npos? s.size():end) - (eq+1));
        if(k == name) return v;
        if(end == std::string::npos) break;
        pos = end+1;
    }
    return {};
}

static bool checkAdminPin(const std::string& pin){
    try{
        smartmower::config::ConfigLoader cfg;
        const auto& j = cfg.json();
        std::string expected = j.value("system", json{}).value("admin_pin", std::string("123456"));
        return !pin.empty() && pin == expected;
    }catch(...){ return pin == "123456"; }
}

static bool isAuthorizedAdmin(struct MHD_Connection* connection){
    std::string pin = getCookie(connection, "admin_pin");
    return checkAdminPin(pin);
}

static HttpResponse apiAdminLogin(const std::string& body){
    try{
        auto js = json::parse(body);
        std::string pin = js.value("pin", std::string());
        if(checkAdminPin(pin)){
            HttpResponse r{200, "application/json", "{\"status\":\"ok\"}", {}};
            // Cookie valido per path /admin e /static
            r.headers.push_back({"Set-Cookie", std::string("admin_pin=")+pin+"; Path=/; HttpOnly; SameSite=Lax"});
            return r;
        }
        return {401, "application/json", "{\"error\":\"invalid_pin\"}", {}};
    }catch(const std::exception& e){
        json err; err["error"] = std::string(e.what());
        return {400, "application/json", err.dump(), {}};
    }
}

static HttpResponse serveUserStatic(const std::string& urlPath){
    const std::vector<std::string> bases = { "/opt/smartmower/web", ".", ".." };
    auto resolve = [&](const std::string& base){
        if(urlPath == "/" || urlPath == "/index.html") return base + "/static/index.html";
        if(urlPath.rfind("/static/",0) == 0) return base + urlPath;
        return base + "/static/index.html";
    };
    std::string chosenPath, data;
    for(const auto& b: bases){
        chosenPath = resolve(b);
        data = readFile(chosenPath);
        if(!data.empty()) break;
    }
    if(data.empty()) return {404, "text/plain", "Not found", {}};
    std::string ctype = "text/html";
    if (chosenPath.rfind(".js") != std::string::npos) ctype = "application/javascript";
    else if (chosenPath.rfind(".css") != std::string::npos) ctype = "text/css";
    else if (chosenPath.rfind(".json") != std::string::npos) ctype = "application/json";
    return {200, ctype, data, {}};
}

static HttpResponse apiPostSupervisorCmd(const std::string& body) {
    try {
        // body atteso: { "action": "start|stop|restart", "service": "costmap_node" }
        auto js = json::parse(body);
        std::string action = js.value("action", std::string());
        std::string service = js.value("service", std::string("costmap_node"));
        if (action != "start" && action != "stop" && action != "restart") {
            return {400, "application/json", "{\\"error\\":\\"invalid action\\"}", {}};
        }

        smartmower::config::ConfigLoader cfg; // default path
        const std::string root = cfg.rootTopic();
        const auto& j = cfg.json();
        std::string broker = j.value("mqtt", json{}).value("broker", std::string("localhost"));
        int port = j.value("mqtt", json{}).value("port", 1883);
        std::string username = j.value("mqtt", json{}).value("username", std::string());
        std::string password = j.value("mqtt", json{}).value("password", std::string());

        smartmower::mqtt::MqttClient cli(broker, port, "smartmower_webadmin", username, password);
        if (!cli.connect()) return {500, "application/json", "{\\"error\\":\\"mqtt connect failed\\"}", {}};
        const std::string t_cmd = root + "/supervisor/cmd";
        json cmd; cmd["action"] = action; cmd["service"] = service;
        cli.publish(t_cmd, cmd.dump(), 1, false);
        return {200, "application/json", std::string("{\\"status\\":\\"ok\\"}"), {}};
    } catch (const std::exception& e) {
        json err; err["error"] = std::string(e.what());
        return {400, "application/json", err.dump(), {}};
    }
}

static HttpResponse serveAdminStatic(const std::string& urlPath) {
    // Percorsi base in ordine di priorità: install, sviluppo (cwd), sviluppo (parent)
    const std::vector<std::string> bases = { "/opt/smartmower/web", ".", ".." };

    auto resolve = [&](const std::string& base) -> std::string {
        if (urlPath == "/admin" || urlPath == "/admin/") {
            return base + "/static/admin/index.html";
        } else if (urlPath.rfind("/static/admin/", 0) == 0) {
            return base + urlPath;
        } else {
            // fallback SPA admin
            return base + "/static/admin/index.html";
        }
    };

    std::string chosenPath;
    std::string data;
    for (const auto& b : bases) {
        chosenPath = resolve(b);
        data = readFile(chosenPath);
        if (!data.empty()) break;
    }
    if (data.empty()) return {404, "text/plain", "Not found", {}};

    std::string ctype = "text/html";
    if (chosenPath.rfind(".js") != std::string::npos) ctype = "application/javascript";
    else if (chosenPath.rfind(".css") != std::string::npos) ctype = "text/css";
    else if (chosenPath.rfind(".json") != std::string::npos) ctype = "application/json";
    return {200, ctype, data, {}};
}

static HttpResponse apiGetConfig() {
    try {
        smartmower::config::ConfigLoader cfg; // default path
        const std::string root = cfg.rootTopic();
        const auto& j = cfg.json();
        std::string broker = j.value("mqtt", json{}).value("broker", std::string("localhost"));
        int port = j.value("mqtt", json{}).value("port", 1883);
        std::string username = j.value("mqtt", json{}).value("username", std::string());
        std::string password = j.value("mqtt", json{}).value("password", std::string());

        smartmower::mqtt::MqttClient cli(broker, port, "smartmower_webadmin", username, password);
        if (!cli.connect()) return {500, "application/json", "{\"error\":\"mqtt connect failed\"}", {}};
        const std::string t_get = root + "/supervisor/config/get";
        const std::string t_state = root + "/supervisor/config/state";
        std::optional<std::string> resp;
        std::mutex m;
        cli.subscribe(t_state, 1);
        cli.setMessageCallback([&](const std::string& topic, const std::vector<uint8_t>& payload){
            if (topic == t_state && !resp.has_value()) {
                resp = std::string(payload.begin(), payload.end());
            }
        });
        // invia GET
        cli.publish(t_get, "{}", 1, false);
        // attendi risposta fino a 5s
        auto start = std::chrono::steady_clock::now();
        while (!resp.has_value()) {
            cli.loop(50);
            if (std::chrono::steady_clock::now() - start > std::chrono::seconds(5)) break;
        }
        if (!resp.has_value()) {
            json err; err["error"] = "timeout"; err["wait_ms"] = 5000; err["subscribe"] = t_state; err["publish"] = t_get;
            return {504, "application/json", err.dump(), {}};
        }
        return {200, "application/json", *resp, {}};
    } catch (const std::exception& e) {
        json err; err["error"] = std::string(e.what());
        return {500, "application/json", err.dump(), {}};
    }
}

static HttpResponse apiPostConfig(const std::string& body) {
    try {
        auto js = json::parse(body);
        smartmower::config::ConfigLoader cfg;
        const std::string root = cfg.rootTopic();
        const auto& j = cfg.json();
        std::string broker = j.value("mqtt", json{}).value("broker", std::string("localhost"));
        int port = j.value("mqtt", json{}).value("port", 1883);
        std::string username = j.value("mqtt", json{}).value("username", std::string());
        std::string password = j.value("mqtt", json{}).value("password", std::string());

        smartmower::mqtt::MqttClient cli(broker, port, "smartmower_webadmin", username, password);
        if (!cli.connect()) return {500, "application/json", "{\"error\":\"mqtt connect failed\"}", {}};
        const std::string t_set = root + "/supervisor/config/set";
        const std::string t_res = root + "/supervisor/config/result";
        std::optional<std::string> resp;
        cli.subscribe(t_res, 1);
        cli.setMessageCallback([&](const std::string& topic, const std::vector<uint8_t>& payload){
            if (topic == t_res && !resp.has_value()) resp = std::string(payload.begin(), payload.end());
        });
        cli.publish(t_set, js.dump(), 1, false);
        auto start = std::chrono::steady_clock::now();
        while (!resp.has_value()) {
            cli.loop(50);
            if (std::chrono::steady_clock::now() - start > std::chrono::seconds(5)) break;
        }
        if (!resp.has_value()) {
            json err; err["error"] = "timeout"; err["wait_ms"] = 5000; err["subscribe"] = t_res; err["publish"] = t_set;
            return {504, "application/json", err.dump(), {}};
        }
        return {200, "application/json", *resp, {}};
    } catch (const std::exception& e) {
        json err; err["error"] = std::string(e.what());
        return {400, "application/json", err.dump(), {}};
    }
}

struct RequestContext {
    std::string body;
};

static MHD_Result handler(void* /*cls*/, struct MHD_Connection* connection,
                   const char* url, const char* method,
                   const char* /*version*/, const char* upload_data,
                   size_t* upload_data_size, void** con_cls) {
    auto* ctx = reinterpret_cast<RequestContext*>(*con_cls);
    if (!ctx) {
        ctx = new RequestContext();
        *con_cls = ctx;
        return MHD_YES;
    }
    if (std::string(method) == "POST") {
        if (*upload_data_size != 0) {
            ctx->body.append(upload_data, upload_data + *upload_data_size);
            *upload_data_size = 0;
            return MHD_YES;
        }
    }

    HttpResponse res;
    std::string path(url);
    if (path == "/api/config" && std::string(method) == "GET") {
        res = apiGetConfig();
    } else if (path == "/api/config" && std::string(method) == "POST") {
        res = apiPostConfig(ctx->body);
    } else if (path == "/api/admin/login" && std::string(method) == "POST") {
        res = apiAdminLogin(ctx->body);
    } else if (path == "/api/state" && std::string(method) == "GET") {
        res = apiGetState();
    } else if (path == "/api/state/cmd" && std::string(method) == "POST") {
        res = apiPostStateCmd(ctx->body);
    } else if (path == "/api/battery" && std::string(method) == "GET") {
        res = apiGetBattery();
    } else if (path == "/api/supervisor/cmd" && std::string(method) == "POST") {
        res = apiPostSupervisorCmd(ctx->body);
    } else {
        // Routing statico: user area su / e /static/* (libera); admin su /admin e /static/admin/* con auth
        bool adminArea = (path == "/admin" || path == "/admin/" || path.rfind("/static/admin/",0)==0);
        if(adminArea){
            if(!isAuthorizedAdmin(connection)){
                res = {401, "text/plain", "Unauthorized", {}};
            } else {
                res = serveAdminStatic(path);
            }
        } else {
            res = serveUserStatic(path);
        }
    }

    struct MHD_Response* response = MHD_create_response_from_buffer(res.body.size(), (void*)res.body.data(), MHD_RESPMEM_MUST_COPY);
    if (!response) { delete ctx; *con_cls = nullptr; return MHD_NO; }
    MHD_add_response_header(response, "Content-Type", res.contentType.c_str());
    for(const auto& h: res.headers){ MHD_add_response_header(response, h.first.c_str(), h.second.c_str()); }
    MHD_Result ret = MHD_queue_response(connection, res.status, response);
    MHD_destroy_response(response);
    delete ctx; *con_cls = nullptr;
    return ret;
}

int main() {
    const char* env_port = std::getenv("SMARTMOWER_WEB_PORT");
    unsigned short port = env_port ? static_cast<unsigned short>(std::stoi(env_port)) : 8080;
    struct MHD_Daemon* d = MHD_start_daemon(MHD_USE_SELECT_INTERNALLY, port, nullptr, nullptr, &handler, nullptr, MHD_OPTION_END);
    if (!d) return 1;
    // Rimani attivo
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(60));
    }
    MHD_stop_daemon(d);
    return 0;
}

#endif // duplicate block disabled
