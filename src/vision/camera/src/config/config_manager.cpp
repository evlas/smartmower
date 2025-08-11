#include "config/config_manager.h"
#include <fstream>
#include <sstream>
#include <iostream>

namespace config {

ConfigManager::ConfigManager(const std::string& path) : path_(path), root_(nullptr) {}

ConfigManager::~ConfigManager() {
    if (root_) {
        cJSON_Delete(root_);
    }
}

bool ConfigManager::load() {
    std::ifstream file(path_);
    if (!file.is_open()) {
        std::cerr << "Failed to open config file: " << path_ << std::endl;
        return false;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string content = buffer.str();

    root_ = cJSON_Parse(content.c_str());
    if (!root_) {
        std::cerr << "Failed to parse config file: " << cJSON_GetErrorPtr() << std::endl;
        return false;
    }

    return true;
}

cJSON* ConfigManager::findNode(const std::string& key) {
    if (!root_) return nullptr;
    
    size_t start = 0;
    size_t end = key.find('.');
    std::string current = key.substr(0, end);
    
    cJSON* node = cJSON_GetObjectItemCaseSensitive(root_, current.c_str());
    if (!node) return nullptr;
    
    while (end != std::string::npos) {
        start = end + 1;
        end = key.find('.', start);
        current = key.substr(start, end - start);
        
        node = cJSON_GetObjectItemCaseSensitive(node, current.c_str());
        if (!node) return nullptr;
    }
    
    return node;
}

std::string ConfigManager::getString(const std::string& key, const std::string& def) {
    cJSON* node = findNode(key);
    if (!node || !cJSON_IsString(node)) {
        return def;
    }
    return node->valuestring;
}

int ConfigManager::getInt(const std::string& key, int def) {
    cJSON* node = findNode(key);
    if (!node || !cJSON_IsNumber(node)) {
        return def;
    }
    return node->valueint;
}

bool ConfigManager::getBool(const std::string& key, bool def) {
    cJSON* node = findNode(key);
    if (!node || !cJSON_IsBool(node)) {
        return def;
    }
    return cJSON_IsTrue(node);
}

std::unordered_map<std::string, std::string> ConfigManager::getObject(const std::string& key) {
    std::unordered_map<std::string, std::string> result;
    cJSON* node = findNode(key);
    
    if (!node || !cJSON_IsObject(node)) {
        return result;
    }
    
    cJSON* child = node->child;
    while (child) {
        if (cJSON_IsString(child)) {
            result[child->string] = child->valuestring;
        } else if (cJSON_IsNumber(child)) {
            result[child->string] = std::to_string(child->valueint);
        } else if (cJSON_IsBool(child)) {
            result[child->string] = cJSON_IsTrue(child) ? "true" : "false";
        }
        child = child->next;
    }
    
    return result;
}

} // namespace config
