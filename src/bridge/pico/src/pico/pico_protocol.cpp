#include "pico/pico_protocol.h"
#include <cstring>

namespace pico {

bool ProtocolParser::validateChecksum(const uint8_t* data, size_t length) {
    if (length < 2) return false; // Almeno tipo e checksum
    
    uint8_t received_checksum = data[length - 1];
    uint8_t calculated_checksum = calculateChecksum(data, length - 1);
    
    return received_checksum == calculated_checksum;
}

uint8_t ProtocolParser::calculateChecksum(const uint8_t* data, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; ++i) {
        checksum ^= data[i]; // XOR semplice per il checksum
    }
    return checksum;
}

size_t ProtocolParser::serializeMotorCommand(const MotorCommand& cmd, uint8_t* buffer, size_t buffer_size) {
    if (buffer_size < sizeof(MotorCommand)) {
        return 0; // Buffer troppo piccolo
    }
    
    std::memcpy(buffer, &cmd, sizeof(MotorCommand));
    buffer[sizeof(MotorCommand)] = calculateChecksum(buffer, sizeof(MotorCommand));
    
    return sizeof(MotorCommand) + 1; // +1 per il checksum
}

size_t ProtocolParser::serializeSystemCommand(const SystemCommand& cmd, uint8_t* buffer, size_t buffer_size) {
    if (buffer_size < sizeof(SystemCommand)) {
        return 0; // Buffer troppo piccolo
    }
    
    std::memcpy(buffer, &cmd, sizeof(SystemCommand));
    buffer[sizeof(SystemCommand)] = calculateChecksum(buffer, sizeof(SystemCommand));
    
    return sizeof(SystemCommand) + 1; // +1 per il checksum
}

bool ProtocolParser::deserializeMessage(const uint8_t* data, size_t length, MessageType& type, void* message) {
    // Verifica la lunghezza minima
    if (length < 2) return false; // Almeno tipo e checksum
    
    // Verifica il checksum
    if (!validateChecksum(data, length)) {
        return false;
    }
    
    // Estrai il tipo di messaggio
    type = static_cast<MessageType>(data[0]);
    
    // Copia i dati nella struttura appropriata
    switch (type) {
        case MessageType::SENSOR_DATA:
            if (length != sizeof(SensorData) + 1) return false; // +1 per il checksum
            std::memcpy(message, data, sizeof(SensorData));
            break;
            
        case MessageType::STATUS_REPORT:
            if (length != sizeof(StatusReport) + 1) return false;
            std::memcpy(message, data, sizeof(StatusReport));
            break;
            
        case MessageType::MOTOR_COMMAND:
            if (length != sizeof(MotorCommand) + 1) return false;
            std::memcpy(message, data, sizeof(MotorCommand));
            break;
            
        case MessageType::SYSTEM_COMMAND:
            if (length != sizeof(SystemCommand) + 1) return false;
            std::memcpy(message, data, sizeof(SystemCommand));
            break;
            
        default:
            return false; // Tipo di messaggio sconosciuto
    }
    
    return true;
}

} // namespace pico
