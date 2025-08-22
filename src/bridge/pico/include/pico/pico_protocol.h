#ifndef PICO_PROTOCOL_H
#define PICO_PROTOCOL_H

#include <cstdint>
#include <string>

namespace pico {

// Framing: [SOF(1)=0xAA][LEN(2, little endian) = bytes(PAYLOAD+CHK)][PAYLOAD][CHK]
// PAYLOAD resta il pacchetto originale (struct) che inizia con il campo 'type'.
// Il CHECKSUM è XOR di tutti i byte del PAYLOAD (non include SOF né LEN).
static constexpr uint8_t PICO_SOF = 0xAA;

// Tipi di messaggio
enum class MessageType : uint8_t {
    SENSOR_DATA = 0x01,
    STATUS_REPORT = 0x02,
    MOTOR_COMMAND = 0x10,
    SYSTEM_COMMAND = 0x11
};

// Struttura per i dati dei sensori (0x01)
#pragma pack(push, 1)
struct SensorData {
    uint8_t type;           // 0x01
    uint32_t timestamp;     // Timestamp in ms
    
    // Dati IMU
    float accel[3];         // Accelerometro [m/s²] (x, y, z)
    float gyro[3];          // Giroscopio [rad/s] (x, y, z)
    float mag[3];           // Magnetometro [uT] (x, y, z)
    
    // Sensori a ultrasuoni [m]
    float us_distances[3];   // Distanze (sinistra, centro, destra)
    
    // Dati di potenza
    float bus_voltage;      // Tensione del bus [V]
    float current;          // Corrente assorbita [A]
    
    // Flag di sicurezza come 4 byte separati [emergency, rain, bumper, lift]
    uint8_t safety_flags[4];
};

// Struttura per il report di stato (0x02)
struct StatusReport {
    uint8_t type;           // 0x02
    uint32_t timestamp;     // Timestamp in ms
    float motor_speeds[4];   // Velocità motori [-1.0, 1.0]
    float motor_rpm[4];      // RPM motori
    uint32_t encoder_counts[4]; // Conteggi encoder
    uint8_t system_flags;    // Flag di sistema (1B)
    uint8_t relay_state;     // Stato relè (1B)
};

// Struttura per il comando motori (0x10)
struct MotorCommand {
    uint8_t type;           // 0x10
    float left_speed;       // Velocità motore sinistro [-1.0, 1.0]
    float right_speed;      // Velocità motore destro [-1.0, 1.0]
    float blade1_speed;     // Velocità lama 1 [0.0, 1.0]
    float blade2_speed;     // Velocità lama 2 [0.0, 1.0]
};

// Struttura per il comando di sistema (0x11)
struct SystemCommand {
    uint8_t type;           // 0x11
    uint8_t command_id;     // ID comando
    float value;            // Valore del comando
};
#pragma pack(pop)

// ID comandi di sistema
enum class SystemCommandId : uint8_t {
    EMERGENCY_STOP = 0x01,
    RESET = 0x02,
    SET_RELAY = 0x03,       // Imposta stato relè (value: 0.0/1.0)
    CALIBRATE = 0x04,       // Calibrazione IMU/Magnetometro
    RESET_ENCODERS = 0x05   // Reset contatori encoder
};

// Classe per il parsing dei messaggi
class ProtocolParser {
public:
    // Verifica se i dati ricevuti contengono un messaggio valido
    static bool validateChecksum(const uint8_t* data, size_t length);
    
    // Calcola il checksum dei dati
    static uint8_t calculateChecksum(const uint8_t* data, size_t length);
    
    // Serializza un comando motori in un buffer
    static size_t serializeMotorCommand(const MotorCommand& cmd, uint8_t* buffer, size_t buffer_size);
    
    // Serializza un comando di sistema in un buffer
    static size_t serializeSystemCommand(const SystemCommand& cmd, uint8_t* buffer, size_t buffer_size);
    
    // Deserializza un messaggio dai dati ricevuti
    static bool deserializeMessage(const uint8_t* data, size_t length, MessageType& type, void* message);
};

} // namespace pico

#endif // PICO_PROTOCOL_H
