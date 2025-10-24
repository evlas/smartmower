/**
 * @file protocol.hpp
 * @brief Definizione protocollo seriale: ID messaggi, CRC16, COBS e decoder stream.
 */
#pragma once
#include <cstdint>
#include <vector>

namespace proto {

/** \brief ID messaggio per telemetria IMU (10 float LE). */
static constexpr uint8_t MSG_IMU = 0x01;
/** \brief ID messaggio per odometria (int32 tL, int32 tR, float dt). */
static constexpr uint8_t MSG_ODOM = 0x02;
/** \brief ID messaggio distanze sonar. */
static constexpr uint8_t MSG_SONAR = 0x03;
/** \brief ID messaggio telemetria batteria. */
static constexpr uint8_t MSG_BATT = 0x04;
/** \brief ID messaggio eventi/bitfield. */
static constexpr uint8_t MSG_EVENT = 0x05;
/** \brief ID messaggio RPM lame. */
static constexpr uint8_t MSG_BLADES_RPM = 0x06;

/** \brief Comando: guida (2 float32 LE: left,right [-1..1]). */
static constexpr uint8_t MSG_CMD_DRIVE  = 0x10;
/** \brief Comando: lame (2 float32 LE). */
static constexpr uint8_t MSG_CMD_BLADES = 0x11;
/** \brief Comando: relay sicurezza (uint8 0/1). */
static constexpr uint8_t MSG_CMD_RELAY  = 0x12;
/** \brief Comando: limiti (2 float32 LE: max_abs_speed, accel_limit). */
static constexpr uint8_t MSG_CMD_LIMITS = 0x13;

/** \brief CRC16-CCITT (poly 0x1021, init 0xFFFF). */
uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t init = 0xFFFF);
/** \brief COBS-encode del buffer in, con terminatore aggiunto nello strato superiore. */
void cobs_encode(const std::vector<uint8_t>& in, std::vector<uint8_t>& out);

/** \brief Impacchetta e COBS-encoda un frame.
 *  Formato: [id(1)][len(1)][seq(1)][ts_ms(4)][payload][crc16(2)] poi COBS + 0x00
 */
void pack_and_encode(uint8_t msg_id, const std::vector<uint8_t>& payload,
                     uint8_t seq, uint32_t ts_ms,
                     std::vector<uint8_t>& encoded);

/** \brief Frame decodificato dal wire (dopo COBS e CRC). */
struct DecodedFrame {
    uint8_t msg_id;
    uint8_t seq;
    uint32_t ts_ms;
    std::vector<uint8_t> payload;
};

/** \brief Decoder incrementale COBS con bufferizzazione frame. */
class COBSStreamDecoder {
public:
    /** \brief Crea decoder con dimensione massima frame. */
    explicit COBSStreamDecoder(size_t max_frame = 512);
    /** \brief Alimenta dati grezzi dal driver UART. */
    void feed(const uint8_t* data, size_t len);
    /** \brief Estrae un frame decodificato, se presente. */
    bool pop_frame(DecodedFrame& out);
private:
    std::vector<uint8_t> buf_;
    std::vector<std::vector<uint8_t>> frames_;
    size_t max_;
};

/** \brief COBS-decode di un singolo frame (senza delimitatore finale 0x00). */
bool cobs_decode(const std::vector<uint8_t>& in, std::vector<uint8_t>& out);
/** \brief Parsifica header e payload, verifica CRC. */
bool parse_frame(const std::vector<uint8_t>& decoded, DecodedFrame& out);

} // namespace proto

