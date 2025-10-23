#include "protocol.hpp"

namespace proto {

uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t init) {
    uint16_t crc = init;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021; else crc <<= 1;
        }
    }
    return crc;
}

void cobs_encode(const std::vector<uint8_t>& in, std::vector<uint8_t>& out) {
    out.clear();
    out.reserve(in.size() + 2);
    size_t code_index = 0;
    uint8_t code = 1;
    out.push_back(0); // placeholder
    for (uint8_t byte : in) {
        if (byte == 0) {
            out[code_index] = code;
            code_index = out.size();
            out.push_back(0);
            code = 1;
        } else {
            out.push_back(byte);
            code++;
            if (code == 0xFF) {
                out[code_index] = code;
                code_index = out.size();
                out.push_back(0);
                code = 1;
            }
        }
    }
    out[code_index] = code;
    out.push_back(0x00);
}

void pack_and_encode(uint8_t msg_id, const std::vector<uint8_t>& payload,
                     uint8_t seq, uint32_t ts_ms,
                     std::vector<uint8_t>& encoded) {
    std::vector<uint8_t> buf;
    buf.reserve(7 + payload.size() + 2);
    buf.push_back(msg_id);
    buf.push_back(static_cast<uint8_t>(payload.size()));
    buf.push_back(seq);
    buf.push_back(static_cast<uint8_t>(ts_ms & 0xFF));
    buf.push_back(static_cast<uint8_t>((ts_ms >> 8) & 0xFF));
    buf.push_back(static_cast<uint8_t>((ts_ms >> 16) & 0xFF));
    buf.push_back(static_cast<uint8_t>((ts_ms >> 24) & 0xFF));
    buf.insert(buf.end(), payload.begin(), payload.end());
    uint16_t crc = crc16_ccitt(buf.data(), buf.size());
    buf.push_back(static_cast<uint8_t>(crc & 0xFF));
    buf.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
    cobs_encode(buf, encoded);
}

// ---- Decoder side ----
static constexpr size_t HEADER_SIZE = 7; // id(1) len(1) seq(1) ts(4)

bool cobs_decode(const std::vector<uint8_t>& in, std::vector<uint8_t>& out) {
    out.clear();
    size_t idx = 0;
    while (idx < in.size()) {
        uint8_t code = in[idx++];
        if (code == 0) return false; // invalid
        size_t end = idx + code - 1;
        if (end > in.size()) return false; // truncated
        out.insert(out.end(), in.begin() + idx, in.begin() + end);
        idx = end;
        if (code < 0xFF && idx < in.size()) out.push_back(0);
    }
    return true;
}

COBSStreamDecoder::COBSStreamDecoder(size_t max_frame) : max_(max_frame) {}

void COBSStreamDecoder::feed(const uint8_t* data, size_t len) {
    if (!data || len == 0) return;
    for (size_t i = 0; i < len; ++i) {
        uint8_t b = data[i];
        if (b == 0) {
            if (!buf_.empty()) {
                std::vector<uint8_t> decoded;
                if (cobs_decode(buf_, decoded)) {
                    frames_.push_back(std::move(decoded));
                }
            }
            buf_.clear();
        } else {
            if (buf_.size() < max_) buf_.push_back(b); else buf_.clear();
        }
    }
}

bool COBSStreamDecoder::pop_frame(DecodedFrame& outf) {
    if (frames_.empty()) return false;
    std::vector<uint8_t> f = std::move(frames_.front());
    frames_.erase(frames_.begin());
    return parse_frame(f, outf);
}

bool parse_frame(const std::vector<uint8_t>& decoded, DecodedFrame& out) {
    if (decoded.size() < HEADER_SIZE + 2) return false;
    const size_t total = decoded.size();
    // CRC check
    uint16_t rx_crc = static_cast<uint16_t>(decoded[total-2]) |
                      (static_cast<uint16_t>(decoded[total-1]) << 8);
    uint16_t calc = crc16_ccitt(decoded.data(), total - 2);
    if (rx_crc != calc) return false;
    uint8_t msg_id = decoded[0];
    uint8_t len = decoded[1];
    uint8_t seq = decoded[2];
    uint32_t ts = static_cast<uint32_t>(decoded[3]) |
                  (static_cast<uint32_t>(decoded[4]) << 8) |
                  (static_cast<uint32_t>(decoded[5]) << 16) |
                  (static_cast<uint32_t>(decoded[6]) << 24);
    if (HEADER_SIZE + len != total - 2) return false;
    out.msg_id = msg_id;
    out.seq = seq;
    out.ts_ms = ts;
    out.payload.assign(decoded.begin() + HEADER_SIZE, decoded.begin() + HEADER_SIZE + len);
    return true;
}

} // namespace proto

