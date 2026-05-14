#pragma once
#include "Types.hpp"
#include <vector>
#include <cstring>
#include <stdexcept>
#include <arpa/inet.h>

namespace robotipc {

class Serializer {
public:
    static std::vector<uint8_t> encode(const void* data, size_t len) {
        if (len > MAX_PAYLOAD_SIZE)
            throw std::runtime_error("Payload too large");
        std::vector<uint8_t> f;
        f.reserve(FRAME_HEADER_SIZE + len);
        f.push_back(FRAME_MAGIC >> 8);
        f.push_back(FRAME_MAGIC & 0xFF);
        uint32_t nl = htonl(static_cast<uint32_t>(len));
        const uint8_t* lp = reinterpret_cast<const uint8_t*>(&nl);
        f.insert(f.end(), lp, lp+4);
        const uint8_t* dp = reinterpret_cast<const uint8_t*>(data);
        f.insert(f.end(), dp, dp+len);
        return f;
    }

    template<typename T>
    static std::vector<uint8_t> encodeStruct(const T& obj) {
        return encode(&obj, sizeof(T));
    }

    static std::vector<uint8_t> encodeVec(const std::vector<uint8_t>& v) {
        return encode(v.data(), v.size());
    }

    static int32_t peekLength(const uint8_t* hdr) {
        uint16_t m = (uint16_t(hdr[0])<<8)|hdr[1];
        if (m != FRAME_MAGIC) return -1;
        uint32_t nl; std::memcpy(&nl, hdr+2, 4);
        return int32_t(ntohl(nl));
    }

    static std::vector<uint8_t> decode(const uint8_t* buf, size_t len) {
        if (len < FRAME_HEADER_SIZE) throw std::runtime_error("Frame too short");
        uint16_t m = (uint16_t(buf[0])<<8)|buf[1];
        if (m != FRAME_MAGIC) throw std::runtime_error("Bad frame magic");
        uint32_t nl; std::memcpy(&nl, buf+2, 4);
        uint32_t pl = ntohl(nl);
        if (pl > MAX_PAYLOAD_SIZE) throw std::runtime_error("Payload too large");
        if (len < FRAME_HEADER_SIZE+pl) throw std::runtime_error("Buffer too short");
        return {buf+FRAME_HEADER_SIZE, buf+FRAME_HEADER_SIZE+pl};
    }

    template<typename T>
    static T decodeStruct(const std::vector<uint8_t>& p) {
        if (p.size() < sizeof(T)) throw std::runtime_error("Payload too small");
        T obj; std::memcpy(&obj, p.data(), sizeof(T)); return obj;
    }
};

} // namespace robotipc
