#ifndef T100_PROTOCOL_H
#define T100_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* Header-only helpers; no packed structs, allocation or socket dependencies. */
static inline uint16_t t100_le16(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static inline uint8_t t100_crc8(const uint8_t *p, size_t n) {
    uint8_t value = 0x77;
    size_t i;
    for (i = 0; i < n; ++i) {
        unsigned bit;
        value ^= p[i];
        for (bit = 0; bit < 8; ++bit)
            value = (uint8_t)((value >> 1) ^ ((value & 1) ? 0x8C : 0));
    }
    return value;
}
static inline uint16_t t100_crc16(const uint8_t *p, size_t n) {
    uint16_t value = 0x3692;
    size_t i;
    for (i = 0; i < n; ++i) {
        unsigned bit;
        value ^= p[i];
        for (bit = 0; bit < 8; ++bit)
            value = (uint16_t)((value >> 1) ^ ((value & 1) ? 0x8408 : 0));
    }
    return value;
}
static inline size_t t100_frame(uint8_t *out, size_t capacity,
                                uint8_t set, uint8_t id, uint16_t sequence,
                                const uint8_t *payload, size_t payload_size) {
    size_t length;
    uint16_t checksum;
    if (!out || payload_size > 1010 || (payload_size && !payload)) return 0;
    length = 13 + payload_size;
    if (capacity < length) return 0;
    out[0] = 0x55; out[1] = (uint8_t)length; out[2] = (uint8_t)(4 | (length >> 8));
    out[3] = t100_crc8(out, 3); out[4] = 0x28; out[5] = 0xAD;
    out[6] = (uint8_t)sequence; out[7] = (uint8_t)(sequence >> 8);
    out[8] = 0x40; out[9] = set; out[10] = id;
    if (payload_size) memcpy(out + 11, payload, payload_size);
    checksum = t100_crc16(out, length - 2);
    out[length - 2] = (uint8_t)checksum;
    out[length - 1] = (uint8_t)(checksum >> 8);
    return length;
}
static inline int t100_valid_frame(const uint8_t *data, size_t size) {
    size_t declared;
    if (!data || size < 13 || size > 1023 || data[0] != 0x55) return 0;
    declared = (size_t)data[1] | (((size_t)data[2] & 3) << 8);
    return declared == size && (data[2] >> 2) == 1 &&
        t100_crc8(data, 3) == data[3] &&
        t100_crc16(data, size - 2) == t100_le16(data + size - 2);
}

#endif
