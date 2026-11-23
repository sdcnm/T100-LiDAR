#include "t100_protocol.h"
#include <assert.h>

int main(void) {
    static const uint8_t golden[] = {0x55,0x0D,0x04,0x33,0x28,0xAD,0x10,0x48,0x40,0x00,0x07,0x76,0xF9};
    uint8_t data[1023];
    uint8_t payload[1010] = {0};
    size_t size = t100_frame(data, sizeof data, 0, 7, 0x4810, NULL, 0);
    assert(size == sizeof golden && memcmp(data, golden, size) == 0);
    assert(t100_valid_frame(data, size));
    data[7] ^= 1;
    assert(!t100_valid_frame(data, size));
    assert(t100_frame(data, 12, 0, 7, 0, NULL, 0) == 0);
    assert(t100_frame(data, sizeof data, 0, 7, 0, NULL, 1) == 0);
    size = t100_frame(data, sizeof data, 0x32, 0x42, 65535, payload, sizeof payload);
    assert(size == 1023 && t100_valid_frame(data, size));
    assert(t100_frame(data, sizeof data, 0, 7, 0, payload, 1011) == 0);
    return 0;
}
