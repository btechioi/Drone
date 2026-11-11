#pragma once
#include <cstdint>
inline uint16_t tbx_crc16(const uint8_t *data, int size) {
  uint16_t crc = 0xffff;
  while (size--) {
    crc ^= static_cast<uint16_t>(*data++) << 8;
    for (int i=0; i<8; ++i) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
  }
  return crc;
}
