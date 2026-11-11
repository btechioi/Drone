#include "rcl/RclGizmoSerial.h"
#include <cassert>
#include <vector>
FakeConfig cfg;

static std::vector<uint8_t> frame(uint8_t cmd, std::vector<uint8_t> payload = {}) {
  std::vector<uint8_t> body = {cmd, static_cast<uint8_t>(payload.size()), 0};
  body.insert(body.end(), payload.begin(), payload.end());
  uint16_t crc = tbx_crc16(body.data(), body.size());
  std::vector<uint8_t> wire = {0xa5, 0x5a};
  wire.insert(wire.end(), body.begin(), body.end());
  wire.push_back(crc & 0xff);
  wire.push_back(crc >> 8);
  return wire;
}
static void feed(MF_Serial &ser, const std::vector<uint8_t> &wire) {
  ser.bytes.insert(ser.bytes.end(), wire.begin(), wire.end());
}
int main() {
  MF_Serial ser;
  uint16_t pwm[RCL_MAX_CH + 1];
  RclGizmoSerial receiver(&ser, pwm);
  // Alternating IDs catches CRC computed with a stale command byte.
  feed(ser, frame(3));
  assert(receiver.update());
  feed(ser, frame(1, {0xdc, 0x05})); // 1500
  assert(receiver.update() && pwm[0] == 1500);
  feed(ser, frame(2, std::vector<uint8_t>(18, 0)));
  assert(receiver.update());
  feed(ser, frame(3));
  assert(receiver.update());
  auto bad = frame(3);
  bad.back() ^= 1;
  feed(ser, bad);
  assert(!receiver.update());
  assert(receiver.frame_count() == 4 && receiver.error_count() == 1);
  for (auto invalid : {frame(0x7f), frame(3, {0}), frame(1), frame(1, {1}),
                       frame(1, std::vector<uint8_t>(42, 0)), frame(2, {0})}) {
    feed(ser, invalid);
    assert(!receiver.update());
  }
  auto nan_sticks = std::vector<uint8_t>(18, 0);
  nan_sticks[2] = 0xc0; nan_sticks[3] = 0x7f; // little-endian quiet NaN
  feed(ser, frame(2, nan_sticks));
  assert(!receiver.update());
  auto bad_arm = std::vector<uint8_t>(18, 0);
  bad_arm[16] = 2;
  feed(ser, frame(2, bad_arm));
  assert(!receiver.update());
  assert(receiver.frame_count() == 4 && receiver.error_count() == 9);
}
