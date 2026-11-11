#pragma once
#include <cstdint>
#include <deque>
class MF_Serial {
 public:
  std::deque<uint8_t> bytes;
  int available() { return bytes.size(); }
  int read() { int v = bytes.front(); bytes.pop_front(); return v; }
};
