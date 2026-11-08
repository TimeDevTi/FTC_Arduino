#pragma once

#include <stdint.h>

namespace ibus {
constexpr uint8_t FRAME_SIZE = 32;
constexpr uint8_t HEADER_LENGTH = 0x20;
constexpr uint8_t HEADER_COMMAND = 0x40;
constexpr uint8_t CHANNEL_COUNT = 14;
constexpr uint8_t CHANNEL_BYTES = 2;
constexpr uint8_t CHECKSUM_BYTES = 2;

// FlySky iBUS: 115200 baud 8N1, 32-byte frames at roughly 143 Hz.
//   [0]      0x20   frame length
//   [1]      0x40   command: servo/channel data
//   [2..29]  14 channels, little endian, 1000-2000 us (no signal is usually 0)
//   [30..31] checksum, little endian: 0xFFFF - sum(bytes 0..29)
class Parser {
 public:
  // Feed one received byte. Returns true when a full, checksum-correct frame was
  // decoded; channel() then holds the new values. A misaligned or corrupt frame is
  // dropped and the parser waits for the next 0x20 header to resynchronise.
  bool feed(uint8_t byte) {
    if (length_ == 0 && byte != HEADER_LENGTH) return false;  // hunt for a header
    frame_[length_++] = byte;
    if (length_ < FRAME_SIZE) return false;

    if (frame_[1] != HEADER_COMMAND || !checksumMatches()) {
      length_ = 0;
      return false;
    }

    for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
      const uint8_t offset = static_cast<uint8_t>(2 + i * CHANNEL_BYTES);
      channels_[i] = static_cast<uint16_t>(frame_[offset] |
                                           (static_cast<uint16_t>(frame_[offset + 1]) << 8));
    }
    length_ = 0;
    return true;
  }

  // Raw pulse width in microseconds; 0 until a valid frame has arrived.
  uint16_t channel(uint8_t index) const {
    return index < CHANNEL_COUNT ? channels_[index] : 0;
  }

 private:
  bool checksumMatches() const {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < FRAME_SIZE - CHECKSUM_BYTES; ++i) sum = static_cast<uint16_t>(sum + frame_[i]);
    const uint16_t received = static_cast<uint16_t>(
        frame_[FRAME_SIZE - 2] | (static_cast<uint16_t>(frame_[FRAME_SIZE - 1]) << 8));
    return static_cast<uint16_t>(0xFFFFu - sum) == received;
  }

  uint8_t frame_[FRAME_SIZE] = {0};
  uint16_t channels_[CHANNEL_COUNT] = {0};
  uint8_t length_ = 0;
};
}  // namespace ibus
