#pragma once

#include <stdint.h>

namespace rc_input {
constexpr uint16_t PULSE_MIN = 1000;
constexpr uint16_t PULSE_CENTER = 1500;
constexpr uint16_t PULSE_MAX = 2000;
constexpr uint16_t DEADBAND_US = 50;
constexpr uint16_t VALID_MIN_US = 800;
constexpr uint16_t VALID_MAX_US = 2200;
// Remaining travel after the deadband: 1000/2000 us reach +-100, and 1050/1950 us
// (just past the deadband edge) reach about +-88.
constexpr int32_t HALF_TRAVEL_US = PULSE_MAX - PULSE_CENTER - DEADBAND_US;

// A FlySky receiver outputs roughly 1000-2000 us; anything outside the valid
// window means a missing pulse, a wrong pin or an unpowered receiver.
inline bool validPulse(uint16_t pulseUs) {
  return pulseUs >= VALID_MIN_US && pulseUs <= VALID_MAX_US;
}

// 1000 us -> -100, 1500 us -> 0, 2000 us -> +100, with a deadband around center.
inline int16_t axisFromPulse(uint16_t pulseUs) {
  if (!validPulse(pulseUs)) return 0;
  const int32_t offset = static_cast<int32_t>(pulseUs) - PULSE_CENTER;
  const int32_t magnitude = offset < 0 ? -offset : offset;
  if (magnitude <= DEADBAND_US) return 0;
  int32_t scaled = ((magnitude - DEADBAND_US) * 100) / HALF_TRAVEL_US;
  if (scaled > 100) scaled = 100;
  const int16_t clamped = static_cast<int16_t>(scaled);
  return offset < 0 ? static_cast<int16_t>(-clamped) : clamped;
}
}  // namespace rc_input
