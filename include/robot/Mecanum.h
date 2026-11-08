#pragma once

#include <stdint.h>

namespace mecanum {
struct WheelPowers {
  int8_t frontLeft;
  int8_t frontRight;
  int8_t rearLeft;
  int8_t rearRight;
};

inline int16_t clampAxis(int16_t value) {
  if (value > 100) return 100;
  if (value < -100) return -100;
  return value;
}

// Axes: forward is positive, strafe-right is positive, clockwise turn is positive.
// Wheel order and X-roller orientation: FL=f+s-t, FR=f-s+t,
// RL=f-s-t, RR=f+s+t. Scale all wheels together to preserve the requested vector.
inline WheelPowers mix(int16_t forward, int16_t strafeRight, int16_t clockwise) {
  const int16_t f = clampAxis(forward);
  const int16_t s = clampAxis(strafeRight);
  const int16_t t = clampAxis(clockwise);
  int16_t wheels[4] = {static_cast<int16_t>(f + s - t),
                       static_cast<int16_t>(f - s + t),
                       static_cast<int16_t>(f - s - t),
                       static_cast<int16_t>(f + s + t)};
  int16_t peak = 100;
  for (uint8_t i = 0; i < 4; ++i) {
    const int16_t magnitude = wheels[i] < 0 ? -wheels[i] : wheels[i];
    if (magnitude > peak) peak = magnitude;
  }
  if (peak > 100) {
    for (uint8_t i = 0; i < 4; ++i)
      wheels[i] = static_cast<int16_t>((static_cast<int32_t>(wheels[i]) * 100) / peak);
  }
  return {static_cast<int8_t>(wheels[0]), static_cast<int8_t>(wheels[1]),
          static_cast<int8_t>(wheels[2]), static_cast<int8_t>(wheels[3])};
}
}  // namespace mecanum
