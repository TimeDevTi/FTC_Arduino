#pragma once

#include <stdint.h>
#include <stdio.h>
#include <robot/Config.h>

namespace telemetry {
constexpr uint8_t CHANNEL_COUNT = 3;

struct Snapshot {
  uint32_t millis;
  uint16_t pulse[CHANNEL_COUNT];  // CH1 strafe, CH2 forward, CH4 turn (us)
  int8_t axis[CHANNEL_COUNT];     // same order as pulse, -100..100
  int8_t wheel[robot_config::MOTOR_COUNT];  // FL, FR, RL, RR as actually applied
  bool link;
  bool armed;
};

constexpr uint8_t LINE_CAPACITY = 96;

// "T,<ms>,<ch1>,<ch2>,<ch4>,<ch1%>,<ch2%>,<ch4%>,<fl>,<fr>,<rl>,<rr>,<link>,<armed>"
inline uint8_t format(const Snapshot &snapshot, char (&line)[LINE_CAPACITY]) {
  const int written = snprintf(
      line, LINE_CAPACITY, "T,%lu,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d",
      static_cast<unsigned long>(snapshot.millis), snapshot.pulse[0], snapshot.pulse[1],
      snapshot.pulse[2], snapshot.axis[0], snapshot.axis[1], snapshot.axis[2],
      snapshot.wheel[0], snapshot.wheel[1], snapshot.wheel[2], snapshot.wheel[3],
      snapshot.link ? 1 : 0, snapshot.armed ? 1 : 0);
  if (written <= 0) {
    line[0] = '\0';
    return 0;
  }
  return static_cast<uint8_t>(written < LINE_CAPACITY ? written : LINE_CAPACITY - 1);
}
}  // namespace telemetry
