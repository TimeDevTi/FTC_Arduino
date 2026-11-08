#pragma once

#include <stdint.h>

namespace robot_config {
enum DriverMode : uint8_t {
  PWM_DIR,   // single-channel drivers such as MD10C R3
  DUAL_PWM   // dual-channel drivers such as MDD3A (two PWM inputs per motor)
};

struct MotorPins {
  uint8_t inputA;
  uint8_t inputB;
  DriverMode mode;
  bool inverted;
};

// Single-board build: every driver input is driven by the UNO R4 Minima itself.
// MDD3A channels drive the front wheels; the two MD10C R3 boards drive the rear
// wheels. MDD3A uses PWM_PWM (A/B inputs), MD10C uses sign-magnitude PWM/DIR.
// All eight motor signals live on D3, D4, D5, D6, D7, D9, D10 and D11.
// Confirm motor/channel order before use.
constexpr MotorPins MOTORS[4] = {
    {3, 5, DUAL_PWM, false},   // FL: MDD3A M1A/M1B on D3/D5
    {6, 9, DUAL_PWM, false},   // FR: MDD3A M2A/M2B on D6/D9
    {10, 4, PWM_DIR, false},   // RL: MD10C PWM D10, DIR D4
    {11, 7, PWM_DIR, false}    // RR: MD10C PWM D11, DIR D7
};
constexpr uint8_t MOTOR_COUNT = 4;

// FlySky iBUS: every channel arrives on one signal wire into Serial1 RX (D0).
// The receiver's i-BUS/servo port outputs 115200 8N1 frames; see robot/Ibus.h.
// Channel numbers 1..14 map to indices 0..13.
constexpr uint8_t IBUS_RX_PIN = 0;    // D0, Serial1 RX
constexpr uint32_t IBUS_BAUD = 115200;
constexpr uint8_t IBUS_CH_STRAFE = 0;   // CH1: right stick left/right
constexpr uint8_t IBUS_CH_FORWARD = 1;  // CH2: right stick up/down
constexpr uint8_t IBUS_CH_TURN = 3;     // CH4: left stick left/right

constexpr uint32_t RC_LINK_TIMEOUT_MS = 100;  // no valid frame -> motors stop + disarm
constexpr uint32_t ARM_CENTER_HOLD_MS = 300;  // sticks must rest centered this long
constexpr int8_t ARM_CENTER_TOLERANCE = 5;    // percent
constexpr uint32_t TELEMETRY_PERIOD_MS = 50;  // 20 Hz CSV line to the USB host
}  // namespace robot_config
