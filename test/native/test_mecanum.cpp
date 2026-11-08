#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <robot/Mecanum.h>
#include <robot/Config.h>
#include <robot/Ibus.h>
#include <robot/RcInput.h>
#include <robot/Telemetry.h>

static void testDriverPinMap() {
  assert(robot_config::MOTOR_COUNT == 4);

  // FL / FR on the MDD3A (dual PWM), RL / RR on the MD10C boards (PWM + DIR).
  assert(robot_config::MOTORS[0].inputA == 3 && robot_config::MOTORS[0].inputB == 5);
  assert(robot_config::MOTORS[0].mode == robot_config::DUAL_PWM);
  assert(robot_config::MOTORS[1].inputA == 6 && robot_config::MOTORS[1].inputB == 9);
  assert(robot_config::MOTORS[1].mode == robot_config::DUAL_PWM);
  assert(robot_config::MOTORS[2].inputA == 10 && robot_config::MOTORS[2].inputB == 4);
  assert(robot_config::MOTORS[2].mode == robot_config::PWM_DIR);
  assert(robot_config::MOTORS[3].inputA == 11 && robot_config::MOTORS[3].inputB == 7);
  assert(robot_config::MOTORS[3].mode == robot_config::PWM_DIR);

  // The eight motor signals must be exactly D3..D7, D9, D10, D11.
  const uint8_t expected[] = {3, 4, 5, 6, 7, 9, 10, 11};
  uint8_t usedPins = 0;
  for (uint8_t i = 0; i < robot_config::MOTOR_COUNT; ++i) {
    const robot_config::MotorPins &motor = robot_config::MOTORS[i];
    for (uint8_t pin = 0; pin < sizeof(expected); ++pin) {
      if (motor.inputA == expected[pin] || motor.inputB == expected[pin]) ++usedPins;
    }
  }
  assert(usedPins == sizeof(expected));

  // The single iBUS wire must not sit on a motor pin either.
  assert(robot_config::IBUS_RX_PIN == 0);  // D0 = Serial1 RX
  for (uint8_t pin = 0; pin < sizeof(expected); ++pin)
    assert(robot_config::IBUS_RX_PIN != expected[pin]);

  // Channels the firmware reads, 0-based: CH1 strafe, CH2 forward, CH4 turn.
  assert(robot_config::IBUS_CH_STRAFE == 0);
  assert(robot_config::IBUS_CH_FORWARD == 1);
  assert(robot_config::IBUS_CH_TURN == 3);
}

static void testMecanumAxes() {
  mecanum::WheelPowers wheels = mecanum::mix(100, 0, 0);
  assert(wheels.frontLeft == 100 && wheels.frontRight == 100);
  assert(wheels.rearLeft == 100 && wheels.rearRight == 100);

  wheels = mecanum::mix(0, 100, 0);
  assert(wheels.frontLeft == 100 && wheels.frontRight == -100);
  assert(wheels.rearLeft == -100 && wheels.rearRight == 100);

  wheels = mecanum::mix(0, 0, 100);
  assert(wheels.frontLeft == -100 && wheels.frontRight == 100);
  assert(wheels.rearLeft == -100 && wheels.rearRight == 100);

  wheels = mecanum::mix(-100, 0, 0);
  assert(wheels.frontLeft == -100 && wheels.frontRight == -100);
  assert(wheels.rearLeft == -100 && wheels.rearRight == -100);
}

static void testMecanumNormalization() {
  const mecanum::WheelPowers wheels = mecanum::mix(100, 100, -100);
  const int16_t output[] = {wheels.frontLeft, wheels.frontRight,
                            wheels.rearLeft, wheels.rearRight};
  for (uint8_t i = 0; i < 4; ++i) assert(output[i] >= -100 && output[i] <= 100);
  assert(wheels.frontLeft == 100);
  assert(wheels.frontRight == -33);
  assert(wheels.rearLeft == 33);
  assert(wheels.rearRight == 33);

  const mecanum::WheelPowers clipped = mecanum::mix(300, -300, 0);
  assert(clipped.frontLeft == 0 && clipped.frontRight == 100);
  assert(clipped.rearLeft == 100 && clipped.rearRight == 0);
}

static void testRcPulseMapping() {
  using rc_input::axisFromPulse;
  using rc_input::validPulse;

  assert(axisFromPulse(1500) == 0);
  assert(axisFromPulse(1540) == 0);   // still inside the deadband
  assert(axisFromPulse(1460) == 0);
  // Linear from the deadband edge to the 1000/2000 us endpoint.
  assert(axisFromPulse(1950) == 88);
  assert(axisFromPulse(2000) == 100);
  assert(axisFromPulse(1000) == -100);
  assert(axisFromPulse(1050) == -88);
  assert(axisFromPulse(1700) == 33);
  assert(axisFromPulse(1300) == -33);

  assert(!validPulse(0));      // pulseIn timeout / unpowered receiver
  assert(!validPulse(799));
  assert(validPulse(1000) && validPulse(1500) && validPulse(2000));
  assert(!validPulse(2201));
  assert(axisFromPulse(0) == 0);
}

static void buildIbusFrame(const uint16_t *channels,
                           uint8_t (&frame)[ibus::FRAME_SIZE]) {
  frame[0] = ibus::HEADER_LENGTH;
  frame[1] = ibus::HEADER_COMMAND;
  for (uint8_t i = 0; i < ibus::CHANNEL_COUNT; ++i) {
    frame[2 + i * 2] = static_cast<uint8_t>(channels[i] & 0xFF);
    frame[3 + i * 2] = static_cast<uint8_t>(channels[i] >> 8);
  }
  uint16_t sum = 0;
  for (uint8_t i = 0; i < ibus::FRAME_SIZE - 2; ++i)
    sum = static_cast<uint16_t>(sum + frame[i]);
  const uint16_t checksum = static_cast<uint16_t>(0xFFFFu - sum);
  frame[30] = static_cast<uint8_t>(checksum & 0xFF);
  frame[31] = static_cast<uint8_t>(checksum >> 8);
}

static bool feedFrame(ibus::Parser &parser, const uint8_t *frame) {
  bool complete = false;
  for (uint8_t i = 0; i < ibus::FRAME_SIZE; ++i) complete = parser.feed(frame[i]);
  return complete;
}

static void testIbusParser() {
  uint16_t channels[ibus::CHANNEL_COUNT];
  for (uint8_t i = 0; i < ibus::CHANNEL_COUNT; ++i) channels[i] = 1500;
  channels[0] = 1500;  // CH1 strafe: centered
  channels[1] = 2000;  // CH2 forward: full ahead
  channels[3] = 1000;  // CH4 turn: full counter-clockwise

  uint8_t frame[ibus::FRAME_SIZE];
  buildIbusFrame(channels, frame);

  ibus::Parser parser;
  assert(feedFrame(parser, frame));
  assert(parser.channel(0) == 1500);
  assert(parser.channel(1) == 2000);
  assert(parser.channel(3) == 1000);
  assert(rc_input::axisFromPulse(parser.channel(0)) == 0);
  assert(rc_input::axisFromPulse(parser.channel(1)) == 100);
  assert(rc_input::axisFromPulse(parser.channel(3)) == -100);

  // A frame that has not fully arrived must never be accepted.
  ibus::Parser partial;
  bool complete = false;
  for (uint8_t i = 0; i < ibus::FRAME_SIZE - 1; ++i) complete = partial.feed(frame[i]);
  assert(!complete);
  assert(partial.channel(1) == 0);

  // A bad checksum is dropped, and the next good frame resynchronises.
  uint8_t corrupt[ibus::FRAME_SIZE];
  memcpy(corrupt, frame, ibus::FRAME_SIZE);
  corrupt[31] ^= 0xFF;
  ibus::Parser resync;
  assert(!feedFrame(resync, corrupt));
  assert(feedFrame(resync, frame));
  assert(resync.channel(1) == 2000);

  // A stray byte before the header must not desynchronise the next frame.
  ibus::Parser stray;
  assert(!stray.feed(0x00));
  assert(feedFrame(stray, frame));
  assert(stray.channel(3) == 1000);
}

static void testTelemetryLine() {
  telemetry::Snapshot snapshot{};
  snapshot.millis = 1234;
  snapshot.pulse[0] = 1500;  // CH1 strafe
  snapshot.pulse[1] = 2000;  // CH2 forward
  snapshot.pulse[2] = 1000;  // CH4 turn
  snapshot.axis[0] = 0;
  snapshot.axis[1] = 100;
  snapshot.axis[2] = -100;
  snapshot.wheel[0] = 100;
  snapshot.wheel[1] = -33;
  snapshot.wheel[2] = 33;
  snapshot.wheel[3] = 33;
  snapshot.link = true;
  snapshot.armed = false;

  char line[telemetry::LINE_CAPACITY];
  const uint8_t length = telemetry::format(snapshot, line);
  assert(length == strlen(line));
  assert(strcmp(line, "T,1234,1500,2000,1000,0,100,-100,100,-33,33,33,1,0") == 0);

  snapshot.link = false;
  snapshot.armed = true;
  telemetry::format(snapshot, line);
  assert(strcmp(line, "T,1234,1500,2000,1000,0,100,-100,100,-33,33,33,0,1") == 0);
}

int main() {
  testDriverPinMap();
  testMecanumAxes();
  testMecanumNormalization();
  testRcPulseMapping();
  testIbusParser();
  testTelemetryLine();
}
