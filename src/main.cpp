#include <Arduino.h>
#include <stdint.h>
#include <robot/Config.h>
#include <robot/Ibus.h>
#include <robot/Mecanum.h>
#include <robot/RcInput.h>
#include <robot/Telemetry.h>

// FlySky iBUS teleop on an UNO R4 Minima, no ESP32: every RC channel arrives on a
// single signal wire into Serial1 RX (D0), the board mixes mecanum and drives all
// four motor controllers. The USB serial port is output only - it streams a 20 Hz
// CSV telemetry line so the laptop can show what the wheels are actually told to do.

namespace {
constexpr uint8_t CHANNEL_COUNT = telemetry::CHANNEL_COUNT;
constexpr uint16_t MAX_BYTES_PER_LOOP = 128;  // about four iBUS frames; keeps the loop responsive

ibus::Parser receiver;
uint16_t pulses[CHANNEL_COUNT] = {0, 0, 0};  // CH1 strafe, CH2 forward, CH4 turn
int8_t axisStrafe = 0;
int8_t axisForward = 0;
int8_t axisTurn = 0;
int8_t requested[robot_config::MOTOR_COUNT] = {0, 0, 0, 0};
int8_t applied[robot_config::MOTOR_COUNT] = {0, 0, 0, 0};
bool linkOk = false;
bool armed = false;
bool centerSeen = false;
uint32_t lastValidAt = 0;
uint32_t centeredAt = 0;
uint32_t lastTelemetryAt = 0;

void writePwm(uint8_t pin, uint8_t duty) {
  analogWrite(pin, duty);
}

void applyMotor(uint8_t index, int8_t power) {
  const robot_config::MotorPins &motor = robot_config::MOTORS[index];
  int16_t signedPower = power;
  if (motor.inverted) signedPower = -signedPower;
  const uint8_t magnitude = static_cast<uint8_t>(signedPower < 0 ? -signedPower : signedPower);
  const uint8_t duty = static_cast<uint8_t>((static_cast<uint16_t>(magnitude) * 255u) / 100u);

  if (motor.mode == robot_config::DUAL_PWM) {
    // MDD3A PWM_PWM: drive only one input for a direction; zero holds both low.
    writePwm(motor.inputA, signedPower > 0 ? duty : 0);
    writePwm(motor.inputB, signedPower < 0 ? duty : 0);
    return;
  }

  // MD10C R3 sign-magnitude mode: PWM controls speed; DIR controls direction.
  digitalWrite(motor.inputB, signedPower >= 0 ? HIGH : LOW);
  writePwm(motor.inputA, duty);
}

void stopDrive() {
  for (uint8_t i = 0; i < robot_config::MOTOR_COUNT; ++i) {
    requested[i] = 0;
    applied[i] = 0;
    applyMotor(i, 0);
  }
}

void setAxes(int16_t forward, int16_t strafeRight, int16_t clockwise) {
  const mecanum::WheelPowers wheels = mecanum::mix(forward, strafeRight, clockwise);
  requested[0] = wheels.frontLeft;
  requested[1] = wheels.frontRight;
  requested[2] = wheels.rearLeft;
  requested[3] = wheels.rearRight;
}

void readReceiver() {
  // Serial1 delivers 32-byte iBUS frames at roughly 143 Hz; drain what arrived since
  // the last pass and keep the newest complete frame.
  uint16_t consumed = 0;
  while (consumed < MAX_BYTES_PER_LOOP && Serial1.available() > 0) {
    const int next = Serial1.read();
    if (next < 0) break;
    ++consumed;
    if (!receiver.feed(static_cast<uint8_t>(next))) continue;

    const uint16_t strafe = receiver.channel(robot_config::IBUS_CH_STRAFE);
    const uint16_t forward = receiver.channel(robot_config::IBUS_CH_FORWARD);
    const uint16_t turn = receiver.channel(robot_config::IBUS_CH_TURN);
    // Receiver in fail-safe (0 us), wrong channel map or a dead transmitter: keep the
    // previous axes and let RC_LINK_TIMEOUT_MS stop the motors.
    if (!rc_input::validPulse(strafe) || !rc_input::validPulse(forward) ||
        !rc_input::validPulse(turn))
      return;

    pulses[0] = strafe;
    pulses[1] = forward;
    pulses[2] = turn;
    lastValidAt = millis();
    linkOk = true;
    axisStrafe = rc_input::axisFromPulse(strafe);
    axisForward = rc_input::axisFromPulse(forward);
    axisTurn = rc_input::axisFromPulse(turn);
    setAxes(axisForward, axisStrafe, axisTurn);
  }
}

// Returns true while the receiver link is fresh. Arming is latched: the sticks have
// to rest near center once, then the wheels follow the sticks until the link drops.
// So a bad failsafe value or a bumped stick at power-on cannot make the robot drive
// off, while moving a stick afterwards still drives it.
bool updateDrive() {
  const uint32_t now = millis();
  const bool linkFresh = linkOk && now - lastValidAt <= robot_config::RC_LINK_TIMEOUT_MS;
  if (!linkFresh) {
    armed = false;
    centerSeen = false;
    stopDrive();
    return false;
  }

  if (!armed) {
    const int forwardMagnitude = axisForward < 0 ? -axisForward : axisForward;
    const int strafeMagnitude = axisStrafe < 0 ? -axisStrafe : axisStrafe;
    const int turnMagnitude = axisTurn < 0 ? -axisTurn : axisTurn;
    const bool centered = forwardMagnitude <= robot_config::ARM_CENTER_TOLERANCE &&
        strafeMagnitude <= robot_config::ARM_CENTER_TOLERANCE &&
        turnMagnitude <= robot_config::ARM_CENTER_TOLERANCE;

    if (!centered) {
      centerSeen = false;
      stopDrive();
      return true;
    }
    if (!centerSeen) {
      centerSeen = true;
      centeredAt = now;
    }
    if (now - centeredAt < robot_config::ARM_CENTER_HOLD_MS) {
      stopDrive();
      return true;
    }
    armed = true;
  }

  for (uint8_t i = 0; i < robot_config::MOTOR_COUNT; ++i) {
    applied[i] = requested[i];
    applyMotor(i, applied[i]);
  }
  return true;
}

void sendTelemetry(bool linkFresh) {
  const uint32_t now = millis();
  if (now - lastTelemetryAt < robot_config::TELEMETRY_PERIOD_MS) return;
  lastTelemetryAt = now;

  telemetry::Snapshot snapshot{};
  snapshot.millis = now;
  snapshot.pulse[0] = pulses[0];
  snapshot.pulse[1] = pulses[1];
  snapshot.pulse[2] = pulses[2];
  snapshot.axis[0] = axisStrafe;
  snapshot.axis[1] = axisForward;
  snapshot.axis[2] = axisTurn;
  for (uint8_t i = 0; i < robot_config::MOTOR_COUNT; ++i) snapshot.wheel[i] = applied[i];
  snapshot.link = linkFresh;
  snapshot.armed = armed;

  char line[telemetry::LINE_CAPACITY];
  telemetry::format(snapshot, line);
  Serial.println(line);
}
}  // namespace

void setup() {
  // Force every driver input low before the receiver or the USB console comes up.
  for (uint8_t i = 0; i < robot_config::MOTOR_COUNT; ++i) {
    const robot_config::MotorPins &motor = robot_config::MOTORS[i];
    pinMode(motor.inputA, OUTPUT);
    pinMode(motor.inputB, OUTPUT);
    digitalWrite(motor.inputA, LOW);
    digitalWrite(motor.inputB, LOW);
  }
  pinMode(robot_config::IBUS_RX_PIN, INPUT);
  Serial1.begin(robot_config::IBUS_BAUD);  // iBUS receiver on D0
  analogWriteResolution(8);  // match the 0-255 duty conversion in applyMotor().

  Serial.begin(115200);
  delay(200);  // let the USB CDC host attach; the motors are already stopped
  Serial.println(F("Mecanum RC teleop ready (iBUS on D0, telemetry only on this port)"));
  Serial.println(F("RC: CH1 strafe, CH2 forward, CH4 turn; all channels over one 115200 wire"));
  Serial.println(F("Center all sticks for 300 ms to arm; loss of signal stops the motors"));
}

void loop() {
  readReceiver();
  const bool linkFresh = updateDrive();
  sendTelemetry(linkFresh);
}
