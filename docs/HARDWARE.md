# Hardware wiring — UNO R4 Minima + FlySky iBUS receiver

## Single board, no ESP32

There is no second controller and no UART link to another board. The UNO R4 Minima
reads the FlySky receiver's iBUS stream on one signal wire, mixes mecanum and drives
all four motor controllers itself. Its USB port is used for telemetry (and firmware
upload) only.

## Pin budget

| Signal | Pins | Notes |
|---|---|---|
| Motor outputs | D3, D4, D5, D6, D7, D9, D10, D11 | all eight motor inputs |
| FlySky iBUS signal | D0 | `Serial1` RX, 115200 baud 8N1 |
| Free | D1 | `Serial1` TX, unused |
| Free | D2, D8, D12, D13, A0–A5 | D13 is the on-board LED |

## Motor outputs

| Wheel | Driver | R4 pins | Driver input mode |
|---|---|---|---|
| FL (front left) | MDD3A channel 1 | D3 (M1A), D5 (M1B) | PWM/PWM |
| FR (front right) | MDD3A channel 2 | D6 (M2A), D9 (M2B) | PWM/PWM |
| RL (rear left) | MD10C R3 | D10 (PWM), D4 (DIR) | sign-magnitude |
| RR (rear right) | MD10C R3 | D11 (PWM), D7 (DIR) | sign-magnitude |

`D3, D5, D6, D9, D10, D11` carry speed (PWM capable), `D4` and `D7` are plain
direction outputs.

- MDD3A: 4–16 V motor supply, 3 A continuous per channel, 5 A peak; inputs M1A/M1B
  and M2A/M2B accept PWM/PWM control.
- MD10C R3: 5–30 V motor supply, up to 13 A continuous and 30 A peak for 10 seconds;
  use sign-magnitude PWM/DIR input mode.
- The shared battery must satisfy both drivers' input ranges. Check every motor's
  stall current against its driver. Never power motors from the R4's GPIO or logic rails.
- Tie all signal grounds (drivers, receiver, battery, R4). Put a physical disconnect
  in the motor battery path. Start with the wheels lifted and low speed.
- If a wheel spins the wrong way, flip its `inverted` flag in `include/robot/Config.h`.

## FlySky receiver (iBUS)

The receiver must have an iBUS port (FS-iA6B, FS-iA10B, FS-X6B…). A PWM-only receiver
such as the plain FS-iA6 cannot be used with this firmware.

```text
FlySky i-BUS / SERVO signal  --> R4 D0 (Serial1 RX)   115200 8N1
FlySky 5V                    --> shared 5V rail     (same one feeding the driver logic)
FlySky GND                   --> R4 GND (and motor battery GND)
```

The 5 V pin is already powering the motor-driver logic, but that is not a conflict: an
iBUS receiver draws only tens of mA, so tap the same rail with a Y-cable / servo
splitter (or a second wire into the same pin) and keep the ground common. Prefer a
separate 5 V BEC fed from the motor battery if you want the receiver isolated; in that
case only GND is shared. Do not power the receiver from the 3.3 V pin - most FlySky
receivers need 4.0-6.5 V.

Only one signal wire reaches the board. The receiver packs all 14 channels into one
32-byte frame (`0x20 0x40`, 14 little-endian channels, `0xFFFF - sum` checksum) and
repeats it every ~7 ms.

Firmware channel map (0-based indices in `include/robot/Config.h`):

| Function | iBUS channel | Index |
|---|---|---|
| strafe (right stick X) | CH1 | 0 |
| forward (right stick Y) | CH2 | 1 |
| turn (left stick X) | CH4 | 3 |

Bind the receiver to the transmitter first. Reverse a channel in the transmitter's own
menu if a stick drives the robot the wrong way (FlySky FS-i6: Functions → Reverse).
Endpoints stay at the default 1000/2000 µs; the firmware maps them to -100..100 with a
±50 µs deadband.

`Serial` (USB CDC) and `Serial1` (D0/D1) are separate peripherals on the R4 Minima, so
telemetry on USB and the receiver on D0 do not conflict. Disconnect the iBUS wire while
uploading if the upload tool complains.

## Behaviour and failsafes

- The UART is drained once per loop (capped at 128 bytes) and parsed by the frame
  parser in `include/robot/Ibus.h`; a frame with a bad header byte or bad checksum is
  dropped and the parser waits for the next `0x20`.
- Channel values outside 800–2200 µs (receiver in fail-safe, transmitter off, wrong
  channel map) are treated as no data. After 100 ms without a valid frame the motors
  stop and the robot must be re-armed.
- Arming requires all three sticks within ±5% of center for 300 ms, so a transmitter
  left on a stick (or a receiver fail-safe value) cannot start the robot at power-on.
- FlySky fail-safe can keep the receiver sending a preset stick position rather than
  going silent; the battery cutoff and stick centering, not this watchdog, are what
  keep the robot still in that case.

## Telemetry output

The USB serial port streams one CSV line at 20 Hz:

```text
T,<ms>,<ch1>,<ch2>,<ch4>,<strafe%>,<forward%>,<turn%>,<FL>,<FR>,<RL>,<RR>,<link>,<armed>
```

`tools/telemetry.html` reads it through the Chrome/Edge Web Serial API and shows the
receiver inputs plus the wheel PWM that was actually applied. It is read-only: the
page cannot stop the robot, and the port is not open for commands.
