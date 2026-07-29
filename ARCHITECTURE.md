# Architecture

## Purpose

This repo is Arduino firmware only. Its job: receive motor speed requests
over a serial connection from a ROS2 host and report back encoder feedback.
It is not a ROS2 node itself and does not use micro-ROS. A separate ROS2
node (outside this repo) is expected to translate `/cmd_vel` and `/odom`
to/from the serial protocol below.

## Target hardware

- Board: ESP32 DOIT DevKit V1 (fqbn `esp32:esp32:esp32doit-devkit-v1`)
- Motor driver: L298N (only driver supported — Pololu/Robogaia code from
  the original upstream project is being dropped)
- Encoders: directly wired to GPIO ("Arduino encoder mode" in the original
  project), read via quadrature decoding — no external encoder-counter
  shield

### Pin assignments (as wired on the actual robot)

Encoders (`encoder_driver.h`):
- LEFT_ENC_PIN_A = 34, LEFT_ENC_PIN_B = 35
- RIGHT_ENC_PIN_A = 32, RIGHT_ENC_PIN_B = 33
- Note: GPIO34/35 are input-only (no internal pull-up available), so the
  encoders must actively drive these lines (true of typical hall-effect
  motor encoders).

Motor driver (`motor_driver.h`):
- RIGHT_MOTOR_BACKWARD = 12, LEFT_MOTOR_BACKWARD = 26
- RIGHT_MOTOR_FORWARD = 14, LEFT_MOTOR_FORWARD = 25
- RIGHT_MOTOR_ENABLE = 13, LEFT_MOTOR_ENABLE = 27

These pins avoid the ESP32's SPI-flash pins (GPIO6-11) and most
boot-strapping pins (GPIO0, 2, 15), with the exception of GPIO12
(MTDI strapping pin), which is already wired and in use — verify it
doesn't cause boot issues (it affects flash voltage selection if pulled
high at reset).

## Why a rewrite instead of porting

The original upstream sketch is AVR-only:
- Encoder ISRs use raw AVR pin-change-interrupt vectors
  (`ISR(PCINT1_vect)`, `ISR(PCINT2_vect)`) and direct register access
  (`DDRD`, `PORTC`, `PCMSK1/2`, `PCICR`) — none of which exist on ESP32.
- A local variable named `index` collided with a libc function
  `index()` pulled in by the ESP32 core's `Arduino.h`.
- Pin numbers (5,6,9,10,12,13 for motors; PD2/PD3/PC4/PC5 for encoders)
  were AVR register/pin names, several of which map to ESP32's
  SPI-flash or strapping pins if reused as plain GPIO numbers.

A first pass ported this in place (attachInterrupt-based quadrature
decode, renamed variable, remapped pins) and it compiled and uploaded
successfully. But this repo's own `README.md` documents a much smaller
surface than the full upstream feature set (see below), and the upstream
code carries a lot of `#ifdef`-driven scaffolding for hardware this fork
will never use (Pololu VNH5019/MC33926 drivers, Robogaia encoder shield,
PWM servos, generic analog/digital sensors, Ping sonar). Decision: rewrite
clean against the actual scope instead of carrying that scaffolding
forward.

## Serial protocol (kept, per README.md)

57600 baud, 8N1, commands are a single letter optionally followed by
space-separated arguments, terminated by carriage return (`\r`).

| Cmd | Args | Meaning |
|-----|------|---------|
| `e` | none | Reply with `<left> <right>` encoder counts |
| `r` | none | Reset both encoder counts to 0 |
| `o` | `<pwm_left> <pwm_right>` | Set raw PWM per motor (-255..255), bypasses PID |
| `m` | `<ticks_left> <ticks_right>` | Set closed-loop target speed in encoder ticks/PID-loop (default loop rate 30 Hz) |
| `u` | `<Kp>:<Kd>:<Ki>:<Ko>` | Update PID parameters (colon-separated, matches `commands.h`'s `UPDATE_PID='u'`, not the `'p'` shown in README.md's example, which looks like a doc typo) |

Everything else from the upstream protocol (`a`,`b`,`c`,`d`,`p`,`s`,`t`,`w`,`x`,
`GET_BAUDRATE`, PWM servos, Ping sonar, generic analog/digital I/O) is
intentionally dropped — this fork only ever used the L298N driver and
Arduino-attached encoders.

Auto-stop: if no `o`/`m` command arrives within `AUTO_STOP_INTERVAL`
(2000 ms default), motors are stopped.

## File layout (as built)

The sketch directory is `~/firmware`, so the main file must be
`firmware.ino`. Plain `.cpp`/`.h` files are used instead of extra `.ino`
files — `arduino-cli` compiles them natively, and it means real
prototypes and no reliance on the `.ino` concatenation/auto-prototype
magic.

- `firmware.ino` — setup/loop, serial command parsing/dispatch
- `config.h` — pins, baud, loop rates, PWM limits; the only file to edit
  for a different robot
- `commands.h` — the 5 command-letter `#define`s + `LEFT`/`RIGHT`
- `motors.h` / `motors.cpp` — L298N only
- `encoders.h` / `encoders.cpp` — quadrature decode
- `pid.h` / `pid.cpp` — PID loop, carried over from upstream's
  `diff_controller.h` (already platform-independent)
- Not carried over: `sensors.h` (Ping/analog sensor helpers),
  `servos.h`/`servos.ino` (PWM servo support) — unused per this fork's
  README

### Two deliberate departures from the upstream implementation

**Encoders use the ESP32's PCNT peripheral, not `attachInterrupt`.** The
chip has a dedicated pulse-counter unit that does 4x quadrature decoding
in hardware, with a configurable glitch filter. One unit per wheel, two
channels each (channel A counts both edges of the A line with B as the
direction input; channel B mirrors it). This costs zero CPU, cannot miss
counts at speed the way an ISR can, and removes GPIO ISRs from the
firmware entirely — which also happens to eliminate the most plausible
suspect for the old runaway-output bug. The hardware counter is only
16-bit, so the unit is configured with `flags.accum_count` plus watch
points on both limits, which widens it in software.

**Motor PWM uses LEDC explicitly at 20 kHz**, rather than the Arduino
core's default `analogWrite` (1 kHz, 8-bit). 1 kHz sits squarely in the
audible band and makes the motors whine. 20 kHz is above hearing while
still slow enough that the L298N's switching losses stay reasonable.
PWM is applied to the direction inputs with the enable pins held high,
matching how the robot is already wired.

## Known-good baseline

Before this rewrite, the ported-in-place version was confirmed to:
- Compile and upload successfully via `arduino-cli` to the ESP32 board
  on `/dev/ttyUSB0`
- Respond correctly to a `b` (GET_BAUDRATE) command with `57600`

It then exhibited a reproducible bug under investigation (not yet root
caused) — see `ROADMAP.md` "Known issue to watch for" — where sending
any single command triggered an endless stream of unsolicited output
afterward, with debug instrumentation showing the serial-parsing loop
was NOT being re-entered (no further bytes were being read), meaning the
runaway output was coming from somewhere else in the call graph. This
was never root-caused before the decision to rewrite; worth keeping an
eye out for a recurrence in the new code, since it may point to a real
ESP32/Arduino-core quirk (e.g. watchdog reset loop, task/ISR interaction)
rather than something specific to the old code.

## Local environment notes

- `arduino-cli` is at `/home/jetson/bin/arduino-cli`, ESP32 core 3.3.11
  already installed (`esp32:esp32`).
- Board shows up as `/dev/ttyUSB0` via a Silicon Labs CP210x USB-UART
  bridge.
- A `micro_ros_agent` (running as root, likely via systemd/docker) was
  found holding `/dev/ttyUSB0` open, which blocked `esptool` uploads with
  a generic pyserial disconnect error. Stop that service before
  compiling/uploading. (There was also a second agent instance on
  `/dev/ttyUSB1` for a different board — leave that one alone unless it's
  also in the way.)
