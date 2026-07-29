# Roadmap

Status as of 2026-07-29. See `ARCHITECTURE.md` for the full rationale
behind each decision referenced here.

## Context for picking this back up

We were mid-rewrite of `ROSArduinoBridge/` when this session ended (user
is continuing in a new session/working dir). The previous in-place ESP32
port (attachInterrupt-based encoders, renamed `index` var, remapped pins)
compiled and uploaded fine, but hit a reproducible unexplained bug (see
"Known issue to watch for" below) which prompted the decision to rewrite
clean against a narrower scope instead of continuing to debug/carry
forward the full upstream feature surface.

Confirmed decisions (already agreed with user, don't re-litigate):
- Keep the plain serial text protocol — no micro-ROS.
- Support only 5 commands: `e`, `r`, `o`, `m`, `u`. Drop everything else
  from upstream (servos, sensors, analog/digital I/O, GET_BAUDRATE, ping).
- PID-update command letter is `u` (matches `commands.h`), not the `p`
  shown in README.md's example (that looks like a doc typo).
- Only L298N motor driver, only direct-GPIO ("Arduino mode") encoders.
- Real wiring pins already chosen and confirmed by user — see
  `ARCHITECTURE.md` pin table. Don't re-derive/re-guess these.

## Where the code lives

The sketch dir is now `~/firmware` itself, so the main file is
`firmware.ino` (Arduino requires the `.ino` basename to match its
directory). The old ported-in-place sketch is still at
`~/ros_arduino_bridge/ROSArduinoBridge/` and is untouched — keep it as a
fallback until the rewrite is validated on hardware.

## Steps

1. **Write the trimmed source files** — [x] done. Final layout:
   - [x] `firmware.ino` — setup/loop, serial parse/dispatch
   - [x] `config.h` — pins, rates, limits (single place to tune)
   - [x] `commands.h` — 5 command letters + LEFT/RIGHT
   - [x] `motors.h` / `motors.cpp` — L298N over LEDC PWM
   - [x] `encoders.h` / `encoders.cpp` — PCNT hardware quadrature
   - [x] `pid.h` / `pid.cpp` — per-wheel PID
   - [x] `sensors.h`, `servos.h`, `servos.ino` not carried over

2. **Compile** — [x] passes clean, 288575 bytes (22% of flash), no warnings.

   `arduino-cli compile --fqbn esp32:esp32:esp32doit-devkit-v1 ~/firmware`

3. **Before uploading**, check nothing else holds `/dev/ttyUSB0`
   (`ps aux | grep micro_ros`, or `fuser -v /dev/ttyUSB0`) — a micro-ROS
   agent grabbed it last time and caused a confusing pyserial error that
   looked like a permissions problem but wasn't. (It was not running on
   2026-07-29, so the upload went straight through.)

4. **Upload** — [x] done, hash verified.

   `arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32doit-devkit-v1 ~/firmware`

5. **Smoke-test over serial** at 57600 baud, CR line ending:
   - [x] boot banner → `# boot reset=1 encoders=ok` (reset=1 is
     `ESP_RST_POWERON`; both PCNT units configured). The garbage before
     it is the ROM bootloader, which always logs at 115200 regardless of
     our baud rate — expected, not a fault.
   - [x] `e\r` → `0 0`
   - [x] `r\r` → `OK`
   - [x] no runaway output: 3 s quiet window after commands, 0 unsolicited
     bytes (see "Known issue" below)
   - [ ] `o 50 50\r` → expect `OK`, motors spin (robot **on blocks** first
     — do not test with wheels on the ground)
   - [ ] `m 20 20\r` → expect `OK`, closed-loop PID engages
   - [ ] auto-stop: after ~2 s with no further `o`/`m`, motors stop

6. **Hardware sanity checks** (still outstanding, need the robot on blocks):
   - [ ] Wheels move forward on positive speed
   - [ ] Encoder counts increase when wheels move forward (after `r`,
     rotate a wheel ~1 turn by hand, check `e` reports a sane count).
     This is safe to do with no power to the motors.
   - [ ] **Encoder sign must agree with motor sign.** If a wheel's count
     runs negative while being driven positive, flip `LEFT_ENC_INVERT` /
     `RIGHT_ENC_INVERT` in `config.h` — otherwise the PID sees the error
     growing as it pushes and runs away to full PWM. This is the one
     failure mode to watch for on the first `m` command.
   - [ ] If the robot drives backward overall, flip `motors_reversed` on
     the ROS2 side rather than rewiring.

## Known issue to watch for (did NOT recur in the rewrite)

Status 2026-07-29: **not reproduced** on the rewritten firmware. Sending
`e`, `r`, `e` produced exactly one response line each, and a 3-second
quiet window afterward saw zero unsolicited bytes. `esp_reset_reason()`
at boot reported `1` (`ESP_RST_POWERON`), not a watchdog or panic reset,
so there is no reboot loop. Not yet exercised under motor load or with
the PID running, which is where an ISR/task interaction would have been
most likely to show up — so keep an eye out during step 5.

Note the rewrite removed the most probable cause by construction: the
encoders no longer use `attachInterrupt` at all, so there are no
GPIO ISRs running. Historical description follows.

Under the previous ported (not rewritten) firmware, sending any single
serial command (verified with `b`, bare CR, and `a0`) caused the board
to then emit a continuous, stable flood of output forever (e.g. `4095`
repeating, or `Invalid Command` repeating, or `0` repeating — always the
same value as whatever the just-processed command legitimately printed
once).

Debug instrumentation (a print of every byte read on entry to the
serial-parsing `while (Serial.available() > 0)` loop) showed only the
original command's bytes were ever read — no further passes through that
loop occurred — yet output continued indefinitely. That rules out:
- TX→RX hardware loopback being parsed as new commands (would have shown
  more bytes read, and would evolve/alternate between response strings
  rather than staying constant)
- The known serial-parsing code path being the source at all

Not yet ruled out: a silent reboot loop (was about to check
`esp_reset_reason()` printed at the very top of `setup()` when the
session was interrupted), a watchdog-triggered panic loop, or some
ISR/task interaction from the encoder `attachInterrupt` callbacks. If
this recurs in the rewritten firmware, that `esp_reset_reason()` check is
the next concrete step, along with checking `idf.py monitor`-style crash
backtraces (may need `--log-level debug` or a corefile-capable monitor
instead of a bare pyserial read loop, since a real crash/backtrace would
print to UART directly via ROM/IDF logging, bypassing Arduino's `Serial`
buffering entirely — which would also explain why our `Serial`-level
instrumentation never saw it coming).

## Open questions for next session

- Should `MAX_PWM`/`BAUDRATE` live in a shared header instead of the
  main `.ino`, now that the file count is shrinking? Minor, not blocking.
- Confirm whether GPIO12 (used for `RIGHT_MOTOR_BACKWARD`, a boot-strapping
  pin) causes any boot-time glitch in practice on this specific board —
  worth a quick empirical check (does it reset/boot reliably across
  several power cycles?).
